# Direct Sparse Jacobian GPU Prototype Design

## Goal

Add a general batch Levenberg-Marquardt prototype that accepts an ordinary
`NonlinearFactorGraph` and `Values`, keeps GTSAM's existing CPU/TBB factor
linearization semantics, avoids constructing a complete `GaussianFactorGraph`,
assembles a reusable scalar CSR Jacobian and right-hand side, forms normal
equations on the GPU, and solves them with the existing cuDSS wrapper.

The prototype tests a narrow research hypothesis:

> A cached, race-free sparse-Jacobian layout can remove domain-specific graph
> conversion and repeated sparse-layout construction while retaining broad
> factor compatibility and existing CPU-parallel linearization.

## Non-goals

- GPU nonlinear-factor evaluation.
- GPU retraction or GPU-resident `Values`.
- iSAM2 or incremental Bayes-tree updates.
- Sparse QR or exact constrained-noise support.
- Eliminating every temporary `GaussianFactor` in the first implementation.
- Replacing GTSAM's existing CPU optimizers or changing their default behavior.
- Supporting topology, variable-dimension, or ordering changes after plan
  construction.

## User-facing behavior

The prototype exposes an opt-in CUDA LM optimizer with the standard graph and
values constructor shape. Existing graph construction code is unchanged.

```cpp
CudaSparseLevenbergMarquardtParams params;
CudaSparseLevenbergMarquardtOptimizer optimizer(graph, initial, params);
Values result = optimizer.optimize();
```

Unsupported linear factors or constrained rows cause a clean fallback to the
existing GTSAM LM path for the complete solve. The prototype never silently
changes factor semantics.

## Key distinction: layout versus elimination ordering

The prototype does not require a user-supplied fill-reducing `Ordering`.
However, every symbolic GTSAM `Key` must still map to a contiguous scalar
column range in the CSR Jacobian and downloaded delta.

The initial implementation uses the natural key/dimension sequence from the
input `Values` and stores it as `SparseJacobianColumnLayout`. This is a storage
layout, not a promise about cuDSS's internal reordering. cuDSS remains
responsible for fill-reducing reordering during analysis.

## Architecture

### 1. SparseJacobianColumnLayout

This immutable object owns:

- ordered keys;
- tangent dimension for each key;
- scalar starting column for each key;
- total tangent dimension;
- conversion from a dense downloaded delta to `VectorValues`.

It is built once from `Values::dims()`. Duplicate or missing keys, zero
dimensions, and dimensions too large for the selected CSR index type are
reported before CUDA allocation.

### 2. SparseJacobianPlan

This immutable symbolic plan is built once from the nonlinear graph and column
layout. For each factor index it stores:

- reserved scalar row range;
- expected residual dimension;
- participating keys and their expected tangent dimensions;
- the sorted global scalar columns occupied by each dense local Jacobian block;
- exact offsets into the CSR numerical values array;
- whether the factor may run in the TBB pass (`sendable()`).

The global plan owns:

- CSR row offsets for the rectangular Jacobian;
- CSR column indices;
- factor write plans;
- total rows, columns, and stored scalar entries;
- a structural fingerprint used to reject incompatible reuse.

Each factor receives exclusive CSR rows. Therefore factor-to-Jacobian packing
requires no atomics, locks, coloring, or thread-local global matrices.

The first prototype assumes an ordinary factor's local Jacobian is dense over
each key it touches. Compact `BatchJacobianFactor` row sparsity is not expanded
through the union of all batch keys; such factors use the compatibility path
described below and may initially trigger CPU fallback if their exact row
structure cannot be represented by the plan.

### 3. StreamingSparseJacobianLinearizer

The numeric linearizer mirrors the scheduling semantics of
`NonlinearFactorGraph::linearize()`:

1. A TBB parallel loop processes non-null factors with `sendable() == true`.
2. A serial loop processes non-sendable factors, including Python-backed custom
   factors.
3. Each factor writes only its preassigned CSR value range and RHS rows.

For minimal code and semantic safety, the first implementation calls the
factor's existing virtual `linearize(values)` method, immediately consumes the
returned factor, and destroys it. It never stores the returned objects in a
`GaussianFactorGraph`.

Supported numeric input in the first implementation is:

- `JacobianFactor` with a null, unit, or non-constrained diagonal noise model;
- a null result from an inactive factor, represented by zeroing its reserved
  rows;
- compact Jacobian factors only when their exact row structure can be copied
  without expanding structural zeros.

The linearizer validates returned keys, dimensions, and row count against the
symbolic plan on every call in debug/test builds. A Hessian-only, implicit,
constrained, or structurally changed result reports `UnsupportedDirectJacobian`
and selects the ordinary GTSAM optimizer.

This design deliberately retains temporary per-factor Gaussian objects in the
first milestone. A later, separately benchmarked optimization may add an
opt-in direct writer for factors that can safely emit whitened blocks and RHS
values without overriding `linearize()` semantics.

### 4. HostSparseJacobian

The reusable host numeric storage contains:

- pinned `double` CSR values for `J`;
- pinned `double` RHS values for `b`;
- immutable pageable row offsets and column indices owned by the plan.

The value and RHS buffers are zeroed at the start of each outer LM iteration.
Their addresses and sizes remain stable across iterations. Timing separates
factor linearization, numerical packing, zeroing, and H2D transfer.

### 5. DeviceSparseJacobianNormalEquations

The device component owns persistent buffers and descriptors for:

- CSR `J`;
- CSR representation of `J^T`;
- full CSR `H = J^T J` for the first implementation;
- dense vectors `b`, `g = J^T b`, `delta`, and the LM damping diagonal;
- cuSPARSE transpose, SpMV, and SpGEMM workspaces;
- cuDSS matrix, analysis, factorization, and solve state.

On the installed CUDA 13.0 toolkit, generic SpGEMM accepts non-transposed CSR
operands. The implementation therefore creates the `J^T` structure once and
updates its values with `cusparseCsr2cscEx2`, then computes:

```text
H = J^T * J       using cuSPARSE SpGEMM
g = J^T * b       using cuSPARSE SpMV
```

The first correctness implementation may use the ordinary SpGEMM sequence.
CUDA 13.0's `cusparseSpGEMMreuse` path is evaluated behind the component
boundary for fixed-pattern repeated numerical updates; no public GTSAM API
depends on that CUDA-version-specific choice.

cuDSS receives the square sparse normal system, not the rectangular Jacobian.
Analysis is reused only while the Hessian CSR structure and device pointers are
unchanged.

### 6. LM iteration

One outer LM iteration performs:

1. CPU/TBB streaming linearization and CSR numeric fill.
2. Asynchronous upload of `J.values` and `b`.
3. Device update of `J^T` values.
4. Device computation of the undamped `H`, `g`, diagonal, and quadratic model
   constant needed for predicted reduction.
5. For each lambda attempt, restore the undamped diagonal, add damping,
   numerically factor with cuDSS, solve for `delta`, and evaluate the linearized
   model change without rebuilding `J` or `H`.
6. Download `delta`, convert it with the column layout, and call
   `Values::retract` on CPU.
7. Evaluate nonlinear trial error through the existing graph API on CPU.
8. Accept or reject using GTSAM-compatible LM rules.

The prototype initially favors correctness and an explicit timing breakdown
over fusing these stages.

## Fallback and error handling

The CUDA path is selected only when all of the following hold:

- CUDA and cuDSS support are compiled and available at runtime;
- the graph topology and value dimensions match the plan;
- every active linearized result can be represented as supported Jacobian rows;
- no constrained noise model requires QR semantics;
- problem dimensions and CSR counts fit the chosen index widths.

Otherwise the optimizer records the reason and runs ordinary GTSAM LM from the
same input graph and values. CUDA errors, failed cuDSS analysis/factorization,
and non-finite systems are surfaced with stage context; they do not return a
partially updated result.

## Correctness requirements

For supported graphs and every outer iteration:

- CSR `J` and `b`, converted to dense form in tests, match the concatenation of
  the corresponding whitened GTSAM `JacobianFactor` rows;
- device `H` and `g` match a CPU `J^T J` and `J^T b` reference;
- damping and predicted reduction match GTSAM LM conventions;
- downloaded delta matches the CPU Cholesky solution within configured
  numerical tolerance;
- accepted/rejected LM steps and final objective remain consistent with the
  CPU optimizer;
- non-sendable factors execute exactly once on the serial calling thread;
- inactive factors produce zero reserved rows without changing the pattern.

Tests cover unary, binary, and ternary factors; heterogeneous tangent and
residual dimensions; arbitrary keys; unit, diagonal, and robust noise;
non-sendable custom factors; inactive factors; unsupported constraints; and
structural-mismatch fallback.

## Performance requirements and measurements

Every benchmark reports separately:

- CPU factor evaluation plus temporary-factor creation;
- direct CSR packing;
- H2D bytes and copy time;
- `J^T` value update;
- GPU `J^T J` and `J^T b` time;
- cuDSS analysis, factorization, and solve;
- D2H delta and CPU retract;
- nonlinear trial-error evaluation;
- end-to-end optimizer time.

The baseline comparisons are:

1. ordinary GTSAM LM;
2. existing CUDA SFM-specialized LM on BAL;
3. the generic direct-sparse-Jacobian CUDA prototype;
4. a diagnostic path that constructs a `GaussianFactorGraph` and then packs the
   same CSR layout.

The prototype is considered informative even if it does not beat the
specialized CUDA kernels. It succeeds as a research milestone if it establishes
whether Gaussian-object allocation, domain conversion, sparse packing, GPU
normal-equation formation, or sparse factorization is the limiting general
path.

## Deliberate follow-up boundary

Only after the timing breakdown is trustworthy will the project consider:

- direct `NoiseModelFactor` emission without temporary `JacobianFactor` objects;
- compact block-row Jacobian storage;
- upper-triangle-only normal-equation generation;
- custom GPU factor emitters;
- persistent device values;
- incremental/iSAM2 integration.

These are not required to validate the first prototype's hypothesis.

## CUDA API references

- NVIDIA cuSPARSE 13.0 documentation for CSR/CSC conversion, SpMV, and SpGEMM:
  <https://docs.nvidia.com/cuda/archive/13.0.3/cusparse/>
- NVIDIA cuDSS documentation for square CSR systems and SPD Cholesky:
  <https://docs.nvidia.com/cuda/cudss/>
