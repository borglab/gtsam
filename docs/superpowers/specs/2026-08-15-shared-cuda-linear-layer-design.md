# Shared CUDA Linear Layer Design

## Goal

Create one CUDA linear-algebra layer used by the general sparse nonlinear
optimizer and the specialized SFM optimizer. Preserve the current general
cuDSS/PCG algorithms and the current dense-Schur SFM algorithm as regression
baselines, while adding sparse-direct and implicit-PCG solves of the reduced
SFM camera system and real GTSAM-provided ordering for cuDSS.

## Scope

This design changes both CUDA nonlinear optimizers internally. It does not
make general nonlinear factor evaluation, `Values::retract`, or nonlinear
error evaluation GPU-resident. It does not require a CUDA implementation of
every GTSAM factor.

The shared boundary is a prepared linear problem

\[
  A(\lambda)x=b(\lambda),
\]

not nonlinear factor linearization and not LM trust-region policy. Each
frontend remains responsible for producing the damped problem for the current
lambda. The shared layer owns numerical backend selection, backend lifecycle,
structural analysis, solution storage, backend statistics, and solve errors.

## Current architecture

### General hybrid CUDA LM

`CudaSparseLevenbergMarquardtOptimizer` performs factor linearization on the
CPU, uploads a sparse Jacobian and residual, and delegates normal-system
construction, damping, cuDSS/PCG solving, and predicted-error evaluation to
`DeviceSparseJacobianNormalEquations`. The accepted delta is downloaded and
retraction and nonlinear error evaluation remain on the CPU.

`DeviceSparseJacobianNormalEquations` currently mixes two responsibilities:

1. general-Jacobian problem production (`J`, `J^T`, `H`, `g`, damping and
   predicted model error); and
2. linear backend ownership (`CudssSpdSolver` and `DevicePcgSolver`).

The CUDA direct path currently ignores `LevenbergMarquardtParams::ordering`;
cuDSS always performs its own ordering.

### Specialized CUDA SFM LM

`CudaSfmDenseSchurSolver::solve()` currently performs projection
linearization, Schur construction, damping, dense cuSOLVER Cholesky, and point
back-substitution in one operation. `CudaSfmLevenbergMarquardt` also contains
a separate full-normal path which directly drives `DeviceSparseNormalEquations`
and `CudssSpdSolver`.

The current SFM public enum couples formulation and backend:

- `DenseSchur`: reduced camera system plus dense cuSOLVER;
- `CudssFullNormal`: full camera-and-point system plus cuDSS.

There is no sparse reduced-camera representation, no cuDSS or PCG solve after
the custom Schur construction, and no GTSAM ordering for either cuDSS path.

## Final module ownership

### `gtsam/linear/cuda`

The shared module owns:

- dense and sparse SPD system views/storage;
- scalar permutations and block-order expansion;
- common backend options, capability validation, and solve statistics;
- persistent dense cuSOLVER, cuDSS, and PCG backend sessions;
- the PCG recurrence, reductions, convergence checks, and warm-start state;
- backend dispatch for a prepared dense system, sparse system, or linear
  operator.

The shared module must not include SFM factor headers, nonlinear factor-graph
headers, `Values`, or LM acceptance logic.

### `gtsam/nonlinear/cuda`

The general CUDA frontend owns:

- sparse-Jacobian planning and CPU streaming linearization;
- `J`/`J^T`, explicit `H`/`g`, and damping-diagonal construction;
- the matrix-free `J^T J + lambda D` PCG operator and its variable-block
  preconditioner data;
- predicted linear-model error from `J`, `b`, and delta;
- CPU/GPU transfers, CPU retraction/error, fallback, and LM policy.

It consumes the common cuDSS and PCG backend sessions rather than owning the
backend implementations.

### `gtsam/slam/cuda`

The SFM frontend owns:

- projection batches, GPU values, residuals, and projection Jacobians;
- persistent camera, point, and camera-point linearization blocks;
- dense and sparse reduced-camera system construction;
- an implicit reduced-camera Schur operator and camera-block preconditioner;
- point-delta back-substitution;
- SFM full-normal system construction;
- GPU trial retraction/error and LM policy.

It consumes all three common backends.

## Common system types

The linear layer exposes three prepared representations:

```cpp
struct CudaDenseSpdSystemView {
  int dimension;
  int leadingDimension;
  double* values;
  double* rhs;
};

struct CudaSparseSpdSystemView {
  int dimension;
  int nonzeros;
  const int* rowPointers;
  const int* columnIndices;
  double* values;
  double* rhs;
  CudaSparseTriangle triangle;
};

class CudaLinearOperator {
 public:
  virtual ~CudaLinearOperator() = default;
  virtual int dimension() const = 0;
  virtual void apply(const double* input, double* output,
                     cudaStream_t stream) const = 0;
};
```

Host-side virtual dispatch is permitted for PCG operator application. Device
kernels do not use virtual dispatch.

The backend enum is:

```cpp
enum class CudaLinearSolverType { DenseCholesky, Cudss, Pcg };
```

`CudaLinearSolverSession` owns a tagged backend instance, validates the input
representation, performs one-time setup/analysis, solves prepared numerical
systems, and reports common statistics. Dense accepts dense systems, cuDSS
accepts sparse CSR systems, and PCG accepts operators plus preconditioners.
Unsupported combinations throw before any solve.

## Dense Cholesky backend

The current `SolveDenseCameraSystemOnDevice` implementation and its
`cusolverDn` handle, workspace, and device info storage move into
`CudaDenseCholeskySolver` under `gtsam/linear/cuda`.

The SFM default continues to construct the same column-major dense reduced
camera matrix, use the same damping, invoke the same Cholesky operations, and
perform the same point recovery. Extraction must not alter the baseline
algorithm, kernel order, matrix layout, or public default.

## cuDSS backend and GTSAM ordering

`CudssSpdSolver` moves into the shared linear module and gains an optional
scalar permutation during analysis. Its CSR pattern, matrix buffers, RHS, and
solution buffers remain stable for the lifetime of one analysis session.
Numerical values may change before each factorization.

GTSAM `Ordering` contains variable keys, while cuDSS requires a scalar
permutation. `CudaBlockOrdering` compiles a strict block layout plus an
`Ordering` by appending every scalar index in each ordered block. It validates
that every expected key occurs exactly once and that the result covers
`[0, dimension)` without duplicates.

For an `int32` CSR system, the compiled host `std::vector<int>` is passed
before `CUDSS_PHASE_ANALYSIS` using:

```cpp
cudssDataSet(handle, data, CUDSS_DATA_USER_PERM,
             permutation.data(), permutation.size() * sizeof(int));
```

cuDSS copies the permutation and applies RHS/solution permutations itself.
The matrix is not physically permuted by GTSAM. The configuration must use a
cuDSS reordering mode compatible with `CUDSS_DATA_USER_PERM`. Automatic mode
does not set user permutation data. A changed permutation invalidates and
recreates symbolic analysis.

General direct solving compiles the ordering against
`SparseJacobianColumnLayout`. SFM Schur direct solving requires a camera-only
ordering compiled against 9-dimensional camera blocks. A helper constructs a
COLAMD or METIS ordering on the symbolic reduced camera graph induced by track
co-visibility. A full camera-and-point ordering is rejected for a reduced
camera system.

Custom ordering is valid only for cuDSS. Dense and PCG configurations reject
it instead of silently ignoring it.

## PCG backend

The current `DevicePcgSolver` is divided into:

1. a common PCG recurrence engine which owns recurrence vectors, reductions,
   convergence checking, warm starts, breakdown behavior, and statistics;
2. a `CudaLinearOperator` implementation; and
3. a preconditioner implementation.

The general frontend supplies `CudaJacobianNormalOperator`, applying
`J^T(Jp) + lambda Dp`, and the existing variable-block Jacobi data.

The SFM frontend supplies `CudaSfmSchurOperator`, applying

\[
 (U + \lambda D_c)p - W(V + \lambda D_p)^{-1}W^T p,
\]

without materializing the reduced matrix. Its default preconditioner is a
9-by-9 camera-block Jacobi approximation to the damped Schur diagonal.

The existing general PCG tolerance, warm-start, finite-iterate-on-breakdown,
and reporting semantics are preserved unless an explicit new parameter
changes them.

## General solver refactor

`DeviceSparseJacobianNormalEquations` is split so the general-specific object
produces and evaluates a linear problem but does not own a solver backend.
The general optimizer owns a common `CudaLinearSolverSession`.

For cuDSS, the producer forms the same explicit sparse `H` and `g`, prepares
the same damping diagonal, overwrites each diagonal from saved undamped values
for the current lambda, and passes the sparse system view to the common cuDSS
session. For PCG, it passes the general normal operator and preconditioner to
the common PCG session. Predicted model error remains a general-Jacobian
operation.

The default cuDSS and PCG paths must retain their current trajectories,
objectives, fallback behavior, transfer behavior, and timing interpretation.

## SFM Schur refactor

`CudaSfmDenseSchurSolver` is replaced by an SFM Schur problem plus the common
solver session. Projection residuals and Jacobians are computed once per outer
LM linearization and reused across lambda attempts.

The Schur problem retains persistent undamped camera blocks `U`, point blocks
`V`, camera-point blocks `W`, residual contributions, damping diagonals, and
point-recovery data.

For dense Cholesky, it produces the same dense damped reduced system and uses
the extracted dense backend.

For cuDSS, `CudaSfmReducedCsrPlan` builds a stable upper-triangular scalar CSR
pattern from track co-visibility. It stores precomputed camera-pair-to-CSR
offsets. Each lambda attempt factors damped 3-by-3 point blocks, zeroes and
fills the reduced numerical values and condensed RHS, invokes the common
cuDSS session, and back-substitutes point deltas. The CSR pattern and optional
camera ordering are analyzed once.

For PCG, the implicit Schur operator uses the persistent blocks directly,
updates lambda-dependent point inverses and preconditioner data, runs the
common PCG recurrence, and uses the same point recovery.

## SFM full-normal path

`CudaSfmSystemFormulation` contains `Schur` and `FullNormal`. Full-normal
construction remains available as a correctness/reference path and consumes
the same common cuDSS or PCG layer. It is not the default for large bundle
adjustment.

## Parameters and capability matrix

Both CUDA optimizer parameter types use GTSAM `LevenbergMarquardtParams`
semantics. SFM no longer manually owns a divergent copy of common LM fields.
The current SFM legacy and Ceres default factories remain available and map
onto the common fields.

The supported final matrix is:

| Frontend/formulation | Dense | cuDSS | PCG |
| --- | --- | --- | --- |
| General Jacobian/normal | no | yes | yes |
| SFM Schur | yes | yes | yes |
| SFM full normal | no | yes | yes |

GTSAM ordering is accepted only by cuDSS. Capability validation occurs before
allocation or graph execution.

## Profiles and results

All backends report a common `CudaLinearSolveStats` containing backend,
whether user ordering was applied, analysis/factorization/solve counts, PCG
iteration/convergence information, and backend-stage timing. Frontends retain
their own linearization, assembly, transfer, model-evaluation, retraction, and
nonlinear-error timings. Common and frontend fields must not double-count
overlapping intervals.

## Testing requirements

The implementation is not complete until all of the following are covered:

- dense, cuDSS, and PCG backend unit solutions on SPD systems;
- stable cuDSS pattern with repeated numerical updates;
- valid and invalid block-to-scalar ordering compilation;
- cuDSS automatic versus user-permutation solution equivalence and proof that
  the requested permutation was applied;
- general cuDSS and PCG pre/post-refactor trajectory and objective regression;
- SFM dense pre/post-refactor trajectory and objective regression;
- dense, sparse-direct, and PCG Schur cross-backend delta/objective parity;
- Schur versus full-normal parity on small systems;
- repeated lambda attempts without stale or accumulated damping;
- robust/non-unit SFM noise and point-recovery parity;
- CUDA boundary, tail, long-track, and compute-sanitizer coverage;
- strict unsupported-combination and build-gating behavior.

## Benchmark requirements

The general benchmark exposes cuDSS automatic ordering, cuDSS with GTSAM
ordering, and PCG. The SFM benchmark exposes dense Schur, sparse Schur with
cuDSS automatic ordering, sparse Schur with GTSAM camera ordering, implicit
Schur PCG, and full-normal controls. Output records formulation, backend,
ordering, analysis/solve counts, PCG convergence, system size, objective,
transfers, and non-overlapping top-level wall time.

## Completion criteria

The design is complete only when:

1. both optimizers consume the shared linear module;
2. the old dense SFM and general paths pass behavior-preserving regressions;
3. cuDSS receives and reports real GTSAM-derived user permutations;
4. SFM Schur supports dense, cuDSS, and implicit PCG backends;
5. full-normal SFM uses the common layer;
6. parameter, capability, profile, test, benchmark, and documentation
   requirements above are satisfied; and
7. all enabled CUDA tests, sanitizer checks, and benchmark correctness gates
   pass on the target GPU.
