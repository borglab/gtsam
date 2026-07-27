# Performance-First General CUDA Nonlinear Optimizer Design

**Status:** Approved architecture, revision 2
**Original date:** 2026-07-26
**Revised:** 2026-07-27

## Goal

Build new GPU-first Levenberg-Marquardt and Gauss-Newton optimizers for
continuous nonlinear least-squares `NonlinearFactorGraph`s. Existing GTSAM
graphs and `Values` remain the public problem representation. A user does not
translate a graph into a CUDA-specific graph API.

The intended public entry points are:

```cpp
CudaLevenbergMarquardtOptimizer
CudaGaussNewtonOptimizer
```

They share an internal GPU nonlinear execution engine, but they do not execute
through either `CudaSparseLevenbergMarquardtOptimizer` or
`CudaSfmLevenbergMarquardtOptimizer`.

The target coverage is:

- continuous built-in GTSAM value and factor types used by LM and
  Gauss-Newton;
- fixed- and runtime-sized value manifolds;
- fixed- and runtime-arity factors;
- fixed- and runtime-sized residuals;
- built-in and registered custom noise models;
- ahead-of-time compiled user value, factor, and noise-model plugins; and
- ordinary explicit-Jacobian factors plus specialized smart, direct-Hessian,
  and implicit factors.

Arbitrary custom host code cannot execute on a GPU automatically. A user
factor becomes supported when the user supplies or generates an equivalent
ahead-of-time CUDA backend.

Correctness is a hard requirement. Subject to that requirement, iteration
speed and end-to-end optimization speed are the primary design objectives.
Code reuse, API uniformity, and implementation convenience must not impose
measurable runtime overhead.

## Non-goals

- Executing arbitrary host C++ virtual functions, `std::function` callbacks,
  or Python `CustomFactor` callbacks from CUDA kernels.
- Supporting discrete or hybrid factors in this continuous optimizer.
- Runtime translation of arbitrary C++ factor source into CUDA.
- Replacing the existing CPU GTSAM type system or changing CPU factor
  semantics.
- Making the new optimizer's internal layout compatible with the existing
  sparse CUDA optimizer when a faster representation is available.
- Requiring every specialized factor or solver to materialize a universal
  scalar CSR Jacobian.
- Preserving a component solely because it already exists.

## Relationship to Existing CUDA Optimizers

The merged `CudaSparseLevenbergMarquardtOptimizer` provides:

- a general CPU-linearized/GPU-solved reference path;
- a `StreamingSparseJacobianLinearizer` oracle;
- LM attempt traces and detailed timing infrastructure;
- tested cuDSS and matrix-free PCG implementations;
- sparse planning and structural validation examples; and
- performance baselines on BAL, Pose2, and Pose3 workloads.

The specialized CUDA SFM optimizer provides:

- proven projection and camera geometry;
- device-resident SFM value handling;
- projection batching;
- Schur and sparse-normal-equation implementation experience; and
- a specialized performance baseline.

These implementations are reference systems and component sources. They are
not runtime layers beneath the new optimizer. The new optimizer may reuse,
refactor, or replace their code based on benchmark evidence.

In particular:

- the existing scalar CSR `SparseJacobianPlan` is not the required internal
  representation;
- the CPU `StreamingSparseJacobianLinearizer` is not present in the production
  iteration loop;
- an adapter that introduces format conversions, extra launches, or additional
  memory traffic is not justified merely by code reuse; and
- existing PCG or cuDSS code may be reused only where its input representation
  and lifecycle match the new fast path.

## Performance Principles

The implementation follows these rules:

1. Compile topology, group factors, and pack immutable data outside the
   iteration loop.
2. Keep current values, trial values, deltas, factor data, solver data, and
   error scalars resident on the GPU.
3. Launch per compatible batch, never per factor.
4. Use structure-of-arrays or another measured coalesced layout by default.
5. Fuse error/Jacobian calculation, whitening, and robust weighting when
   fusion removes memory traffic or launches without duplicating semantics.
6. Cache the accepted nonlinear error. Do not recompute it unnecessarily.
7. Reuse one linearization across all LM lambda attempts for an outer
   iteration.
8. Accept a step by swapping device-buffer handles; reject it without copying
   or reconstructing values.
9. Reuse symbolic solver analysis and stable sparse/operator structure.
10. Do not force a factor or solver through scalar CSR if a block or implicit
    representation is faster.
11. Keep validation and shadow comparison out of production performance
    builds unless explicitly enabled.
12. Measure every abstraction against a direct implementation before making
    it part of the hot path.

Graph compilation latency and one-time packing matter, especially for short
solves, but they are reported separately from per-iteration execution.

## Architecture

The system has six internal layers:

1. **Registry:** maps concrete host value, factor, and noise-model types to
   ahead-of-time compiled CUDA backends.
2. **Graph compiler:** validates a graph and initial values, groups compatible
   objects, selects execution representations, and creates an immutable
   `CudaGraphPlan`.
3. **Graph data:** `CudaGraphData` owns packed immutable parameters and mutable
   device-resident numerical state.
4. **Batch execution:** value and factor backends launch error,
   linearization, retraction, and specialized operator kernels.
5. **Linear-system backend:** consumes block, assembled, or implicit
   contributions and solves the GN/LM system.
6. **Nonlinear engine:** coordinates cached error, outer linearizations,
   lambda attempts, trial evaluation, acceptance, and convergence.

The public optimizers construct this engine with LM- or GN-specific control
logic. Factor and value plugins do not implement optimizer policy.

## Registry

The process-wide CUDA registry is populated once during explicit CUDA backend
initialization. Registration happens once per concrete backend type, not once
per factor instance or optimization iteration.

```cpp
void RegisterBuiltinCudaBackends(CudaRegistry& registry) {
  registry.registerValue<Pose3>(CudaPose3ValueBackend{});
  registry.registerValue<Point3>(CudaPoint3ValueBackend{});
  registry.registerValue<Vector>(CudaDynamicVectorValueBackend{});

  registry.registerFactor<BetweenFactor<Pose3>>(
      GeneratedBatchFactory<CudaBetweenPose3>{});
  registry.registerFactor<PriorFactor<Pose3>>(
      GeneratedBatchFactory<CudaPriorPose3>{});
  registry.registerFactor<SmartProjectionPoseFactor<Cal3_S2>>(
      SmartProjectionPoseBatchFactory{});

  registry.registerNoiseModel<noiseModel::Diagonal>(
      CudaDiagonalNoiseBackend{});
}
```

User plugins export a registration function and link into the process
ahead-of-time.

Registration means a backend exists for the concrete C++ type. It does not
mean that every runtime arity, shape, parameter choice, or degeneracy policy
is supported. The graph compiler asks the backend to validate each instance.

`NonlinearFactor::sendable()` is unrelated to CUDA support. It describes
host-side parallel sendability in the CPU implementation. CUDA support is a
separate registry lookup and runtime-capability decision.

## Value Backends

### Responsibilities

A value backend owns the device behavior and layout policy for one host value
family:

- identify and validate runtime shape;
- report storage size and tangent dimension;
- choose a packed device layout;
- pack host values into current and trial device groups;
- expose efficient typed device views to factor kernels;
- implement device `retract` and `localCoordinates`;
- unpack accepted values into GTSAM `Values`; and
- provide conformance samples and tolerances.

The central engine does not contain a universal value union. It holds
type-erased group plans and ownership handles supplied by registered
backends. Device kernels remain statically typed.

### Storage and tangent dimensions

Storage dimension and tangent dimension are distinct:

```text
Pose3:
  storage representation: backend selected
  tangent dimension: 6

Point3:
  storage scalars: 3
  tangent dimension: 3

Rot3:
  storage representation: backend selected
  tangent dimension: 3
```

The backend may choose a redundant storage representation when it improves
factor or retraction kernels. Global solver columns are still assigned by
tangent dimension.

### Runtime-sized values

For `Vector`, `Matrix`, `SOn`, `ExtendedPose3d`, and user-defined dynamic
manifolds, the backend inspects each object and returns a runtime shape
descriptor.

```text
Vector(length=8)
  storage scalars: 8
  tangent dimension: 8

Matrix(rows=2, cols=6)
  storage scalars: 12
  tangent dimension: 12
  shape metadata: (2, 6)

SOn(n=5)
  storage scalars: 25
  tangent dimension: 10
  shape metadata: n=5

ExtendedPose3d(k=6)
  tangent dimension: 3 + 3*k = 21
  shape metadata: k=6
```

Equal scalar counts do not imply equal shapes or semantics. A `2x6` matrix
and `3x4` matrix therefore have different shape descriptors.

Runtime shape is structural. Numerical values may change during optimization,
but vector length, matrix shape, `SOn::n`, and `ExtendedPose3d::k` remain
fixed. Changing shape invalidates the plan.

### Value grouping and offsets

Values are grouped by compatible backend and runtime shape. Each backend may
choose its own structure-of-arrays, array-of-structures, split-coordinate, or
other representation. The choice is benchmarked for its factor access and
retraction patterns.

The common plan records routing information:

```cpp
struct CudaValueSlotPlan {
  Key key;
  BackendId backend;
  RuntimeShapeId shape;
  ValueGroupId group;
  std::size_t indexInGroup;
  std::size_t tangentOffset;
  std::size_t tangentDimension;
};
```

Backend-specific storage offsets and shape metadata remain opaque to the
central plan.

## Factor Backends

### Ordinary factor contract

An ordinary factor backend supplies factor-specific mathematics:

```text
raw residual e
raw Jacobian H_i for every connected value i
optional active predicate
```

For residual dimension `m` and connected tangent dimensions
`d_0 ... d_(k-1)`:

```text
e   has shape m
H_i has shape m x d_i
```

The backend also declares which execution forms it can produce:

```cpp
enum class LinearizationRepresentation {
  BlockJacobian,
  DirectHessian,
  ImplicitOperator,
};
```

The compiler selects a representation compatible with the requested solver
and with all batches in the graph. A factor may implement multiple
representations so the compiler can select the fastest measured combination.

### Fused semantic utilities

Noise models, robust loss, constrained behavior, and activation semantics have
shared CUDA implementations. Shared semantics do not require separate kernel
launches.

For common fixed signatures, generated factor kernels may inline:

```text
active predicate
factor residual and Jacobians
whitening
robust weight/loss
write block/operator contribution
```

This keeps factor-specific mathematics separate in source while allowing the
compiler to fuse execution. A less common dynamic noise backend may use a
separate batch or pass when that is faster than excessive specialization.

### Activation

Each batch declares:

```cpp
enum class ActivationMode {
  AlwaysActive,
  DevicePredicate,
};
```

`AlwaysActive` has no mask calculation. `DevicePredicate` evaluates activation
from current device values. Inactive factors contribute zero error and no
linear-system terms. An arbitrary host-only override of `active()` is
unsupported unless the plugin supplies equivalent device behavior.

### Runtime arity and residual size

A host C++ factor type is not a complete execution signature. The signature
also includes runtime properties that affect control flow, storage, and
specialization:

```text
factor backend
linearization representation
activation mode
ordered connected value backends and shapes
arity
residual dimension
factor-specific shape, such as track length
noise backend and execution shape
```

Examples:

```text
PriorFactor<Vector>, vector length 5
PriorFactor<Vector>, vector length 20

SmartProjectionPoseFactor, 4 observations
SmartProjectionPoseFactor, 11 observations
```

Equal signatures are grouped. Dimension and arity bucketing is the default so
threads in a launch follow identical loops and layouts. A backend may choose a
ragged offset representation if benchmarking shows that bucketing creates too
many small launches. The backend then owns bounds checks, offsets, and
divergence tests.

### Batch work assignment

One thread per factor is not a universal rule. The generated or custom backend
declares an execution policy such as:

- one thread per small scalar factor;
- one warp per medium fixed-size factor;
- one cooperative block per large or dynamic factor; or
- persistent work queues for many small heterogeneous buckets.

Launch and work-partition choices are implementation details selected by
benchmarking. They do not change the registry or conformance contracts.

## Linear-System Representations

The new optimizer does not require one universal scalar CSR Jacobian.

### Block Jacobian

Ordinary batches may store compact factor-local blocks:

```text
A_0 ... A_(k-1), b
```

with:

```text
minimize 0.5 * ||sum_i A_i * delta_i - b||^2
```

The plan records each block's connected global tangent range. A PCG backend
can apply `J*x` and `J^T*y` directly from factor-local blocks, avoiding
conversion to generic CSR when that is faster.

The exact block storage may be batch-specific and structure-of-arrays. A
solver-facing operator schedule provides the necessary routing without
requiring factor kernels to understand the complete graph.

### Assembled Hessian

For a direct sparse solver, factor batches may accumulate:

```text
H_ij = A_i^T * A_j
eta_i = A_i^T * b
```

into a precomputed block or scalar sparse pattern. The cuDSS adapter receives
the final format it needs. The assembly design must avoid a full
block-Jacobian-to-scalar-CSR conversion when direct accumulation is faster.

The compiler may select a two-stage reduction instead of atomic accumulation
when duplicate block contention dominates.

### Implicit operator

Specialized batches may retain compact state and implement:

```text
applyH(x)
accumulateRhs()
accumulateDiagonal()
linearizedModelError(delta)
```

This supports smart factors and other representations whose explicit Hessian
would be unnecessarily dense.

### Mixed graphs

A graph may contain block-Jacobian, direct-Hessian, and implicit batches.
The chosen solver backend must compose all contributions:

```text
H*x =
    block-Jacobian contributions
  + direct-Hessian contributions
  + implicit contributions
  + LM damping
```

The graph compiler rejects a solver choice that cannot represent every
registered batch.

## Smart Projection and Explicit-Landmark Factors

`GeneralSFMFactor<Camera, Point3>` has explicit camera and landmark keys. Its
ordinary linearization produces camera and landmark Jacobian blocks. The
global solver may later eliminate the explicit landmark, but it remains a
value in `Values`.

`SmartProjectionPoseFactor<Calibration>` has camera keys only. It defines:

```text
phi(C) = min_P sum_i reprojection_error(C_i, P, z_i)
```

The landmark is triangulated internally and is not a key in `Values`. During
linearization it forms:

```text
F * delta_C + E * delta_P - b
```

and eliminates the temporary landmark update:

```text
P_cov = inverse(E^T * E)
Q = I - E * P_cov * E^T
camera Hessian = F^T * Q * F
camera RHS = F^T * Q * b
```

This is a local per-landmark Schur complement.

The backend may emit assembled camera Hessian blocks or retain
`F`, `E`, `P_cov`, and `b` as an implicit operator. It is not forced through
the ordinary explicit-landmark or materialized-Jacobian path.

## Noise-Model Backends

Built-in CUDA noise backends cover unit, isotropic, diagonal, dense Gaussian,
constrained, and robust models. User-defined host noise models require a
registered CUDA backend.

The semantic order is:

```text
activation
  -> raw factor residual/Jacobians
  -> unweighted whitening
  -> robust loss and weight
  -> selected linearization representation
```

Error evaluation and linearization use the same residual and loss
conventions as the CPU factor. Smart-factor measurement and landmark
Jacobians are whitened before local elimination.

The execution schedule may fuse these steps into one factor-batch kernel or
split them when measured performance favors a split. Tests validate semantics
independently of fusion.

## `CudaGraphPlan`

`CudaGraphPlan` is immutable structural metadata compiled for a particular
graph topology, value schema, backend registry, and selected solver.

It contains:

- ordered value slots and global tangent offsets;
- type-erased value-group plans and runtime shapes;
- ordered factor batches and exact execution signatures;
- factor-to-value group connectivity;
- plugin-owned parameter and measurement layouts;
- error-reduction offsets;
- block-Jacobian, assembled-Hessian, and implicit-operator schedules;
- solver symbolic structure and capability decisions;
- launch order and stream dependencies;
- temporary workspace sizes; and
- a structural identity used for cache validation.

Its structural identity includes:

- graph keys and topology;
- concrete value, factor, and noise backend IDs;
- runtime value and factor shapes;
- batch signatures;
- selected linearization representations; and
- solver representation.

It does not own current values, trial values, changing active masks, robust
weights, residuals, Jacobian values, Hessian values, deltas, or current error.

A plan may be cached and reused for another problem instance only when the
complete structural identity matches. Numerical initial values and
measurements may be repacked without recompiling if they do not change a
runtime signature.

## `CudaGraphData`

`CudaGraphData` owns numerical state described by a plan:

- two device value sets: current and trial;
- the global device tangent update;
- packed factor measurements and parameters;
- packed noise-model parameters;
- active masks and robust weights when not fused away;
- residual, block-Jacobian, Hessian, RHS, and implicit-factor workspaces;
- solver numeric state and warm-start vectors;
- device-side error, validity, and convergence scalars; and
- type-erased plugin-owned allocations.

There is no third accepted-value buffer. The current buffer is the accepted
state. Acceptance swaps current and trial handles in constant time; rejection
leaves current unchanged.

The central class does not interpret plugin-specific arrays. It owns them
through type-erased handles with explicit stream-safe destruction.

## Graph Compilation and Instantiation

Plan compilation and numerical instantiation are separate.

### Plan compilation

Compilation:

1. Receives a fully initialized registry.
2. Resolves every value backend and validates every runtime shape.
3. Assigns global tangent offsets and compatible value groups.
4. Resolves every factor backend.
5. Validates keys, ordered value types and shapes, arity, residual dimension,
   activation, parameters that affect execution shape, and supported
   linearization representations.
6. Resolves every noise backend and noise execution shape.
7. Deterministically groups factors by exact execution signature while
   retaining original factor indices for diagnostics.
8. Builds packed connectivity and plugin-owned layout plans.
9. Evaluates solver/representation combinations and selects a compatible
   requested or automatic configuration.
10. Builds operator, sparse pattern, symbolic solver, launch, and workspace
    schedules.
11. Produces a complete structural identity.

Preflight collects all unsupported entries before throwing. Diagnostics
identify:

- factor index and concrete type;
- factor keys;
- connected value types and runtime shapes;
- noise-model type;
- requested solver;
- missing capability or unsupported runtime configuration; and
- available alternatives when another registered solver representation works.

### Numerical instantiation

Instantiation:

1. Allocates `CudaGraphData` according to the plan.
2. Packs and uploads current values.
3. Initializes trial storage.
4. Packs and uploads measurements, factor parameters, and noise parameters.
5. Creates solver numeric descriptors and reusable workspaces.
6. Evaluates or initializes the cached current error.

Updating numerical initial values or measurements reuses the plan when all
structural signatures remain unchanged.

## Optimizer Iteration

The optimized LM iteration is:

1. At the current values, launch factor batches that compute the current
   linearization. When current error is not already cached, compute error in
   the same factor pass.
2. Produce the selected block, assembled, and implicit contributions.
3. Finalize reductions, RHS, diagonal, or preconditioner data that is shared
   by all lambda attempts.
4. For each LM lambda attempt:
   1. apply or parameterize damping;
   2. solve for device-resident `delta`;
   3. check delta validity on the device;
   4. retract current values into trial values;
   5. evaluate trial nonlinear error on the device;
   6. transfer only the minimum control scalars required by optimizer policy;
   7. accept by swapping current/trial handles and caching trial error, or
      reject without copying values.
5. Test termination.
6. Linearize the new accepted state only when another outer iteration is
   required.
7. Materialize final GTSAM `Values` once when requested.

GN follows the same path without the LM lambda-attempt loop and damping
policy.

The initial implementation may use host-side LM control with small scalar
transfers. Device-side control and CUDA Graph capture are later optimizations
and are adopted only when contemporaneous A/B measurements show an
end-to-end gain.

## Linear-System Backend Contract

The internal solver interface is capability-based rather than
format-prescriptive. Conceptually it provides:

```cpp
class CudaLinearSystemBackend {
 public:
  SolverCapabilities capabilities() const;
  SolverPlan compile(const CudaGraphPlanInput&) const;
  void prepareOuterLinearization(
      const CudaGraphPlan&, CudaGraphData&, cudaStream_t);
  DeviceSolveResult solve(
      const CudaGraphPlan&, CudaGraphData&, double lambda, cudaStream_t);
};
```

Concrete implementations may consume different data:

- block-operator PCG;
- assembled-Hessian cuDSS;
- specialized Schur PCG;
- future iterative or direct backends.

The actual interface may use typed internal classes rather than this exact
type-erased surface. The contract is the required behavior, not an obligation
to introduce virtual dispatch in the hot path.

### PCG reuse policy

The existing PCG implementation contributes tested behavior:

- block-Jacobi preconditioning;
- warm starts across lambda attempts;
- residual and finite-value verification;
- controlled host convergence checks;
- breakdown handling; and
- iteration accounting.

Its scalar CSR `J`/`J^T` operator is reused only if it remains competitive
with a batch-block operator. If scalar CSR becomes a conversion or memory
bandwidth tax, the algorithm and tests are retained while its storage and
operator kernels are replaced.

### cuDSS reuse policy

The existing cuDSS lifecycle and error handling may be reused when the new
assembled Hessian matches the required input format. Symbolic analysis remains
cached.

The new factor pipeline should accumulate directly into the chosen Hessian
representation when that outperforms materializing and transposing a global
Jacobian. Any conversion introduced to satisfy the old wrapper must be
benchmarked against a direct adapter.

## Strict Execution and Fallback

The new optimizers use strict full-GPU execution by default:

- the complete graph is preflighted before optimization;
- every unsupported value, factor, noise model, activation predicate, runtime
  shape, and solver representation is reported;
- unsupported content causes compilation to fail; and
- individual unsupported factors never silently execute on the CPU.

The existing `CudaSparseLevenbergMarquardtOptimizer` may remain available as a
separate hybrid/reference optimizer. It is not an automatic hidden fallback
inside the new optimizer.

An explicitly named development shadow mode may evaluate CPU reference work
for validation. It is disabled in normal and benchmark execution.

## User Plugins and Deterministic Generation

The universal extension point is an ahead-of-time compiled batch backend.
A user factor supplies:

- host C++ type identity;
- ordered expected value types and accepted runtime shapes;
- residual-dimension and arity rules;
- packable parameters and measurements;
- device residual mathematics;
- analytic device Jacobians or a specialized linear representation;
- activation mode and optional device predicate;
- supported solver representations; and
- conformance sample construction.

A semantic manifest drives a deterministic generator. The generator produces:

- registry boilerplate;
- runtime validation and diagnostics;
- batch signatures and grouping scaffolding;
- packed structure-of-arrays layouts;
- typed device views;
- error and linearization launch wrappers;
- common noise and activation integration;
- solver-routing metadata;
- boundary and dimension checks;
- CPU/GPU comparison tests;
- finite-difference tests; and
- registry-coverage metadata.

AI supplies or ports the semantic portions:

- factor residual mathematics;
- analytic Jacobians;
- parameter extraction;
- value manifold primitives when adding a value backend;
- valid randomized samples;
- adversarial geometric samples; and
- factor-specific degeneracy behavior.

AI does not generate the correctness oracle. Tests call the original CPU
GTSAM value and factor implementations and use a generic numerical derivative
harness. Generated code is committed source code, reviewed normally, and
accepted only after all conformance and performance gates pass.

Complex factors may replace the generated kernel body or complete batch
implementation while preserving the registry, validation, diagnostic,
solver-capability, and testing contracts.

Host-only `CustomFactor` callbacks require a separately compiled CUDA backend
that describes equivalent mathematics.

## Correctness and Conformance

Correctness tests run independently of the production hot path. Debug shadow
validation is optional and disabled for performance measurements.

### Value tests

Every value backend tests:

- host pack -> device -> host round trips;
- CPU versus GPU `retract`;
- CPU versus GPU `localCoordinates`;
- `local(x, retract(x, delta))` consistency;
- zero, small, large, and near-singular updates appropriate to the manifold;
- every supported runtime-shape family;
- mixed dynamic shapes in one `Values`; and
- both current/trial buffer directions and accepted-buffer swaps.

### Ordinary factor tests

Every ordinary backend compares:

- CPU and GPU raw residuals;
- CPU analytic and GPU analytic Jacobians;
- GPU analytic Jacobians against manifold-aware central finite differences;
- CPU and GPU active predicates;
- CPU and GPU nonlinear error for every supported noise backend;
- CPU and GPU whitened blocks and RHS; and
- single-factor and batched results.

For value `x_i` and tangent basis vector `u_j`, the numerical column is:

```text
[e(..., retract(x_i, +epsilon*u_j), ...)
 - e(..., retract(x_i, -epsilon*u_j), ...)] / (2*epsilon)
```

Tests use type-appropriate epsilon and combined absolute/relative tolerances.
They cover deterministic randomized valid states plus adversarial geometry,
such as chart boundaries, small depth, and near-degenerate configurations.

The three comparisons are deliberately independent:

1. CPU analytic versus CPU numerical behavior.
2. GPU analytic versus CPU analytic behavior.
3. GPU analytic versus numerical behavior evaluated through GPU retraction
   and residual kernels.

This prevents a shared GPU retraction/Jacobian error from passing through
mutual cancellation.

### Batch and packing tests

For `BlockJacobian` batches, tests compare the complete GPU batch result with
the current CPU `StreamingSparseJacobianLinearizer` or an equivalent
independent CPU assembly:

- original factor index to batch index mapping;
- key order and global tangent offsets;
- residual row ownership;
- every Jacobian coefficient and RHS entry;
- whitening and robust scaling;
- inactive and null factor behavior; and
- mixed factor types and shapes.

This oracle is a test tool. It is never invoked by the new optimizer's
production iteration loop.

### Direct and implicit tests

Direct-Hessian and implicit backends compare:

- local and global Hessian block contributions;
- RHS and diagonal;
- scalar linearized model error;
- implicit `H*x` against explicit CPU `H*x`;
- mixed-representation `H*x`;
- damping behavior; and
- solver results against explicit reference systems.

Smart factors additionally compare:

- triangulated point and degeneracy decisions;
- `F`, `E`, `b`, and whitening;
- explicit Schur blocks;
- different track lengths and observation orderings;
- rank-deficient and missing-parallax cases;
- cheirality behavior; and
- factor-specific damping or regularization.

### Optimizer tests

End-to-end tests cover:

- mixed fixed-size values and ordinary factor types;
- dynamic values and mixed runtime shapes;
- fixed- and runtime-arity factors;
- all built-in noise-model families;
- activation changes between iterations;
- one-step error, linearization, delta, retraction, and acceptance;
- complete LM and GN convergence and final error;
- explicit-landmark and smart-factor bundle adjustment;
- empty graphs and disconnected components;
- zero-sized batches created by filtering;
- repeated keys where GTSAM permits them;
- strict preflight diagnostics for every unsupported category; and
- plan reuse with changed numerical values and measurements.

For deterministic small graphs, the GPU attempt trace is compared with the CPU
optimizer and existing sparse CUDA reference:

```text
outer linearization number
lambda
delta
linearized model change
trial nonlinear error
model fidelity
accept/reject decision
termination reason
```

Large performance problems compare convergence and final objective within
documented tolerances rather than requiring identical floating-point
trajectories.

### Tooling

CUDA conformance includes:

- `compute-sanitizer` memory and race checks on focused tests;
- deterministic seed recording;
- finite-value checks for every emitted representation;
- debug bounds checking for ragged batches;
- tests with non-multiple launch sizes and empty tails; and
- CPU-only build coverage that excludes the CUDA optimizer cleanly.

## Performance Validation

Correct code is not accepted into the production path without performance
evidence.

### Baselines

The baseline suite includes:

- the merged `CudaSparseLevenbergMarquardtOptimizer`;
- the specialized CUDA SFM optimizer where applicable;
- CPU GTSAM LM/GN; and
- direct handwritten reference kernels for new batch abstractions.

The initial multidomain benchmark set remains:

- Dubrovnik/BAL small;
- Dubrovnik/BAL large;
- Pose2 large graph; and
- Pose3 sphere.

Coverage expands as value and factor families are added.

### Measurements

Report separately:

- registry initialization;
- graph compilation and grouping;
- allocation and initial packing;
- factor error and linearization;
- whitening and robust handling;
- operator or Hessian construction;
- preconditioner construction;
- solve;
- retraction;
- trial error;
- accept/reject control;
- final unpack;
- kernel launch count;
- host-device transfer bytes; and
- temporary and persistent device memory.

Compilation is amortized separately from steady-state iteration time.

### Acceptance rules

- Use contemporaneous interleaved A/B measurements.
- Do not accept a conversion or abstraction based only on microbenchmarks if
  end-to-end time regresses.
- Do not sacrifice objective/convergence tolerances to claim speedup.
- Treat statistically meaningful iteration-time regression against an
  equivalent direct implementation as a design problem.
- Allow a slower path for a rare dynamic feature only when it does not burden
  common fixed-size batches.
- Keep expensive validation, tracing, and profiling opt-in.
- Record negative results so failed optimizations are not repeated unchanged.

## Coverage Automation

The repository maintains a generated coverage inventory for the targeted
GTSAM build:

```text
host value/factor/noise type
registered CUDA backend
supported runtime signatures
supported linearization representations
supported solver backends
conformance test status
performance test status
known unsupported configurations
```

The AI-assisted porting loop is:

1. Select one uncovered built-in family.
2. Inspect its CPU semantics, charts, Jacobians, noise behavior, and edge
   cases.
3. Produce or update the semantic manifest.
4. Generate deterministic scaffolding and tests.
5. Port the device mathematics and analytic Jacobians.
6. Run value/factor/batch/numerical conformance.
7. Run sanitizer checks.
8. Benchmark against CPU and existing GPU references.
9. Commit only after required gates pass.
10. Update the generated coverage inventory.

The generator removes repetitive registration, packing, batching, and testing
work. It is not a DSL that reimplements arbitrary C++ or GTSAM.

## Delivery Decomposition

This umbrella architecture is too large for one implementation plan. Work is
split into independently reviewed subprojects.

### 0. Preserve reference systems

- Keep the merged sparse optimizer and specialized SFM optimizer working.
- Freeze correctness, attempt-trace, transfer, and performance baselines.
- Add no new runtime dependency from them to the new optimizer.

### 1. New optimizer core and value runtime

- Add the new public LM entry point and shared internal engine skeleton.
- Implement the registry and strict preflight framework.
- Implement `CudaGraphPlan`/`CudaGraphData` ownership and structural identity.
- Implement fixed-shape value groups, double-buffered device values, tangent
  routing, pack/unpack, and device retraction.
- Use minimal Rn and Pose value backends to validate the interfaces.

This is the first implementation plan after the umbrella spec is reviewed.

### 2. Error batches and nonlinear loop

- Define the ordinary factor error ABI.
- Add fused noise/robust utilities.
- Implement device error reduction.
- Implement device trial evaluation, pointer-swap acceptance, and cached
  current error.
- Validate the full device-resident LM control flow before broad factor
  coverage.
- Add the GN public entry point as the zero-damping policy over the same
  execution engine.

### 3. Linearization and solver integration

- Implement block-Jacobian batches.
- Define the capability-based solver contract.
- Benchmark block-operator PCG against the current scalar-CSR PCG.
- Integrate or adapt cuDSS through direct Hessian assembly.
- Support multiple lambda attempts without relinearization or value download.
- Port only the minimal factor set needed for the four reference benchmarks.

### 4. Generator and plugin SDK

- Stabilize the semantic manifest after reference backends prove the ABI.
- Generate registration, packing, batch, launch, and conformance scaffolding.
- Provide one external user value/factor plugin example.
- Add the generated coverage inventory.

### 5. Ordinary built-in coverage

- Port fixed-size built-in value and factor families.
- Add runtime-sized `Vector`, `Matrix`, `SOn`, `ExtendedPose3d`, and related
  factors.
- Add runtime arity and residual-size buckets.
- Use custom ragged batches only where measured launch fragmentation warrants
  them.

### 6. Specialized representations

- Implement `DirectHessian` and `ImplicitOperator`.
- Add smart projection triangulation and degeneracy policies.
- Compose block, direct, and implicit contributions in compatible solvers.
- Add other built-in factors with specialized `linearize()` behavior.

### 7. Coverage and performance completion

- Drive the targeted built-in unsupported inventory toward zero.
- Tune layouts, launch policies, fusion, assembly, preconditioners, and solver
  selection.
- Evaluate CUDA Graph capture and device-side optimizer control after the
  dominant kernels and launch schedule stabilize.

Each subproject receives its own implementation plan, correctness gate,
performance gate, and review.

## Acceptance Criteria

The architecture is complete when:

- unchanged supported GTSAM nonlinear graphs can be optimized through the new
  CUDA LM and GN entry points;
- the production nonlinear iteration contains no CPU factor evaluation,
  retraction, `Values` reconstruction, or per-attempt delta download;
- accepted and trial values remain device-resident and acceptance is a buffer
  swap;
- unsupported content is rejected completely and diagnostically before
  optimization;
- ordinary, direct, and implicit factors can coexist when the selected solver
  advertises the required capabilities;
- built-in and user backends can be added without editing a central value or
  factor union;
- runtime-sized values and factors use validated stable shape descriptors;
- generated and handwritten backends pass CPU, analytic, finite-difference,
  batch, solver, sanitizer, and end-to-end conformance;
- test and validation machinery adds no overhead to normal benchmark
  execution;
- PCG, cuDSS, sparse planning, and other existing code are reused only where
  benchmark evidence shows no material reuse tax; and
- performance reports demonstrate the new optimizer against the merged sparse
  CUDA optimizer, specialized CUDA implementations, and CPU GTSAM without
  relaxing the declared correctness tolerances.
