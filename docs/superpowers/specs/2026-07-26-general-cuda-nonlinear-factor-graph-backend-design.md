# General CUDA Nonlinear Factor-Graph Backend Design

**Status:** Approved architecture
**Date:** 2026-07-26

## Goal

Provide a CUDA execution backend for continuous nonlinear least-squares
`NonlinearFactorGraph`s accepted by GTSAM's Levenberg-Marquardt and
Gauss-Newton optimizers. Existing GTSAM graphs and `Values` remain the public
model. Users do not translate a graph into a CUDA-specific graph API.

The backend supports:

- built-in continuous GTSAM value and factor types with registered CUDA
  implementations;
- fixed- and runtime-sized value manifolds;
- fixed- and runtime-arity factors;
- fixed- and runtime-sized residuals;
- built-in and registered custom noise models;
- ahead-of-time compiled user value, factor, and noise-model plugins; and
- ordinary explicit-Jacobian factors plus specialized smart/implicit factors.

CUDA execution is strict. Graph compilation reports every unsupported value,
factor, noise model, activation predicate, or solver representation and fails
before optimization. It never silently evaluates an unsupported factor on the
CPU.

## Non-goals

- Executing arbitrary host C++ virtual functions, `std::function` callbacks, or
  Python `CustomFactor` callbacks from a CUDA kernel.
- Supporting discrete or hybrid factors in the continuous LM/GN backend.
- Runtime translation of arbitrary C++ factor code to CUDA.
- Replacing the existing CPU GTSAM type system or changing CPU factor
  semantics.
- Requiring every specialized factor to materialize an explicit dense
  Jacobian.

## Architecture

The backend has four layers:

1. **Registry:** maps host C++ value, factor, and noise-model types to
   ahead-of-time compiled CUDA backends.
2. **Graph compiler:** validates a `NonlinearFactorGraph` and `Values`, groups
   compatible instances, and creates an immutable `CudaGraphPlan`.
3. **Graph data:** `CudaGraphData` owns the mutable numerical buffers and
   plugin-owned packed parameters described by the plan.
4. **Execution engine:** evaluates errors, linearizes, applies noise models,
   assembles or applies the linear system, solves it, and retracts values on
   the device.

The existing SFM CUDA implementation supplies proven geometry, projection,
Schur, sparse-normal-equation, cuDSS, and LM components. The general backend
must extract reusable components from it rather than make
`CudaSfmProjectionBatch` or `CudaSfmValues` the universal representation.

## Registry

The process-wide CUDA registry is populated once, normally during explicit
backend initialization. Built-in registration is deterministic:

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

Registration does not occur once per factor or once per LM iteration. A graph
compiler receives a fully initialized registry and resolves every concrete
host object through it.

User plugins export a registration function and link into the process
ahead-of-time. Registration of a C++ type means that a backend exists; it does
not imply that every runtime shape or configuration of that type is accepted.
The backend validates those details while compiling a graph.

## Value Backends

### Responsibilities

A value backend owns all CUDA knowledge for one host value family:

- identify and validate the runtime shape;
- report storage size and tangent dimension;
- pack host objects into plugin-owned device storage;
- expose device views used by factor kernels;
- implement device `retract` and `localCoordinates`;
- copy accepted values back into GTSAM `Values`; and
- test CPU/GPU manifold equivalence.

The central backend does not contain a union listing every possible GTSAM
value. It stores type-erased group handles and layouts owned by registered
value backends.

### Fixed-size values

For `Pose3`, `Point3`, `Rot3`, and similar types, the backend knows the tangent
dimension from the type:

```text
Pose3  -> tangent dimension 6
Point3 -> tangent dimension 3
Rot3   -> tangent dimension 3
```

Storage dimension and tangent dimension are separate. For example, a rotation
may use more stored scalars than its three-dimensional tangent update.

### Runtime-sized values

For `Vector`, `Matrix`, `SOn`, `ExtendedPose3d`, and user-defined dynamic
manifolds, the backend inspects each object and creates a runtime shape
descriptor.

Examples:

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

Equal tangent dimensions do not necessarily imply equal shapes or semantics.
A `2x6` matrix and a `3x4` matrix therefore receive different shape
descriptors.

Runtime shape is structural. Numerical values may change during optimization,
but vector length, matrix shape, `SOn::n`, and `ExtendedPose3d::k` remain
unchanged. Replacing a value with a different shape invalidates the plan and
requires recompilation.

### Value plan entries

The central plan records only common routing information:

```cpp
struct CudaValueSlotPlan {
  Key key;
  BackendId backend;
  RuntimeShapeId shape;
  std::size_t storageOffset;
  std::size_t tangentOffset;
  std::size_t storageScalars;
  std::size_t tangentDimension;
};
```

Backend-specific shape metadata and buffer layouts remain opaque to the central
plan. Values with compatible backend and runtime shape may share a packed
device group.

## Factor Backends

### Ordinary factor contract

An ordinary factor backend computes the factor-specific mathematics only:

```text
raw residual e
raw Jacobian H_i for every connected value i
optional active flag
```

For a factor with residual dimension `m` connected to values with tangent
dimensions `d_0 ... d_(k-1)`:

```text
e   has shape m
H_0 has shape m x d_0
...
H_i has shape m x d_i
```

The common noise layer, not each factor kernel, applies whitening, robust loss,
and constrained-noise behavior. This preserves the CPU separation between
factor mathematics and the noise model and prevents every factor port from
duplicating noise code.

A continuous least-squares `NonlinearFactor` that is not a
`NoiseModelFactor` may use `BlockJacobian` with identity noise or provide a
specialized `DirectHessian` implementation. In either case its backend must
also provide the scalar nonlinear error used for step acceptance. A custom
override of `linearize()` is therefore supported through an explicit
registered representation, not by attempting to call that host virtual method
from the device.

### Activation

Each factor batch declares:

```cpp
enum class ActivationMode {
  AlwaysActive,
  DevicePredicate,
};
```

`AlwaysActive` adds no predicate overhead. `DevicePredicate` computes an active
mask from current device values. Inactive factors contribute zero error and no
linear-system terms. The mask is numerical state and may change each
iteration without rebuilding the plan.

An arbitrary host-only override of `NonlinearFactor::active()` is unsupported
until its plugin supplies an equivalent device predicate.

### Linearization representations

The backend recognizes concrete linear representations:

```cpp
enum class LinearizationRepresentation {
  BlockJacobian,
  DirectHessian,
  ImplicitSchur,
};
```

`BlockJacobian` is the default. It carries whitened `A_i` blocks and RHS `b`
for:

```text
minimize 0.5 * ||sum_i A_i * delta_i - b||^2
```

`DirectHessian` emits local quadratic contributions:

```text
H_ij = A_i^T * A_j
eta_i = A_i^T * b
```

These contributions are scattered into an explicitly assembled global normal
system.

`ImplicitSchur` retains a compact eliminated-landmark representation and
implements the solver operations required to apply its equivalent quadratic
factor without materializing every camera-pair Hessian block.

The representation is a declared backend capability. Plan compilation verifies
that the selected linear solver supports every representation in the graph.

### Smart projection factors

A smart projection factor is a structureless, camera-key-only factor. For
current cameras `C`, it defines the reduced nonlinear cost:

```text
phi(C) = min_P sum_i reprojection_error(C_i, P, z_i)
```

The landmark `P` is triangulated internally and is not a key in `Values`.
During linearization, the factor forms:

```text
F * delta_C + E * delta_P - b
```

where `F` contains camera Jacobian blocks and `E` contains landmark Jacobians.
It whitens `F`, `E`, and `b`, then eliminates the temporary landmark update:

```text
P_cov = inverse(E^T * E)
Q = I - E * P_cov * E^T
camera Hessian = F^T * Q * F
camera RHS = F^T * Q * b
```

This is a local, per-landmark Schur complement, not the global graph's complete
Schur complement.

The CUDA backend may emit `DirectHessian` for an assembled solver or retain
`F`, `E`, `P_cov`, and `b` as `ImplicitSchur` for a compatible matrix-free
solver. It is not forced through the ordinary explicit-Jacobian path.

By contrast, `GeneralSFMFactor<Camera, Point3>` has explicit camera and
landmark keys. It uses `BlockJacobian`; the global solver may eliminate the
explicit landmark later.

### Runtime arity and residual size

A factor's host C++ type is not always a complete batch signature. The
signature also includes the runtime properties that determine control flow and
array layout:

```text
factor backend
linearization representation
activation mode
number and ordered shapes of connected values
residual dimension
factor-specific shape, such as smart-track length
noise execution shape
```

Examples:

```text
PriorFactor<Vector>, vector length 5
PriorFactor<Vector>, vector length 20

SmartProjectionPoseFactor, 4 observations
SmartProjectionPoseFactor, 11 observations
```

Equal signatures are grouped. The default is dimension/arity bucketing because
threads in a launch then follow the same loops and layouts. A custom backend
may declare a ragged representation when bucketing would create too many tiny
batches, but it owns and tests the offsets, bounds checks, and divergent path.

## Noise-Model Backends

The common noise pipeline is itself extensible. Built-in CUDA noise backends
cover GTSAM's unit, isotropic, diagonal, dense Gaussian, constrained, and
robust models. A user-defined host noise model requires a registered CUDA noise
backend.

The ordinary pipeline is:

```text
active predicate
  -> factor raw residual/Jacobians for active instances
  -> unweighted whitening
  -> robust weight/loss
  -> whitened Jacobian/RHS or scalar error
  -> assembly or operator representation
```

Error evaluation and linearization share the same residual convention and
noise semantics. Robust loss is evaluated exactly where the CPU
`NoiseModelFactor` would evaluate it.

Specialized factors use the same device noise utilities. For smart factors,
measurement Jacobians `F`, landmark Jacobians `E`, and RHS `b` are whitened
before landmark elimination.

## `CudaGraphPlan`

`CudaGraphPlan` is immutable structural metadata created by compiling a
particular graph topology and value schema. It contains:

- ordered value slots and global tangent offsets;
- type-erased value-group plans;
- ordered factor batches and their batch signatures;
- packed factor-to-value-slot connectivity;
- noise-model group plans;
- residual and linearization output offsets;
- global sparse block pattern or matrix-free operator schedule;
- solver symbolic-analysis data; and
- launch ordering and temporary-workspace requirements.

It does not own the current numerical estimate, trial estimate, residuals,
Jacobians, or changing robust weights.

The plan is reusable across LM/GN iterations and across numerical solves having
the same graph topology, registered concrete types, runtime shapes, and batch
signatures.

## `CudaGraphData`

`CudaGraphData` owns mutable numerical state described by a plan:

- current, trial, and accepted device value storage;
- the global tangent update;
- packed factor measurements and parameters;
- packed noise-model parameters;
- active masks and robust weights;
- residual, Jacobian, Hessian, RHS, and specialized-factor workspaces;
- solver numeric state; and
- device-side error and convergence scalars.

Plugin-specific arrays are held by type-erased ownership handles. The central
class does not know that one factor stores a `Pose3` measurement while another
stores a variable-length observation track.

## Graph Compilation

Compilation performs these steps:

1. Initialize or receive the process-wide registry.
2. Inspect every value, resolve its backend, validate its runtime shape, and
   assign storage and tangent offsets.
3. Inspect every factor, resolve its backend, validate key types/shapes, arity,
   residual dimension, activation support, and linearization representation.
4. Resolve and validate every noise model.
5. Ask each factor backend for its runtime batch signature.
6. Deterministically group factors by exact signature while retaining original
   factor indices for diagnostics and result correspondence.
7. Build packed connectivity and plugin-owned parameter layouts.
8. Build the global sparse pattern or matrix-free operator schedule.
9. Verify that the requested LM/GN and linear-solver configuration supports
   every batch.
10. Allocate `CudaGraphData`, pack numerical inputs, and upload them.

Preflight collects all errors before throwing. Diagnostics identify factor
index, factor type, keys, connected value types and shapes, noise-model type,
and the missing capability.

Compilation is proportional to graph size and is outside the iteration loop.
Plans may be cached by graph topology and schema identity. Packing of changed
numerical measurements remains separate from structural planning.

## Iteration Data Flow

An LM/GN iteration follows:

1. Evaluate active predicates and factor residuals at current device values.
2. Apply noise models and reduce the scalar nonlinear error.
3. Linearize factor batches.
4. Apply noise transformations and produce the declared linear
   representations.
5. Assemble the sparse normal system or prepare the matrix-free operator.
6. Apply LM damping when requested.
7. Solve for the global tangent update.
8. Ask each value backend to retract its update into trial device values.
9. Evaluate trial error.
10. Accept by swapping current/trial buffers or reject without downloading
    all values.
11. Download final accepted values only when the API requests them.

Factor evaluation, linearization, retraction, and trial evaluation remain on
the GPU. Host participation in the loop is limited to unavoidable solver/API
control and small convergence state, with device-side control or CUDA Graph
capture considered after correctness.

## User Factors and Deterministic Generation

The universal extension point is an ahead-of-time compiled batch backend. A
user factor supplies:

- host C++ factor identity;
- expected ordered value types and accepted runtime shapes;
- residual-dimension and arity rules;
- packable factor parameters and measurements;
- device residual mathematics;
- analytic device Jacobians;
- activation mode and optional device predicate;
- chosen linearization representation; and
- conformance-test sample generation.

A semantic manifest drives a deterministic generator. The generator produces:

- registry boilerplate;
- host batch grouping and validation scaffolding;
- packed structure-of-arrays layouts;
- device view types;
- kernel launch scaffolding;
- boundary and dimension checks; and
- CPU/GPU and finite-difference test harnesses.

AI may port factor-specific mathematics and analytic Jacobians into the marked
device implementation regions. AI output is not accepted based on generation
alone. Generated code is committed and must pass the same conformance gates as
handwritten code.

Complex factors may replace generated kernel scaffolding with a custom batch
implementation while preserving the same registry, validation, diagnostics,
and testing contracts.

Host-only `CustomFactor` callbacks cannot be made CUDA-compatible by
registration alone. The user supplies a separately compiled CUDA factor
backend describing equivalent mathematics.

## Correctness and Conformance

### Value tests

Every value backend must test:

- host pack -> device -> host round trips;
- CPU versus GPU `retract`;
- CPU versus GPU `localCoordinates`;
- `local(x, retract(x, delta))` consistency;
- zero, small, large, and near-singular updates appropriate to the manifold;
- every supported runtime-shape family; and
- mixed dynamic shapes in one `Values`.

### Factor tests

Every factor backend must compare:

- CPU and GPU raw residuals;
- CPU analytic and GPU analytic Jacobians;
- GPU analytic Jacobians against manifold-aware central finite differences;
- CPU and GPU active predicates;
- CPU and GPU error after each supported noise model; and
- CPU and GPU whitened linearization blocks.

For value `x_i` and tangent basis vector `u_j`, the numerical column is:

```text
[e(..., retract(x_i, +epsilon*u_j), ...)
 - e(..., retract(x_i, -epsilon*u_j), ...)] / (2*epsilon)
```

Tests use type-appropriate epsilon values and compare absolute and relative
tolerances. They cover randomized valid states plus adversarial geometric
conditions such as rotations near chart boundaries, small depth, and
near-degenerate triangulation.

### Specialized-factor tests

Smart/implicit factors additionally compare:

- triangulated point and degeneracy decisions;
- `F`, `E`, `b`, and whitening;
- explicit Hessian blocks against CPU Schur-complement blocks;
- implicit `H*x` against explicit `H*x`;
- RHS, diagonal, and scalar linearized error;
- different track lengths and observation orderings; and
- rank-deficient, cheirality, missing-parallax, and damping behavior.

### Graph tests

End-to-end tests cover:

- mixed fixed-size value and factor types;
- dynamic values and mixed runtime shapes;
- fixed- and runtime-arity factors;
- all built-in noise-model families;
- activation changes between iterations;
- one-step error, linearization, delta, and retraction equivalence;
- complete LM and GN convergence and final error;
- explicit-landmark BA and smart-factor BA;
- empty graphs, zero-factor batches, repeated keys where GTSAM permits them,
  and disconnected components; and
- strict preflight diagnostics for every unsupported category.

Numerical equivalence means matching CPU semantics within documented
floating-point tolerances, not bitwise identity.

## Performance Principles

- Compile and pack outside the iteration loop.
- Keep accepted and trial values resident on the GPU.
- Use structures of arrays and runtime-shape buckets by default.
- Fuse residual/Jacobian work only when it preserves reusable factor and noise
  boundaries.
- Reuse symbolic sparse analysis across iterations.
- Avoid per-factor kernel launches; launch per compatible batch.
- Preserve specialized Schur representations where they reduce memory or
  computation for the selected solver.
- Measure compilation, packing, factor evaluation, noise handling, assembly,
  solve, retraction, trial error, and download separately.

## Delivery Decomposition

This architecture is too large for one implementation plan. It is delivered as
ordered, independently reviewable subprojects:

1. **Core registry and graph compiler:** fixed-size value slots, plan/data
   ownership, strict preflight, and diagnostics.
2. **Ordinary factor pipeline:** `BlockJacobian`, built-in noise models,
   assembly, fixed-arity generated factor batches, and LM/GN integration.
3. **Value coverage:** device-safe fixed manifolds followed by runtime-sized
   `Vector`, `Matrix`, `SOn`, and `ExtendedPose3d`.
4. **Generator and plugin SDK:** semantic manifest, deterministic scaffolding,
   conformance harness, and one external example plugin.
5. **Runtime-arity factors:** batching signatures, arity buckets, optional
   ragged custom batches, and dynamic residual layouts.
6. **Smart and specialized factors:** `DirectHessian`, `ImplicitSchur`, smart
   triangulation, degeneracy handling, and solver compatibility.
7. **Coverage expansion and automation:** port remaining continuous built-in
   factors/value families, run generated conformance suites, and track
   unsupported registry inventory to zero for the targeted GTSAM build.

Each subproject receives its own implementation plan and review gate. The first
implementation plan covers only subproject 1 after this umbrella specification
is reviewed.

## Acceptance Criteria

The architecture is complete when:

- an unchanged supported GTSAM nonlinear graph can be compiled and optimized
  through the CUDA backend;
- unsupported content is rejected completely and diagnostically before
  optimization;
- ordinary factors share value, noise, assembly, and solver infrastructure;
- smart factors preserve landmark elimination rather than materializing an
  unsuitable universal representation;
- user plugins can add ahead-of-time compiled value, factor, and noise
  backends without editing a central type union;
- runtime-sized values and factors are represented by validated shape
  descriptors and stable plan offsets; and
- automated CPU, finite-difference, specialized-factor, and end-to-end
  conformance gates enforce correctness before generated or handwritten CUDA
  support is accepted.
