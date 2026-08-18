# SparseLevenbergMarquardtOptimizer

`gtsam::cuda::SparseLevenbergMarquardtOptimizer` accelerates
Levenberg-Marquardt optimization for general factor graphs whose factors
linearize to `JacobianFactor`s. Factor linearization, retraction, and trial-error
evaluation remain on the CPU. The GPU assembles and solves the damped normal
equations.

The optimizer builds a symbolic plan once from the graph topology and reuses its
CSR structure and factor write offsets on every iteration. CUDA builds support a
matrix-free PCG solver; builds with `GTSAM_ENABLE_CUDSS` also support the cuDSS
direct solver. Unsupported graphs can fall back to the standard CPU optimizer.

## C++ usage

```cpp
#include <gtsam/nonlinear/cuda/SparseLevenbergMarquardt.h>

using namespace gtsam;
using namespace gtsam::cuda;

NonlinearFactorGraph graph = /* ... */;
Values initial = /* ... */;

SparseLevenbergMarquardtParams params;
params.linear.backend = LinearSolverType::Pcg;
params.pcg.relativeTolerance = 1e-6;

SparseLevenbergMarquardtOptimizer optimizer(graph, initial, params);
Values result = optimizer.optimize();
```

`SparseLevenbergMarquardtParams` derives from
`LevenbergMarquardtParams`, so the usual convergence tolerances, lambda policy,
maximum iteration count, and iteration hook remain available.

## Solver backends

- `LinearSolverType::Cudss` forms the sparse normal matrix and uses the cuDSS
  direct solver. It requires `GTSAM_ENABLE_CUDSS`.
- `LinearSolverType::Pcg` applies the normal-equation operator through two sparse
  matrix-vector products and does not form the normal matrix. Its accuracy and
  runtime depend on the tolerance, iteration cap, and preconditioner.

Set `fallbackOnUnsupported = false` when a caller requires GPU execution instead
of CPU fallback. Set `collectTiming = true` to populate the stage timings in
`SparseLevenbergMarquardtResult`. The `backend` field reports `Device` or
`CpuFallback`; fallback details identify unsupported configurations.

The `timeCudaSparseLM` timing executable compares CPU LM with the available GPU
solver backends and checks final-objective agreement. It is built for every CUDA
configuration; cuDSS-specific options are available only when cuDSS is enabled.
