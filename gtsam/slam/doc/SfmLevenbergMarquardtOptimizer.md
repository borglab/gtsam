# SfmLevenbergMarquardtOptimizer

`gtsam::cuda::SfmLevenbergMarquardtOptimizer` is a batch-only,
GPU-resident Levenberg-Marquardt optimizer for BAL-style bundle adjustment.
Projection-factor linearization, system assembly, the damped solve, retraction,
and error evaluation run on the GPU; the host drives the lambda search.

Supported graphs contain
`GeneralSFMFactor<PinholeCamera<Cal3Bundler>, Point3>` factors. Unit,
diagonal, isotropic, and supported robust noise models are converted by
`convertGeneralSfmGraph`. The `optimizeSfm` entry point accepts `SfmData`
directly.

The class intentionally does not inherit `NonlinearOptimizer`: the resident
pipeline provides a complete batch solve, not an independently usable
single-iteration operation.

## Solver selection

`SfmLevenbergMarquardtParams::linear.backend` selects the solver for the
camera-only Schur complement: `DenseCholesky`, `Cudss`, or `Pcg`. cuDSS
requires `GTSAM_ENABLE_CUDSS`; the other backends require only CUDA. A supplied
camera `Ordering` is expanded to scalar indices for cuDSS.

## C++ usage

```cpp
#include <gtsam/slam/cuda/SfmLevenbergMarquardt.h>

using namespace gtsam;
using namespace gtsam::cuda;

NonlinearFactorGraph graph = /* BAL-style graph */;
Values initial = /* cameras and points */;

SfmLevenbergMarquardtParams params =
    SfmLevenbergMarquardtParams::ceresDefaults();
params.setLinearSolver(LinearSolverType::DenseCholesky);

SfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
const Values& result = optimizer.optimize();
const SfmLevenbergMarquardtResult& diagnostics = optimizer.result();
```

Set `enableDetailedProfiling = true` to collect per-iteration and per-attempt
profiles. The result also records the linear-system kind, backend statistics,
transfer counts, and final optimized values.
