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

`SfmLevenbergMarquardtParams::formulation` and `linear.backend` are independent:

| Formulation | Backends |
|---|---|
| `Schur` | `DenseCholesky`, `Cudss`, or `Pcg` |
| `FullNormal` | `Cudss` or `Pcg` |

cuDSS backends require `GTSAM_ENABLE_CUDSS`. Dense Cholesky and PCG require only
a CUDA build. A supplied `Ordering` is expanded to scalar indices for cuDSS.

## C++ usage

```cpp
#include <gtsam/slam/cuda/SfmLevenbergMarquardt.h>

using namespace gtsam;
using namespace gtsam::cuda;

NonlinearFactorGraph graph = /* BAL-style graph */;
Values initial = /* cameras and points */;

SfmLevenbergMarquardtParams params =
    SfmLevenbergMarquardtParams::ceresDefaults();
params.formulation = SfmSystemFormulation::Schur;
params.linear.backend = LinearSolverType::DenseCholesky;

SfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
const Values& result = optimizer.optimize();
const SfmLevenbergMarquardtResult& diagnostics = optimizer.result();
```

Set `enableDetailedProfiling = true` to collect per-iteration and per-attempt
profiles. The result also records the selected formulation, linear-system kind,
backend statistics, transfer counts, and final optimized values.

See [SfmGncOptimizer](SfmGncOptimizer.md) for robust optimization using this
batch optimizer as the GNC inner solver.
