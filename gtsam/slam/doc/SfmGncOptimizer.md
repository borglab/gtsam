# GNC with the CUDA SFM optimizer

`GncOptimizer` can use `SfmLevenbergMarquardtOptimizer` as its inner batch
optimizer. GNC retains its CPU weight updates, continuation schedule, and
convergence logic while each weighted bundle-adjustment problem runs through
the resident CUDA SFM pipeline.

```cpp
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/slam/cuda/SfmLevenbergMarquardt.h>

using namespace gtsam;
using namespace gtsam::cuda;

NonlinearFactorGraph graph = /* BAL-style graph */;
Values initial = /* cameras and points */;

SfmLevenbergMarquardtParams lmParams =
    SfmLevenbergMarquardtParams::legacyDefaults();
GncParams<SfmLevenbergMarquardtParams> gncParams{lmParams};
gncParams.setLossType(GncLossType::TLS);

GncOptimizer<GncParams<SfmLevenbergMarquardtParams>> optimizer(
    graph, initial, gncParams);
Values result = optimizer.optimize();
Vector weights = optimizer.getWeights();
```

TLS may assign exactly zero weight to rejected factors. The CUDA SFM path
accepts zero square-root-information blocks so those factors contribute
nothing to the system. GM keeps small nonzero weights instead.

Each GNC outer iteration currently constructs a new CUDA SFM optimizer for the
reweighted graph. Reusing setup across GNC iterations is a possible future
optimization.

The supported problem class is described in
[SfmLevenbergMarquardtOptimizer](SfmLevenbergMarquardtOptimizer.md).
