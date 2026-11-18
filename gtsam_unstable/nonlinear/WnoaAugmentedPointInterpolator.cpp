#include <gtsam_unstable/geometry/AugmentedPoint.h>
#include <gtsam_unstable/dllexport.h>
#include <gtsam/nonlinear/WnoaInterpolator-inl.h>

namespace gtsam {

template class GTSAM_UNSTABLE_EXPORT Interpolator<AugmentedPoint<1>>;
template class GTSAM_UNSTABLE_EXPORT Interpolator<AugmentedPoint<2>>;
template class GTSAM_UNSTABLE_EXPORT Interpolator<AugmentedPoint<3>>;

}  // namespace gtsam
