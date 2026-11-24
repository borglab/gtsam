/* ----------------------------------------------------------------------------
 * @file WnoaInterpolator.cpp
 *
 * Note: References to (Barfoot 2024) refer to the following textbook:
 * Barfoot, Timothy D. State estimation for robotics. Cambridge University
 * Press, 2024.
 * -------------------------------------------------------------------------- */

#include "WnoaInterpolator-inl.h"

#include <gtsam/geometry/Point1.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

// ---- Explicit Instantiations ----
template class GTSAM_EXPORT Interpolator<Point1>;
template class GTSAM_EXPORT Interpolator<Point2>;
template class GTSAM_EXPORT Interpolator<Point3>;
template class GTSAM_EXPORT Interpolator<Pose2>;
template class GTSAM_EXPORT Interpolator<Pose3>;

}  // namespace gtsam
