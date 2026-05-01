/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ExtendedPose3.cpp
 * @brief   Explicit instantiations for ExtendedPose3<K>
 * @author  Frank Dellaert, et al.
 */

#include <gtsam/geometry/ExtendedPose3.h>

namespace gtsam {

template class GTSAM_EXPORT ExtendedPose3<1>;
template class GTSAM_EXPORT ExtendedPose3<2>;
template class GTSAM_EXPORT ExtendedPose3<3>;
template class GTSAM_EXPORT ExtendedPose3<4>;
template class GTSAM_EXPORT ExtendedPose3<6>;

}  // namespace gtsam
