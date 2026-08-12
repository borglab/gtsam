/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   AttitudeFactor.cpp
 *  @author Frank Dellaert
 *  @brief  Implementation file for Attitude factor
 *  @date   January 28, 2014
 **/

#include "AttitudeFactor.h"

namespace gtsam {

template class AttitudeFactor<Rot3>;
template class AttitudeFactor<Pose3>;
template class AttitudeFactor<NavState>;
template class AttitudeFactor<Gal3>;
template class AttitudeFactor<Se23>;
template class AttitudeFactor<ExtendedPose3d>;

}  // namespace gtsam
