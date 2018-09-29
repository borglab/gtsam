/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ISAM2-inl.h
 * @brief
 * @author Richard Roberts
 * @date Mar 16, 2012
 */

#pragma once

#include <gtsam/nonlinear/ISAM2.h>

namespace gtsam {

/* ************************************************************************* */
template <class VALUE>
VALUE ISAM2::calculateEstimate(Key key) const {
  const Vector& delta = getDelta()[key];
  return traits<VALUE>::Retract(theta_.at<VALUE>(key), delta);
}

/* ************************************************************************* */

}  // namespace gtsam
