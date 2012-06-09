/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    simulated2DOriented
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

#include <tests/simulated2DOriented.h>

namespace simulated2DOriented {

  static Matrix I = gtsam::eye(3);

  /* ************************************************************************* */
  Pose2 prior(const Pose2& x, boost::optional<Matrix&> H) {
    if (H) *H = I;
    return x;
  }

  /* ************************************************************************* */
  Pose2 odo(const Pose2& x1, const Pose2& x2, boost::optional<Matrix&> H1,
      boost::optional<Matrix&> H2) {
    return x1.between(x2, H1, H2);
  }

/* ************************************************************************* */

} // namespace simulated2DOriented

