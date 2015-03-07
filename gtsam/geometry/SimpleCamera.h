/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SimpleCamera.h
 * @brief A simple camera class with a Cal3_S2 calibration
 * @date Aug 16, 2009
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>

namespace gtsam {

	/// A simple camera class with a Cal3_S2 calibration
  typedef PinholeCamera<Cal3_S2> SimpleCamera;

  /// Recover camera from 3*4 camera matrix
	SimpleCamera simpleCamera(const Matrix& P);
}
