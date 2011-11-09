/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3.h
 * @brief   Contains a typedef to the default 3D rotation implementation determined at compile time
 * @author  Richard Roberts
 */

// \callgraph

#pragma once


// The following preprocessor blocks select the main 3D rotation implementation,
// creating a typedef from Rot3M (the rotation matrix implementation) or Rot3Q
// (the quaternion implementation) to Rot3.  The type selected here will be
// used in all built-in gtsam geometry types that involve 3D rotations, such as
// Pose3, SimpleCamera, CalibratedCamera, StereoCamera, etc.
#ifdef GTSAM_DEFAULT_QUATERNIONS

#include <gtsam/geometry/Rot3Q.h>

namespace gtsam {
  /**
   * Typedef to the main 3D rotation implementation, which is Rot3M by default,
   * or Rot3Q if GTSAM_DEFAULT_QUATERNIONS is defined.  Depending on whether
   * GTSAM_DEFAULT_QUATERNIONS is defined, Rot3M (the rotation matrix
   * implementation) or Rot3Q (the quaternion implementation) will used in all
   * built-in gtsam geometry types that involve 3D rotations, such as Pose3,
   * SimpleCamera, CalibratedCamera, StereoCamera, etc.
   */
  typedef Rot3Q Rot3;
}

#else

#include <gtsam/geometry/Rot3M.h>

namespace gtsam {
  /**
   * Typedef to the main 3D rotation implementation, which is Rot3M by default,
   * or Rot3Q if GTSAM_DEFAULT_QUATERNIONS is defined.  Depending on whether
   * GTSAM_DEFAULT_QUATERNIONS is defined, Rot3M (the rotation matrix
   * implementation) or Rot3Q (the quaternion implementation) will used in all
   * built-in gtsam geometry types that involve 3D rotations, such as Pose3,
   * SimpleCamera, CalibratedCamera, StereoCamera, etc.
   */
  typedef Rot3M Rot3;
}

#endif
