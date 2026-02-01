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

#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace gtsam {

  /// Convenient aliases for Pinhole camera classes with different calibrations.
  /// Also needed as forward declarations in the wrapper.
  using PinholePoseCal3_S2 = gtsam::PinholePose<gtsam::Cal3_S2>;
  using PinholePoseCal3Bundler = gtsam::PinholePose<gtsam::Cal3Bundler>;
  using PinholePoseCal3DS2 = gtsam::PinholePose<gtsam::Cal3DS2>;
  using PinholePoseCal3Unified = gtsam::PinholePose<gtsam::Cal3Unified>;
  using PinholePoseCal3Fisheye = gtsam::PinholePose<gtsam::Cal3Fisheye>;
  using PinholeCameraCal3_S2 = gtsam::PinholeCamera<gtsam::Cal3_S2>;
  using PinholeCameraCal3Bundler = gtsam::PinholeCamera<gtsam::Cal3Bundler>;
  using PinholeCameraCal3DS2 = gtsam::PinholeCamera<gtsam::Cal3DS2>;
  using PinholeCameraCal3Unified = gtsam::PinholeCamera<gtsam::Cal3Unified>;
  using PinholeCameraCal3Fisheye = gtsam::PinholeCamera<gtsam::Cal3Fisheye>;
  
}  // namespace gtsam
