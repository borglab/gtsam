/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DeviceGeometryTypes.h
 * @brief   Trivially copyable device forms of the supported manifold types
 * @author  Ruogu Li
 * @date    Jun 17, 2026
 */

#pragma once

#include <cstdint>

namespace gtsam::cuda {

/// Type tag identifying a PinholeCamera<Cal3Bundler> block in DeviceValues.
inline constexpr uint32_t kDevicePinholeCameraCal3BundlerType = 0x50434243u;
/// Type tag identifying a Point3 block in DeviceValues.
inline constexpr uint32_t kDevicePoint3Type = 0x50544e33u;
/// Tangent width of PinholeCamera<Cal3Bundler>: 6 pose plus f, k1, k2.
inline constexpr int kDevicePinholeCameraCal3BundlerTangentDim = 9;
/// Tangent width of Point3.
inline constexpr int kDevicePoint3TangentDim = 3;

/**
 * Device form of gtsam::PinholeCamera<gtsam::Cal3Bundler>.
 *
 * A flat, trivially copyable struct because the host type is not usable in a
 * kernel: Rot3 may hold a quaternion or an Eigen matrix depending on build
 * configuration, and Cal3Bundler carries virtual machinery. Storing the rotation
 * as an explicit 3x3 matrix also means a kernel never has to normalize or
 * convert anything. Retract and project with the free functions in
 * DeviceGeometryKernels.h.
 */
struct DevicePinholeCameraCal3Bundler {
  /// Pose rotation, row-major, matching Rot3::matrix().
  double R[9];
  /// Pose translation.
  double t[3];
  /// Focal length.
  double f;
  /// First radial distortion coefficient.
  double k1;
  /// Second radial distortion coefficient.
  double k2;
};

/// Device form of gtsam::Point3, which is an Eigen type the kernels cannot use.
struct DevicePoint3 {
  double x;
  double y;
  double z;
};

}  // namespace gtsam::cuda
