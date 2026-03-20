/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOCommon.h
/// @brief Common EqVIO math/data types for unstable navigation.
/// @author Rohan Bansal

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam_unstable/dllexport.h>

#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/navigation/ImuBias.h>

#include <map>
#include <memory>
#include <string>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace gtsam {
namespace eqvio {

using SOT3 = ProductLieGroup<SO3, double>;

using Se23 = ExtendedPose3<2>;
using Bias = imuBias::ConstantBias;
using LandmarkGroup = PowerLieGroup<SOT3, Eigen::Dynamic>;
using SensorCore = ProductLieGroup<Se23, Bias>;
using LandmarkCore = ProductLieGroup<Pose3, LandmarkGroup>;
using VioGroup = ProductLieGroup<SensorCore, LandmarkCore>;

/// Approximate gravitational acceleration magnitude in m/s^2.
constexpr double GRAVITY_CONSTANT = 9.80665;

/// Return positive scale component of SOT3.
inline double SOT3Scale(const SOT3& Q) { return std::exp(Q.second); }

/// Return SO3 rotation component of SOT3.
inline const SO3& SOT3Rotation(const SOT3& Q) { return Q.first; }

/// Return scaled-rotation matrix a*R for SOT3 element (R,log(a)).
inline Matrix3 SOT3ScaledRotation(const SOT3& Q) {
  return SOT3Rotation(Q).matrix() * SOT3Scale(Q);
}

/// Apply inverse SOT3 transform to a 3D point.
inline Vector3 SOT3ApplyInverse(const SOT3& Q, const Vector3& p) {
  return (1.0 / SOT3Scale(Q)) * (SOT3Rotation(Q).matrix().transpose() * p);
}

/// Construct SOT3 from rotation and positive scale.
inline SOT3 MakeSOT3(const SO3& R, double scale) {
  if (scale <= 0.0) {
    throw std::invalid_argument("MakeSOT3: scale must be strictly positive");
  }
  return SOT3(R, std::log(scale));
}

/// IMU input bundle used by EqVIO propagation.
struct GTSAM_UNSTABLE_EXPORT IMUInput {
  static constexpr int CompDim = 12;
  using Vector12 = Eigen::Matrix<double, 12, 1>;

  double stamp = -1.0;
  Vector3 gyr = Vector3::Zero();
  Vector3 acc = Vector3::Zero();
  Vector3 gyrBiasVel = Vector3::Zero();
  Vector3 accBiasVel = Vector3::Zero();

  /// Return a zero-initialized input with invalid timestamp.
  static IMUInput Zero() { return IMUInput(); }

  IMUInput() = default;

  /// Construct from stacked [gyr, acc, gyrBiasVel, accBiasVel].
  explicit IMUInput(const Vector12& vec) {
    gyr = vec.segment<3>(0);
    acc = vec.segment<3>(3);
    gyrBiasVel = vec.segment<3>(6);
    accBiasVel = vec.segment<3>(9);
  }

  /// Component-wise addition.
  IMUInput operator+(const IMUInput& other) const {
    IMUInput out;
    out.stamp = stamp >= 0.0 ? stamp : other.stamp;
    out.gyr = gyr + other.gyr;
    out.acc = acc + other.acc;
    out.gyrBiasVel = gyrBiasVel + other.gyrBiasVel;
    out.accBiasVel = accBiasVel + other.accBiasVel;
    return out;
  }

  /// Subtract a ConstantBias from [gyr, acc].
  IMUInput operator-(const Bias& bias) const {
    IMUInput out(*this);
    out.gyr -= bias.gyroscope();
    out.acc -= bias.accelerometer();
    return out;
  }

  /// Scale all components.
  IMUInput operator*(double c) const {
    IMUInput out(*this);
    out.gyr *= c;
    out.acc *= c;
    out.gyrBiasVel *= c;
    out.accBiasVel *= c;
    return out;
  }
};

/**
 * @brief Camera model used by EqVIO measurement functions.
 *
 * We reuse GTSAM's existing `PinholeCamera<Cal3_S2>` instead of defining a new
 * camera class here.
 *
 * More general camera models (distortion, fisheye, etc.) are intentionally not
 * used here because EqVIO also needs:
 * 1. an explicit "undistort to normalized bearing" operation, and
 * 2. a closed-form projection Jacobian with respect to the 3D ray.
 * Those operations are model-specific for non-pinhole cameras and would add
 * extra branches and conversion code.
 */
using CameraModel = PinholeCamera<Cal3_S2>;

/**
 * @brief Convert image coordinates into a normalized bearing-like vector.
 *
 * For `Cal3_S2`, calibration inversion maps pixel coordinates to normalized
 * image coordinates `(x/z, y/z)`. EqVIO represents this as a 3D direction-like
 * vector `[x/z, y/z, 1]`.
 */
inline Vector3 undistortPoint(const CameraModel& camera, const Point2& y) {
  const Point2 p = camera.calibration().calibrate(y);
  return Vector3(p.x(), p.y(), 1.0);
}

/**
 * @brief Jacobian of pixel projection with respect to a camera-frame 3D point.
 *
 * This returns `d pi(y) / d y` for the `Cal3_S2` pinhole projection used by
 * EqVIO's linearized output model.
 *
 * @throws std::invalid_argument if `y.z()` is numerically near zero.
 */
inline Matrix23 projectionJacobian(const CameraModel& camera, const Vector3& y) {
  if (std::abs(y.z()) < 1e-12) {
    throw std::invalid_argument("projectionJacobian: z is near zero");
  }

  const double invz = 1.0 / y.z();
  const double invz2 = invz * invz;
  const double fx = camera.calibration().fx();
  const double fy = camera.calibration().fy();
  const double s = camera.calibration().skew();

  Matrix23 J;
  J << fx * invz, s * invz, -(fx * y.x() + s * y.y()) * invz2, 0.0, fy * invz,
      -fy * y.y() * invz2;
  return J;
}

/// Vision measurement keyed by landmark id.
using VisionMeasurement = std::map<int, Point2>;

/// Ordered landmark ids matching map iteration order.
inline std::vector<int> measurementIds(const VisionMeasurement& measurement) {
  std::vector<int> ids;
  ids.reserve(measurement.size());
  for (const auto& item : measurement) {
    ids.push_back(item.first);
  }
  return ids;
}

/// Remove a contiguous row block from a matrix.
inline void removeRows(Matrix& mat, int startRow, int numRows) {
  const int rows = mat.rows();
  const int cols = mat.cols();
  assert(startRow >= 0 && numRows >= 0 && startRow + numRows <= rows);
  mat.block(startRow, 0, rows - numRows - startRow, cols) =
      mat.block(startRow + numRows, 0, rows - numRows - startRow, cols);
  mat.conservativeResize(rows - numRows, Eigen::NoChange);
}

/// Remove a contiguous column block from a matrix.
inline void removeCols(Matrix& mat, int startCol, int numCols) {
  const int rows = mat.rows();
  const int cols = mat.cols();
  assert(startCol >= 0 && numCols >= 0 && startCol + numCols <= cols);
  mat.block(0, startCol, rows, cols - numCols - startCol) =
      mat.block(0, startCol + numCols, rows, cols - numCols - startCol);
  mat.conservativeResize(Eigen::NoChange, cols - numCols);
}

/// Readable accessors for the composed ProductLieGroup VioGroup.
inline const Se23& A_sensorKinematics(const VioGroup& X) {
  return X.first.first;
}

inline const Bias& Beta_biasOffset(const VioGroup& X) {
  return X.first.second;
}

inline const Pose3& B_cameraExtrinsics(const VioGroup& X) {
  return X.second.first;
}

inline const LandmarkGroup& Q_landmarkTransforms(const VioGroup& X) {
  return X.second.second;
}

inline size_t N_landmarkCount(const VioGroup& X) {
  return Q_landmarkTransforms(X).size();
}
inline size_t Dim_groupTangent(const VioGroup& X) {
  return 21 + 4 * N_landmarkCount(X);
}

inline VioGroup makeVioGroup(const Se23& sensor_kinematics,
                             const Bias& bias_offset,
                             const Pose3& camera_extrinsics,
                             const LandmarkGroup& landmark_transforms) {
  return VioGroup(SensorCore(sensor_kinematics, bias_offset),
                  LandmarkCore(camera_extrinsics, landmark_transforms));
}

inline VioGroup makeVioGroupIdentity(size_t n = 0) {
  return makeVioGroup(Se23::Identity(), Bias::Identity(), Pose3::Identity(),
                      LandmarkGroup(n));
}

}  // namespace eqvio

}  // namespace gtsam

namespace gtsam {

template <size_t I>
inline decltype(auto) get(eqvio::VioGroup& X) {
  static_assert(I < 4, "VioGroup index out of range");
  if constexpr (I == 0) {
    return (X.first.first);
  } else if constexpr (I == 1) {
    return (X.first.second);
  } else if constexpr (I == 2) {
    return (X.second.first);
  } else {
    return (X.second.second);
  }
}

template <size_t I>
inline decltype(auto) get(const eqvio::VioGroup& X) {
  static_assert(I < 4, "VioGroup index out of range");
  if constexpr (I == 0) {
    return (X.first.first);
  } else if constexpr (I == 1) {
    return (X.first.second);
  } else if constexpr (I == 2) {
    return (X.second.first);
  } else {
    return (X.second.second);
  }
}

}  // namespace gtsam

namespace std {

template <>
struct tuple_size<gtsam::eqvio::VioGroup> : std::integral_constant<size_t, 4> {};

template <>
struct tuple_element<0, gtsam::eqvio::VioGroup> {
  using type = gtsam::eqvio::Se23;
};

template <>
struct tuple_element<1, gtsam::eqvio::VioGroup> {
  using type = gtsam::eqvio::Bias;
};

template <>
struct tuple_element<2, gtsam::eqvio::VioGroup> {
  using type = gtsam::Pose3;
};

template <>
struct tuple_element<3, gtsam::eqvio::VioGroup> {
  using type = gtsam::eqvio::LandmarkGroup;
};

}  // namespace std
