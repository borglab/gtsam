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
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace gtsam {
namespace eqvio {

using SOT3 = ProductLieGroup<SO3, double>;

using VIOSE23 = ExtendedPose3<2>;
using VIOBias = imuBias::ConstantBias;
using VIOLandmarkGroup = PowerLieGroup<SOT3, Eigen::Dynamic>;
using VIOSensorCore = ProductLieGroup<VIOSE23, VIOBias>;
using VIOLandmarkCore = ProductLieGroup<Pose3, VIOLandmarkGroup>;
using VIOGroup = ProductLieGroup<VIOSensorCore, VIOLandmarkCore>;

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
  IMUInput operator-(const VIOBias& bias) const {
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

/// EqVIO camera model.
using VIOCameraModel = PinholeCamera<Cal3_S2>;

/// Convert image coordinates to an undistorted 3D bearing-like vector.
inline Vector3 undistortPoint(const VIOCameraModel& camera, const Point2& y) {
  const Point2 p = camera.calibration().calibrate(y);
  return Vector3(p.x(), p.y(), 1.0);
}

/// Projection Jacobian with respect to the input 3D vector.
inline Matrix23 projectionJacobian(const VIOCameraModel& camera, const Vector3& y) {
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

/// Readable accessors for the composed ProductLieGroup VIOGroup.
inline const VIOSE23& A_sensorKinematics(const VIOGroup& X) {
  return X.first.first;
}

inline const VIOBias& Beta_biasOffset(const VIOGroup& X) {
  return X.first.second;
}

inline const Pose3& B_cameraExtrinsics(const VIOGroup& X) {
  return X.second.first;
}

inline const VIOLandmarkGroup& Q_landmarkTransforms(const VIOGroup& X) {
  return X.second.second;
}

inline size_t N_landmarkCount(const VIOGroup& X) {
  return Q_landmarkTransforms(X).size();
}
inline size_t Dim_groupTangent(const VIOGroup& X) {
  return 21 + 4 * N_landmarkCount(X);
}

inline VIOGroup makeVIOGroup(const VIOSE23& sensor_kinematics,
                             const VIOBias& bias_offset,
                             const Pose3& camera_extrinsics,
                             const VIOLandmarkGroup& landmark_transforms) {
  return VIOGroup(VIOSensorCore(sensor_kinematics, bias_offset),
                  VIOLandmarkCore(camera_extrinsics, landmark_transforms));
}

inline VIOGroup makeVIOGroupIdentity(size_t n = 0) {
  return makeVIOGroup(VIOSE23::Identity(), VIOBias::Identity(), Pose3::Identity(),
                      VIOLandmarkGroup(n));
}

}  // namespace eqvio

}  // namespace gtsam

namespace gtsam {

template <size_t I>
inline decltype(auto) get(eqvio::VIOGroup& X) {
  static_assert(I < 4, "VIOGroup index out of range");
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
inline decltype(auto) get(const eqvio::VIOGroup& X) {
  static_assert(I < 4, "VIOGroup index out of range");
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
struct tuple_size<gtsam::eqvio::VIOGroup> : std::integral_constant<size_t, 4> {};

template <>
struct tuple_element<0, gtsam::eqvio::VIOGroup> {
  using type = gtsam::eqvio::VIOSE23;
};

template <>
struct tuple_element<1, gtsam::eqvio::VIOGroup> {
  using type = gtsam::eqvio::VIOBias;
};

template <>
struct tuple_element<2, gtsam::eqvio::VIOGroup> {
  using type = gtsam::Pose3;
};

template <>
struct tuple_element<3, gtsam::eqvio::VIOGroup> {
  using type = gtsam::eqvio::VIOLandmarkGroup;
};

}  // namespace std
