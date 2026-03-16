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

  /// Construct from stacked [gyr, acc].
  explicit IMUInput(const Vector6& vec) {
    gyr = vec.head<3>();
    acc = vec.tail<3>();
  }

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

  /// Subtract stacked [gyr, acc].
  IMUInput operator-(const Vector6& vec) const {
    IMUInput out(*this);
    out.gyr -= vec.head<3>();
    out.acc -= vec.tail<3>();
    return out;
  }

  /// Subtract stacked [gyr, acc, gyrBiasVel, accBiasVel].
  IMUInput operator-(const Vector12& vec) const {
    IMUInput out(*this);
    out.gyr -= vec.segment<3>(0);
    out.acc -= vec.segment<3>(3);
    out.gyrBiasVel -= vec.segment<3>(6);
    out.accBiasVel -= vec.segment<3>(9);
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

/// EqVIO camera built on top of GTSAM PinholeCamera<Cal3_S2>.
class GTSAM_UNSTABLE_EXPORT VIOCameraModel
    : public PinholeCamera<Cal3_S2> {
 public:
  using Base = PinholeCamera<Cal3_S2>;

  VIOCameraModel() : Base(Pose3::Identity(), Cal3_S2()) {}
  explicit VIOCameraModel(const Cal3_S2& K) : Base(Pose3::Identity(), K) {}
  VIOCameraModel(const Pose3& pose, const Cal3_S2& K) : Base(pose, K) {}
  virtual ~VIOCameraModel() = default;

  /// Project a camera-frame 3D point to image coordinates.
  virtual Point2 projectPoint(const Point3& p) const {
    if (std::abs(p.z()) < 1e-12) {
      throw std::invalid_argument("VIOCameraModel::projectPoint: z is near zero");
    }
    const Point2 pn(p.x() / p.z(), p.y() / p.z());
    return this->calibration().uncalibrate(pn);
  }

  /// Convert image coordinates to an undistorted 3D bearing-like vector.
  virtual Vector3 undistortPoint(const Point2& y) const {
    const Point2 p = this->calibration().calibrate(y);
    return Vector3(p.x(), p.y(), 1.0);
  }

  /// Projection Jacobian with respect to the input 3D vector.
  virtual Matrix23 projectionJacobian(const Vector3& y) const {
    if (std::abs(y.z()) < 1e-12) {
      throw std::invalid_argument("VIOCameraModel::projectionJacobian: z is near zero");
    }

    const double invz = 1.0 / y.z();
    const double invz2 = invz * invz;
    const double fx = this->calibration().fx();
    const double fy = this->calibration().fy();
    const double s = this->calibration().skew();

    Matrix23 J;
    J << fx * invz, s * invz, -(fx * y.x() + s * y.y()) * invz2, 0.0,
        fy * invz, -fy * y.y() * invz2;
    return J;
  }
};

/// Vision measurement keyed by landmark id.
using VisionMeasurement = std::map<int, Point2>;

/// Ordered landmark ids matching map iteration order.
inline std::vector<int> measurementIds(const VisionMeasurement& measurement) {
  std::vector<int> ids;
  ids.reserve(measurement.size());
  for (const auto& [id, _] : measurement) {
    (void)_;
    ids.push_back(id);
  }
  return ids;
}

/// Flatten measurements to [u0,v0,u1,v1,...] in map iteration order.
inline Vector measurementVector(const VisionMeasurement& measurement) {
  Vector v = Vector::Zero(static_cast<int>(2 * measurement.size()));
  int i = 0;
  for (const auto& [_, y] : measurement) {
    (void)_;
    v.segment<2>(2 * i) = y;
    ++i;
  }
  return v;
}

/// Compute lhs-rhs in flattened measurement coordinates.
inline Vector measurementDifference(const VisionMeasurement& lhs,
                                    const VisionMeasurement& rhs) {
  if (lhs.size() != rhs.size()) {
    throw std::invalid_argument("measurementDifference: size mismatch");
  }

  Vector diff = Vector::Zero(static_cast<int>(2 * lhs.size()));
  auto itL = lhs.begin();
  auto itR = rhs.begin();
  int i = 0;
  for (; itL != lhs.end(); ++itL, ++itR) {
    if (itL->first != itR->first) {
      throw std::invalid_argument("measurementDifference: id mismatch");
    }
    diff.segment<2>(2 * i) = itL->second - itR->second;
    ++i;
  }
  return diff;
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
