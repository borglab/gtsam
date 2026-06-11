/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EqVIOCommon.h
 * @brief Common EqVIO math/data types for unstable navigation.
 * @author Rohan Bansal
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam_unstable/dllexport.h>

#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace gtsam {
namespace eqvio {

/// Similarity transform group element represented as `(SO3, log_scale)`.
using SOT3 = ProductLieGroup<SO3, double>;

using Se23 = ExtendedPose3<2>;
using Bias = imuBias::ConstantBias;
/// See eq. (22) in arXiv:2205.01980v3 for VI-SLAM Group defn.
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
  double stamp = -1.0;
  Vector3 gyr = Z_3x1;
  Vector3 acc = Z_3x1;
  Vector3 gyrBiasVel = Z_3x1;
  Vector3 accBiasVel = Z_3x1;

  IMUInput() = default;

  /// Subtract a ConstantBias from [gyr, acc].
  IMUInput operator-(const Bias& bias) const {
    IMUInput out(*this);
    out.gyr -= bias.gyroscope();
    out.acc -= bias.accelerometer();
    return out;
  }
};

class State;

/// Discrete lift map used by EqVIO propagation helpers.
GTSAM_UNSTABLE_EXPORT VioGroup liftVelocityDiscrete(const State& state,
                                                    const IMUInput& velocity,
                                                    double dt);

/**
 * @brief Camera model used by EqVIO measurement functions.
 *
 * We reuse GTSAM's existing `PinholeCamera<Cal3_S2>` instead of defining a new
 * camera class here, and try to keep helper functions minimal.
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
inline Matrix23 projectionJacobian(const CameraModel& camera,
                                   const Vector3& y) {
  if (std::abs(y.z()) < 1e-12) {
    throw std::invalid_argument("projectionJacobian: z is near zero");
  }
  Matrix23 Dpn_dy;
  Matrix22 Dpi_dpn;
  const Point2 pn = PinholeBase::Project(Point3(y), Dpn_dy);
  camera.calibration().uncalibrate(pn, {}, Dpi_dpn);
  return Dpi_dpn * Dpn_dy;
}

/// Vision measurement keyed by landmark id.
using VisionMeasurement = std::map<Key, Point2>;

/// Ordered landmark ids matching map iteration order.
inline KeyVector measurementIds(const VisionMeasurement& measurement) {
  KeyVector ids;
  ids.reserve(measurement.size());
  for (const auto& item : measurement) {
    ids.push_back(item.first);
  }
  return ids;
}

/// Stack a landmark-id-ordered measurement map into a dense vector.
inline Vector measurementVector(const VisionMeasurement& measurement) {
  Vector v = Vector::Zero(static_cast<int>(2 * measurement.size()));
  int i = 0;
  for (const auto& item : measurement) {
    v.segment<2>(2 * i) = item.second;
    ++i;
  }
  return v;
}

/// Discrete propagation lift functor for the explicit EqVIO predict path.
struct Lift {
  Lift(const IMUInput& imu, double dt) : imu_(imu), dt_(dt) {}

  Vector operator()(
      const State& xi,
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = {}) const {
    const Vector y0 =
        (VioGroup::Logmap(liftVelocityDiscrete(xi, imu_, dt_)) / dt_).eval();
    if (H) *H = Matrix();
    return y0;
  }

 private:
  IMUInput imu_;
  double dt_;
};

/// Readable accessors for the composed ProductLieGroup VioGroup.
inline const Se23& A_sensorKinematics(const VioGroup& X) {
  return X.first.first;
}

inline const Bias& Beta_biasOffset(const VioGroup& X) { return X.first.second; }

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

/// Decompose VioGroup into (A, Beta, B, Q) references for structured bindings.
inline auto decompose(const VioGroup& X) {
  return std::tie(A_sensorKinematics(X), Beta_biasOffset(X),
                  B_cameraExtrinsics(X), Q_landmarkTransforms(X));
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
