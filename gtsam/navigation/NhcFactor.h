/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   NhcFactor.h
 *  @author inuex35
 *  @brief  Non-holonomic constraint (NHC) factors for cars and other
 *          non-holonomic robots
 *  @date   July 4, 2026
 **/
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <string>

namespace gtsam {

/**
 * Non-holonomic constraint (NHC) factor for a car or other non-holonomic robot.
 *
 * A wheeled vehicle that does not slip has no lateral or vertical velocity at
 * its reference point (typically the center of the rear axle). This factor
 * expresses the velocity of that point in the vehicle frame and constrains its
 * lateral and vertical components to zero:
 *
 *   v_point   = wRb^T * v_world + omega x leverArm     (body frame)
 *   v_vehicle = bRv^T * v_point                         (vehicle frame)
 *   error     = [v_vehicle.y, v_vehicle.z]              (2-vector)
 *
 * Frames and inputs:
 *   - The pose is the body (e.g. IMU) frame in the world, wTb; only its
 *     rotation wRb enters the error.
 *   - The velocity is the world-frame velocity of the body-frame origin.
 *   - omega is the bias-corrected body-frame angular velocity.
 *   - leverArm is the vector from the body-frame origin to the vehicle
 *     reference point, expressed in the body frame.
 *   - bRv is the mounting rotation: the orientation of the vehicle frame
 *     (x forward, y lateral, z vertical) relative to the body frame.
 *
 * The forward speed is not constrained; combine this factor with a
 * wheel-odometry or other velocity measurement to observe it. The 2-D noise
 * model is expressed in the vehicle frame (lateral, vertical).
 *
 * Here bRv is a fixed constant; the default identity means the body and vehicle
 * frames are aligned. Use NhcFactorCalib to estimate bRv online.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT NhcFactor : public NoiseModelFactorN<Pose3, Vector3> {
 private:
  typedef NoiseModelFactorN<Pose3, Vector3> Base;

  Vector3 angularVelocity_;  ///< body-frame angular velocity [rad/s]
  Vector3 leverArm_;         ///< body origin -> vehicle reference point [m]
  Rot3 mountRotation_;       ///< fixed vehicle-to-body rotation bRv

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<NhcFactor> shared_ptr;
  typedef NhcFactor This;

  /// Default constructor, for serialization only.
  NhcFactor()
      : angularVelocity_(Vector3::Zero()), leverArm_(Vector3::Zero()) {}

  ~NhcFactor() override = default;

  /**
   * Construct an NhcFactor with a fixed mounting rotation.
   *
   * @param poseKey         key of the body pose wTb.
   * @param velocityKey     key of the world-frame velocity.
   * @param angularVelocity bias-corrected body-frame angular velocity [rad/s].
   * @param leverArm        body origin to vehicle reference point [m], body
   *                        frame.
   * @param model           2-D noise model (lateral, vertical) in the vehicle
   *                        frame.
   * @param mountRotation   fixed vehicle-to-body rotation bRv.
   */
  NhcFactor(Key poseKey, Key velocityKey, const Vector3& angularVelocity,
            const Vector3& leverArm, const SharedNoiseModel& model,
            const Rot3& mountRotation = Rot3());

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Print the factor.
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// Check equality up to a tolerance.
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  /// Return the lateral and vertical velocity of the vehicle reference point.
  Vector evaluateError(const Pose3& pose, const Vector3& velocity,
                       OptionalMatrixType H_pose,
                       OptionalMatrixType H_velocity) const override;

  /// Body-frame angular velocity [rad/s].
  const Vector3& angularVelocity() const { return angularVelocity_; }
  /// Body origin to vehicle reference point [m].
  const Vector3& leverArm() const { return leverArm_; }
  /// Fixed vehicle-to-body rotation bRv.
  const Rot3& mountRotation() const { return mountRotation_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(angularVelocity_);
    ar& BOOST_SERIALIZATION_NVP(leverArm_);
    ar& BOOST_SERIALIZATION_NVP(mountRotation_);
  }
#endif
};

/// traits
template <>
struct traits<NhcFactor> : public Testable<NhcFactor> {};

/**
 * Non-holonomic constraint factor that also estimates the mounting rotation.
 *
 * Uses the same error model as NhcFactor, but the vehicle-to-body rotation bRv
 * is an estimated Rot3 variable. It is typically shared by all factors along a
 * trajectory and anchored by a loose prior, which supports online calibration
 * of an arbitrarily mounted sensor: a misaligned mounting makes forward motion
 * appear as lateral or vertical velocity.
 *
 * Only the pitch and yaw of bRv are observable, scaled by the forward speed.
 * Rotation about the vehicle's forward axis (roll) is never observable from
 * the NHC alone: whenever the constraint holds, the reference-point velocity
 * points along that axis and is unchanged by the rotation. Constrain the roll
 * with a tight prior or another sensor. Mounting pitch and yaw are also
 * correlated with errors in the body attitude, so the attitude must be
 * observable from other measurements.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT NhcFactorCalib
    : public NoiseModelFactorN<Pose3, Vector3, Rot3> {
 private:
  typedef NoiseModelFactorN<Pose3, Vector3, Rot3> Base;

  Vector3 angularVelocity_;  ///< body-frame angular velocity [rad/s]
  Vector3 leverArm_;         ///< body origin -> vehicle reference point [m]

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<NhcFactorCalib> shared_ptr;
  typedef NhcFactorCalib This;

  /// Default constructor, for serialization only.
  NhcFactorCalib()
      : angularVelocity_(Vector3::Zero()), leverArm_(Vector3::Zero()) {}

  ~NhcFactorCalib() override = default;

  /**
   * Construct an NhcFactorCalib with an estimated mounting rotation.
   *
   * @param poseKey          key of the body pose wTb.
   * @param velocityKey      key of the world-frame velocity.
   * @param mountRotationKey key of the vehicle-to-body rotation bRv.
   * @param angularVelocity  bias-corrected body-frame angular velocity [rad/s].
   * @param leverArm         body origin to vehicle reference point [m], body
   *                         frame.
   * @param model            2-D noise model (lateral, vertical) in the vehicle
   *                         frame.
   */
  NhcFactorCalib(Key poseKey, Key velocityKey, Key mountRotationKey,
                 const Vector3& angularVelocity, const Vector3& leverArm,
                 const SharedNoiseModel& model);

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Print the factor.
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// Check equality up to a tolerance.
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  /// Return the lateral and vertical velocity of the vehicle reference point.
  Vector evaluateError(const Pose3& pose, const Vector3& velocity,
                       const Rot3& mountRotation, OptionalMatrixType H_pose,
                       OptionalMatrixType H_velocity,
                       OptionalMatrixType H_mountRotation) const override;

  /// Body-frame angular velocity [rad/s].
  const Vector3& angularVelocity() const { return angularVelocity_; }
  /// Body origin to vehicle reference point [m].
  const Vector3& leverArm() const { return leverArm_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(angularVelocity_);
    ar& BOOST_SERIALIZATION_NVP(leverArm_);
  }
#endif
};

/// traits
template <>
struct traits<NhcFactorCalib> : public Testable<NhcFactorCalib> {};

}  // namespace gtsam
