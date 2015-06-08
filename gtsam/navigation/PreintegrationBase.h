/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegrationBase.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

namespace gtsam {

/**
 * Struct to hold all state variables of returned by Predict function
 */
struct PoseVelocityBias {
  Pose3 pose;
  Vector3 velocity;
  imuBias::ConstantBias bias;

  PoseVelocityBias(const Pose3& _pose, const Vector3& _velocity, const imuBias::ConstantBias _bias)
      : pose(_pose),
        velocity(_velocity),
        bias(_bias) {
  }
};

/**
 * PreintegrationBase is the base class for PreintegratedMeasurements
 * (in ImuFactor) and CombinedPreintegratedMeasurements (in CombinedImuFactor).
 * It includes the definitions of the preintegrated variables and the methods
 * to access, print, and compare them.
 */
class PreintegrationBase : public PreintegratedRotation {

  imuBias::ConstantBias biasHat_;  ///< Acceleration and angular rate bias values used during preintegration
  bool use2ndOrderIntegration_;  ///< Controls the order of integration

  Vector3 deltaPij_;  ///< Preintegrated relative position (does not take into account velocity at time i, see deltap+, in [2]) (in frame i)
  Vector3 deltaVij_;  ///< Preintegrated relative velocity (in global frame)

  Matrix3 delPdelBiasAcc_;  ///< Jacobian of preintegrated position w.r.t. acceleration bias
  Matrix3 delPdelBiasOmega_;  ///< Jacobian of preintegrated position w.r.t. angular rate bias
  Matrix3 delVdelBiasAcc_;  ///< Jacobian of preintegrated velocity w.r.t. acceleration bias
  Matrix3 delVdelBiasOmega_;  ///< Jacobian of preintegrated velocity w.r.t. angular rate bias

  const Matrix3 accelerometerCovariance_;  ///< continuous-time "Covariance" of accelerometer measurements
  const Matrix3 integrationCovariance_;  ///< continuous-time "Covariance" describing integration uncertainty
  /// (to compensate errors in Euler integration)

 public:

  /**
   *  Default constructor, initializes the variables in the base class
   *  @param bias Current estimate of acceleration and rotation rate biases
   *  @param use2ndOrderIntegration     Controls the order of integration
   *  (if false: p(t+1) = p(t) + v(t) deltaT ; if true: p(t+1) = p(t) + v(t) deltaT + 0.5 * acc(t) deltaT^2)
   */
  PreintegrationBase(const imuBias::ConstantBias& bias, const Matrix3& measuredAccCovariance,
                     const Matrix3& measuredOmegaCovariance,
                     const Matrix3&integrationErrorCovariance, const bool use2ndOrderIntegration);

  /// methods to access class variables
  bool use2ndOrderIntegration() const {
    return use2ndOrderIntegration_;
  }
  const Vector3& deltaPij() const {
    return deltaPij_;
  }
  const Vector3& deltaVij() const {
    return deltaVij_;
  }
  const imuBias::ConstantBias& biasHat() const {
    return biasHat_;
  }
  Vector6 biasHatVector() const {
    return biasHat_.vector();
  }  // expensive
  const Matrix3& delPdelBiasAcc() const {
    return delPdelBiasAcc_;
  }
  const Matrix3& delPdelBiasOmega() const {
    return delPdelBiasOmega_;
  }
  const Matrix3& delVdelBiasAcc() const {
    return delVdelBiasAcc_;
  }
  const Matrix3& delVdelBiasOmega() const {
    return delVdelBiasOmega_;
  }

  const Matrix3& accelerometerCovariance() const {
    return accelerometerCovariance_;
  }
  const Matrix3& integrationCovariance() const {
    return integrationCovariance_;
  }

  /// print
  void print(const std::string& s) const;

  /// check equality
  bool equals(const PreintegrationBase& other, double tol) const;

  /// Re-initialize PreintegratedMeasurements
  void resetIntegration();

  /// Update preintegrated measurements
  void updatePreintegratedMeasurements(const Vector3& correctedAcc, const Rot3& incrR,
                                       const double deltaT, OptionalJacobian<9, 9> F);

  /// Update Jacobians to be used during preintegration
  void updatePreintegratedJacobians(const Vector3& correctedAcc,
                                    const Matrix3& D_Rincr_integratedOmega, const Rot3& incrR,
                                    double deltaT);

  void correctMeasurementsByBiasAndSensorPose(const Vector3& measuredAcc,
                                              const Vector3& measuredOmega, Vector3& correctedAcc,
                                              Vector3& correctedOmega,
                                              boost::optional<const Pose3&> body_P_sensor);

  /// Predict state at time j
  PoseVelocityBias predict(
      const Pose3& pose_i, const Vector3& vel_i, const imuBias::ConstantBias& bias_i,
      const Vector3& gravity, const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis = false,
      boost::optional<Vector3&> deltaPij_biascorrected_out = boost::none,
      boost::optional<Vector3&> deltaVij_biascorrected_out = boost::none) const;

  /// Compute errors w.r.t. preintegrated measurements and jacobians wrt pose_i, vel_i, bias_i, pose_j, bias_j
  Vector9 computeErrorAndJacobians(const Pose3& pose_i, const Vector3& vel_i, const Pose3& pose_j,
                                   const Vector3& vel_j, const imuBias::ConstantBias& bias_i,
                                   const Vector3& gravity, const Vector3& omegaCoriolis,
                                   const bool use2ndOrderCoriolis, OptionalJacobian<9, 6> H1 =
                                       boost::none,
                                   OptionalJacobian<9, 3> H2 = boost::none,
                                   OptionalJacobian<9, 6> H3 = boost::none,
                                   OptionalJacobian<9, 3> H4 = boost::none,
                                   OptionalJacobian<9, 6> H5 = boost::none) const;

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotation);
    ar & BOOST_SERIALIZATION_NVP(biasHat_);
    ar & BOOST_SERIALIZATION_NVP(deltaPij_);
    ar & BOOST_SERIALIZATION_NVP(deltaVij_);
    ar & BOOST_SERIALIZATION_NVP(delPdelBiasAcc_);
    ar & BOOST_SERIALIZATION_NVP(delPdelBiasOmega_);
    ar & BOOST_SERIALIZATION_NVP(delVdelBiasAcc_);
    ar & BOOST_SERIALIZATION_NVP(delVdelBiasOmega_);
  }
};

class ImuBase {

 protected:

  Vector3 gravity_;
  Vector3 omegaCoriolis_;
  boost::optional<Pose3> body_P_sensor_;  ///< The pose of the sensor in the body frame
  bool use2ndOrderCoriolis_;  ///< Controls whether higher order terms are included when calculating the Coriolis Effect

 public:

  /// Default constructor, with decent gravity and zero coriolis
  ImuBase();

  /// Fully fledge constructor
  ImuBase(const Vector3& gravity, const Vector3& omegaCoriolis,
          boost::optional<const Pose3&> body_P_sensor = boost::none,
          const bool use2ndOrderCoriolis = false);

  const Vector3& gravity() const {
    return gravity_;
  }
  const Vector3& omegaCoriolis() const {
    return omegaCoriolis_;
  }

};

}  /// namespace gtsam
