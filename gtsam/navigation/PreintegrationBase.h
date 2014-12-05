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

namespace gtsam {

/**
 * PreintegrationBase is the base class for PreintegratedMeasurements
 * (in ImuFactor) and CombinedPreintegratedMeasurements (in CombinedImuFactor).
 * It includes the definitions of the preintegrated variables and the methods
 * to access, print, and compare them.
 */
class PreintegrationBase : public PreintegratedRotation {

  imuBias::ConstantBias biasHat_; ///< Acceleration and angular rate bias values used during preintegration
  bool use2ndOrderIntegration_; ///< Controls the order of integration

  Vector3 deltaPij_; ///< Preintegrated relative position (does not take into account velocity at time i, see deltap+, in [2]) (in frame i)
  Vector3 deltaVij_; ///< Preintegrated relative velocity (in global frame)

  Matrix3 delPdelBiasAcc_;   ///< Jacobian of preintegrated position w.r.t. acceleration bias
  Matrix3 delPdelBiasOmega_; ///< Jacobian of preintegrated position w.r.t. angular rate bias
  Matrix3 delVdelBiasAcc_;   ///< Jacobian of preintegrated velocity w.r.t. acceleration bias
  Matrix3 delVdelBiasOmega_; ///< Jacobian of preintegrated velocity w.r.t. angular rate bias

public:

  /**
   *  Default constructor, initializes the variables in the base class
   *  @param bias Current estimate of acceleration and rotation rate biases
   *  @param use2ndOrderIntegration     Controls the order of integration
   *  (if false: p(t+1) = p(t) + v(t) deltaT ; if true: p(t+1) = p(t) + v(t) deltaT + 0.5 * acc(t) deltaT^2)
   */
  PreintegrationBase(const imuBias::ConstantBias& bias, const bool use2ndOrderIntegration) :
    biasHat_(bias), use2ndOrderIntegration_(use2ndOrderIntegration),
    deltaPij_(Vector3::Zero()), deltaVij_(Vector3::Zero()),
    delPdelBiasAcc_(Z_3x3), delPdelBiasOmega_(Z_3x3),
    delVdelBiasAcc_(Z_3x3), delVdelBiasOmega_(Z_3x3) {}

  /// methods to access class variables
  const Vector3& deltaPij() const {return deltaPij_;}
  const Vector3& deltaVij() const {return deltaVij_;}
  const imuBias::ConstantBias& biasHat() const { return biasHat_;}
  Vector biasHatVector() const { return biasHat_.vector();} // expensive
  const Matrix3& delPdelBiasAcc() const { return delPdelBiasAcc_;}
  const Matrix3& delPdelBiasOmega() const { return delPdelBiasOmega_;}
  const Matrix3& delVdelBiasAcc() const { return delVdelBiasAcc_;}
  const Matrix3& delVdelBiasOmega() const { return delVdelBiasOmega_;}

  /// Needed for testable
  void print(const std::string& s) const {
    PreintegratedRotation::print(s);
    std::cout << "  deltaPij [ " << deltaPij_.transpose() << " ]" << std::endl;
    std::cout << "  deltaVij [ " << deltaVij_.transpose() << " ]" << std::endl;
    biasHat_.print("  biasHat");
  }

  /// Needed for testable
  bool equals(const PreintegrationBase& expected, double tol) const {
    return PreintegratedRotation::equals(expected, tol)
    && biasHat_.equals(expected.biasHat_, tol)
    && equal_with_abs_tol(deltaPij_, expected.deltaPij_, tol)
    && equal_with_abs_tol(deltaVij_, expected.deltaVij_, tol)
    && equal_with_abs_tol(delPdelBiasAcc_, expected.delPdelBiasAcc_, tol)
    && equal_with_abs_tol(delPdelBiasOmega_, expected.delPdelBiasOmega_, tol)
    && equal_with_abs_tol(delVdelBiasAcc_, expected.delVdelBiasAcc_, tol)
    && equal_with_abs_tol(delVdelBiasOmega_, expected.delVdelBiasOmega_, tol);
  }

  /// Re-initialize PreintegratedMeasurements
  void resetIntegration(){
    PreintegratedRotation::resetIntegration();
    deltaPij_ = Vector3::Zero();
    deltaVij_ = Vector3::Zero();
    delPdelBiasAcc_ = Z_3x3;
    delPdelBiasOmega_ = Z_3x3;
    delVdelBiasAcc_ = Z_3x3;
    delVdelBiasOmega_ = Z_3x3;
  }

  /// Update preintegrated measurements
  void updatePreintegratedMeasurements(const Vector3& correctedAcc,
      const Rot3& incrR, double deltaT) {
    Matrix3 dRij = deltaRij(); // expensive
    Vector3 temp = dRij * correctedAcc * deltaT;
    if(!use2ndOrderIntegration_){
      deltaPij_ += deltaVij_ * deltaT;
    }else{
      deltaPij_ += deltaVij_ * deltaT + 0.5 * temp * deltaT;
    }
    deltaVij_ += temp;
    // TODO: we update rotation *after* the others. Is that correct?
    updateIntegratedRotationAndDeltaT(incrR,deltaT);
  }

  /// Update Jacobians to be used during preintegration
  void updatePreintegratedJacobians(const Vector3& correctedAcc,
      const Matrix3& Jr_theta_incr, const Rot3& incrR, double deltaT){
    Matrix3 dRij = deltaRij(); // expensive
    Matrix3 temp = -dRij * skewSymmetric(correctedAcc) * deltaT * delRdelBiasOmega();
    if (!use2ndOrderIntegration_) {
      delPdelBiasAcc_ += delVdelBiasAcc_ * deltaT;
      delPdelBiasOmega_ += delVdelBiasOmega_ * deltaT;
    } else {
      delPdelBiasAcc_ += delVdelBiasAcc_ * deltaT - 0.5 * dRij * deltaT * deltaT;
      delPdelBiasOmega_ += deltaT*(delVdelBiasOmega_ + temp * 0.5);
    }
    delVdelBiasAcc_ += -dRij * deltaT;
    delVdelBiasOmega_ += temp;
    // TODO: we update rotation *after* the others. Is that correct?
    update_delRdelBiasOmega(Jr_theta_incr,incrR,deltaT);
  }

  void correctMeasurementsByBiasAndSensorPose(const Vector3& measuredAcc,
      const Vector3& measuredOmega, Vector3& correctedAcc,
      Vector3& correctedOmega, boost::optional<const Pose3&> body_P_sensor) {
    correctedAcc = biasHat_.correctAccelerometer(measuredAcc);
    correctedOmega = biasHat_.correctGyroscope(measuredOmega);

    // Then compensate for sensor-body displacement: we express the quantities
    // (originally in the IMU frame) into the body frame
    if(body_P_sensor){
      Matrix3 body_R_sensor = body_P_sensor->rotation().matrix();
      correctedOmega = body_R_sensor * correctedOmega; // rotation rate vector in the body frame
      Matrix3 body_omega_body__cross = skewSymmetric(correctedOmega);
      correctedAcc = body_R_sensor * correctedAcc - body_omega_body__cross * body_omega_body__cross * body_P_sensor->translation().vector();
      // linear acceleration vector in the body frame
    }
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // This function is only used for test purposes (compare numerical derivatives wrt analytic ones)
  static inline Vector PreIntegrateIMUObservations_delta_vel(const Vector& msr_gyro_t, const Vector& msr_acc_t, const double msr_dt,
      const Vector3& delta_angles, const Vector& delta_vel_in_t0){
    // Note: all delta terms refer to an IMU\sensor system at t0
    Vector body_t_a_body = msr_acc_t;
    Rot3 R_t_to_t0 = Rot3::Expmap(delta_angles);
    return delta_vel_in_t0 + R_t_to_t0.matrix() * body_t_a_body * msr_dt;
  }

  // This function is only used for test purposes (compare numerical derivatives wrt analytic ones)
  static inline Vector PreIntegrateIMUObservations_delta_angles(const Vector& msr_gyro_t, const double msr_dt,
      const Vector3& delta_angles){
    // Note: all delta terms refer to an IMU\sensor system at t0
    // Calculate the corrected measurements using the Bias object
    Vector body_t_omega_body= msr_gyro_t;
    Rot3 R_t_to_t0 = Rot3::Expmap(delta_angles);
    R_t_to_t0    = R_t_to_t0 * Rot3::Expmap( body_t_omega_body*msr_dt );
    return Rot3::Logmap(R_t_to_t0);
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
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
  boost::optional<Pose3> body_P_sensor_;        ///< The pose of the sensor in the body frame
  bool use2ndOrderCoriolis_; ///< Controls whether higher order terms are included when calculating the Coriolis Effect

public:

  ImuBase() :
    gravity_(Vector3(0.0,0.0,9.81)), omegaCoriolis_(Vector3(0.0,0.0,0.0)),
    body_P_sensor_(boost::none), use2ndOrderCoriolis_(false) {}

  ImuBase(const Vector3& gravity, const Vector3& omegaCoriolis,
      boost::optional<const Pose3&> body_P_sensor = boost::none, const bool use2ndOrderCoriolis = false) :
        gravity_(gravity), omegaCoriolis_(omegaCoriolis),
        body_P_sensor_(body_P_sensor), use2ndOrderCoriolis_(use2ndOrderCoriolis) {}

  const Vector3& gravity() const { return gravity_; }
  const Vector3& omegaCoriolis() const { return omegaCoriolis_; }

};

} /// namespace gtsam
