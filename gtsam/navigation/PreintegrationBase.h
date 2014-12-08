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
 * Struct to hold all state variables of returned by Predict function
 */
struct PoseVelocityBias {
  Pose3 pose;
  Vector3 velocity;
  imuBias::ConstantBias bias;

  PoseVelocityBias(const Pose3& _pose, const Vector3& _velocity,
      const imuBias::ConstantBias _bias) :
        pose(_pose), velocity(_velocity), bias(_bias) {
  }
};

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
      const Rot3& incrR, const double deltaT) {
    Matrix3 dRij = deltaRij(); // expensive
    Vector3 temp = dRij * correctedAcc * deltaT;
    if(!use2ndOrderIntegration_){
      deltaPij_ += deltaVij_ * deltaT;
    }else{
      deltaPij_ += deltaVij_ * deltaT + 0.5 * temp * deltaT;
    }
    deltaVij_ += temp;
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

  /// Predict state at time j
  //------------------------------------------------------------------------------
  PoseVelocityBias predict(const Pose3& pose_i, const Vector3& vel_i,
      const imuBias::ConstantBias& bias_i, const Vector3& gravity,
      const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis = false) const {

    const Vector3 biasAccIncr = bias_i.accelerometer() - biasHat().accelerometer();
    const Vector3 biasOmegaIncr = bias_i.gyroscope() - biasHat().gyroscope();

    const Rot3& Rot_i = pose_i.rotation();
    const Vector3& pos_i = pose_i.translation().vector();

    // Predict state at time j
    /* ---------------------------------------------------------------------------------------------------- */
    Vector3 pos_j =  pos_i + Rot_i.matrix() * (deltaPij()
        + delPdelBiasAcc() * biasAccIncr
        + delPdelBiasOmega() * biasOmegaIncr)
        + vel_i * deltaTij()
        - omegaCoriolis.cross(vel_i) * deltaTij()*deltaTij()  // Coriolis term - we got rid of the 2 wrt ins paper
        + 0.5 * gravity * deltaTij()*deltaTij();

    Vector3 vel_j = Vector3(vel_i + Rot_i.matrix() * (deltaVij()
        + delVdelBiasAcc() * biasAccIncr
        + delVdelBiasOmega() * biasOmegaIncr)
        - 2 * omegaCoriolis.cross(vel_i) * deltaTij()  // Coriolis term
        + gravity * deltaTij());

    if(use2ndOrderCoriolis){
      pos_j += - 0.5 * omegaCoriolis.cross(omegaCoriolis.cross(pos_i)) * deltaTij()*deltaTij();  // 2nd order coriolis term for position
      vel_j += - omegaCoriolis.cross(omegaCoriolis.cross(pos_i)) * deltaTij(); // 2nd order term for velocity
    }

    const Rot3 deltaRij_biascorrected = biascorrectedDeltaRij(biasOmegaIncr);
    // deltaRij_biascorrected = deltaRij * expmap(delRdelBiasOmega * biasOmegaIncr)

    Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);
    Vector3 theta_corrected = theta_biascorrected  -
        Rot_i.inverse().matrix() * omegaCoriolis * deltaTij(); // Coriolis term
    const Rot3 deltaRij_corrected =
        Rot3::Expmap( theta_corrected );
    const Rot3 Rot_j = Rot_i.compose( deltaRij_corrected  );

    Pose3 pose_j = Pose3( Rot_j, Point3(pos_j) );
    return PoseVelocityBias(pose_j, vel_j, bias_i); // bias is predicted as a constant
  }

  /// Compute errors w.r.t. preintegrated measurements and jacobians wrt pose_i, vel_i, bias_i, pose_j, bias_j
  //------------------------------------------------------------------------------
  Vector computeErrorAndJacobians(const Pose3& pose_i, const Vector3& vel_i,
      const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i, const Vector3& gravity,
      const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis,
      boost::optional<Matrix&> H1, boost::optional<Matrix&> H2,
      boost::optional<Matrix&> H3, boost::optional<Matrix&> H4,
      boost::optional<Matrix&> H5) const {

    // We need the mismatch w.r.t. the biases used for preintegration
    const Vector3 biasAccIncr = bias_i.accelerometer() - biasHat().accelerometer();
    const Vector3 biasOmegaIncr = bias_i.gyroscope() - biasHat().gyroscope();

    // we give some shorter name to rotations and translations
    const Rot3& Ri = pose_i.rotation();
    const Rot3& Rj = pose_j.rotation();
    const Vector3& pos_j = pose_j.translation().vector();

    // Jacobian computation
    /* ---------------------------------------------------------------------------------------------------- */
    // Get Get so<3> version of bias corrected rotation
    // If H5 is asked for, we will need the Jacobian, which we store in H5
    // H5 will then be corrected below to take into account the Coriolis effect
    Vector3 theta_biascorrected = biascorrectedThetaRij(biasOmegaIncr, H5);

    // Coriolis term, note inconsistent with AHRS, where coriolisHat is *after* integration
    const Matrix3 omegaCoriolisHat = skewSymmetric(omegaCoriolis);
    const Vector3 coriolis = integrateCoriolis(Ri, omegaCoriolis);
    Vector3 theta_corrected = theta_biascorrected  - coriolis;

    Rot3 deltaRij_corrected, fRhat;
    Vector3 fR;

    // Accessory matrix, used to build the jacobians
    Matrix3 Jr_theta_bcc, Jtheta, Jrinv_fRhat;

    // This is done to save computation: we ask for the jacobians only when they are needed
    if(H1 || H2 || H3 || H4 ||  H5){
      deltaRij_corrected = Rot3::Expmap( theta_corrected, Jr_theta_bcc);
      // Residual rotation error
      fRhat = deltaRij_corrected.between(Ri.between(Rj));
      fR = Rot3::Logmap(fRhat, Jrinv_fRhat);
      Jtheta = -Jr_theta_bcc  * skewSymmetric(coriolis);
    }else{
      deltaRij_corrected = Rot3::Expmap( theta_corrected);
      // Residual rotation error
      fRhat = deltaRij_corrected.between(Ri.between(Rj));
      fR = Rot3::Logmap(fRhat);
    }

    if(H1) {
      H1->resize(9,6);

      Matrix3 dfPdPi;
      Matrix3 dfVdPi;
      if(use2ndOrderCoriolis){
        dfPdPi = - Ri.matrix() + 0.5 * omegaCoriolisHat * omegaCoriolisHat * Ri.matrix() * deltaTij()*deltaTij();
        dfVdPi = omegaCoriolisHat * omegaCoriolisHat * Ri.matrix() * deltaTij();
      }
      else{
        dfPdPi = - Ri.matrix();
        dfVdPi = Z_3x3;
      }
      (*H1) <<
          // dfP/dRi
          Ri.matrix() * skewSymmetric(deltaPij()
          + delPdelBiasOmega() * biasOmegaIncr + delPdelBiasAcc() * biasAccIncr),
          // dfP/dPi
          dfPdPi,
          // dfV/dRi
          Ri.matrix() * skewSymmetric(deltaVij()
          + delVdelBiasOmega() * biasOmegaIncr + delVdelBiasAcc() * biasAccIncr),
          // dfV/dPi
          dfVdPi,
          // dfR/dRi
          Jrinv_fRhat *  (- Rj.between(Ri).matrix() - fRhat.inverse().matrix() * Jtheta),
          // dfR/dPi
          Z_3x3;
    }
    if(H2) {
      H2->resize(9,3);
      (*H2) <<
          // dfP/dVi
          - I_3x3 * deltaTij()
          + omegaCoriolisHat * deltaTij() * deltaTij(),  // Coriolis term - we got rid of the 2 wrt ins paper
          // dfV/dVi
          - I_3x3
          + 2 * omegaCoriolisHat * deltaTij(), // Coriolis term
          // dfR/dVi
          Z_3x3;
    }
    if(H3) {
      H3->resize(9,6);
      (*H3) <<
          // dfP/dPosej
          Z_3x3, Rj.matrix(),
          // dfV/dPosej
          Matrix::Zero(3,6),
          // dfR/dPosej
          Jrinv_fRhat *  ( I_3x3 ), Z_3x3;
    }
    if(H4) {
      H4->resize(9,3);
      (*H4) <<
          // dfP/dVj
          Z_3x3,
          // dfV/dVj
          I_3x3,
          // dfR/dVj
          Z_3x3;
    }
    if(H5) {
      // H5 by this point already contains 3*3 biascorrectedThetaRij derivative
      const Matrix3 JbiasOmega = Jr_theta_bcc * (*H5);
      H5->resize(9,6);
      (*H5) <<
          // dfP/dBias
          - Ri.matrix() * delPdelBiasAcc(),
          - Ri.matrix() * delPdelBiasOmega(),
          // dfV/dBias
          - Ri.matrix() * delVdelBiasAcc(),
          - Ri.matrix() * delVdelBiasOmega(),
          // dfR/dBias
          Matrix::Zero(3,3),
          Jrinv_fRhat * ( - fRhat.inverse().matrix() * JbiasOmega);
    }

    // Evaluate residual error, according to [3]
    /* ---------------------------------------------------------------------------------------------------- */
    PoseVelocityBias predictedState_j = predict(pose_i, vel_i, bias_i, gravity,
        omegaCoriolis, use2ndOrderCoriolis);

    const Vector3 fp = pos_j - predictedState_j.pose.translation().vector();

    const Vector3 fv = vel_j - predictedState_j.velocity;

    // fR was computes earlier. Note: it is the same as: dR = (predictedState_j.pose.translation()).between(Rot_j)

    Vector r(9); r << fp, fv, fR;
    return r;
  }

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
