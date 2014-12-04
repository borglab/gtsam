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

/* GTSAM includes */
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationBase.h>

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

class ImuFactorBase {

protected:

  Vector3 gravity_;
  Vector3 omegaCoriolis_;
  boost::optional<Pose3> body_P_sensor_;        ///< The pose of the sensor in the body frame
  bool use2ndOrderCoriolis_; ///< Controls whether higher order terms are included when calculating the Coriolis Effect

public:

  ImuFactorBase() :
    gravity_(Vector3(0.0,0.0,9.81)), omegaCoriolis_(Vector3(0.0,0.0,0.0)),
    body_P_sensor_(boost::none), use2ndOrderCoriolis_(false) {}

  ImuFactorBase(const Vector3& gravity, const Vector3& omegaCoriolis,
      boost::optional<const Pose3&> body_P_sensor = boost::none, const bool use2ndOrderCoriolis = false) :
        gravity_(gravity), omegaCoriolis_(omegaCoriolis),
        body_P_sensor_(body_P_sensor), use2ndOrderCoriolis_(use2ndOrderCoriolis) {}

  const Vector3& gravity() const { return gravity_; }
  const Vector3& omegaCoriolis() const { return omegaCoriolis_; }

  /// Needed for testable
  //------------------------------------------------------------------------------
  void print(const std::string& s) const {
    std::cout << "  gravity: [ " << gravity_.transpose() << " ]" << std::endl;
    std::cout << "  omegaCoriolis: [ " << omegaCoriolis_.transpose() << " ]" << std::endl;
    std::cout << "  use2ndOrderCoriolis: [ " << use2ndOrderCoriolis_ << " ]" << std::endl;
    if(this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
  }

  /// Needed for testable
  //------------------------------------------------------------------------------
  bool equals(const ImuFactorBase& expected, double tol) const {
    return equal_with_abs_tol(gravity_, expected.gravity_, tol)
    && equal_with_abs_tol(omegaCoriolis_, expected.omegaCoriolis_, tol)
    && (use2ndOrderCoriolis_ == expected.use2ndOrderCoriolis_)
    && ((!body_P_sensor_ && !expected.body_P_sensor_) ||
        (body_P_sensor_ && expected.body_P_sensor_ && body_P_sensor_->equals(*expected.body_P_sensor_)));
  }

  /// Compute errors w.r.t. preintegrated measurements and jacobians wrt pose_i, vel_i, bias_i, pose_j, bias_j
  //------------------------------------------------------------------------------
  Vector computeErrorAndJacobians(const PreintegrationBase& preintegratedMeasurements_, const Pose3& pose_i, const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
      const imuBias::ConstantBias& bias_i, boost::optional<Matrix&> H1,  boost::optional<Matrix&> H2,
      boost::optional<Matrix&> H3,  boost::optional<Matrix&> H4, boost::optional<Matrix&> H5) const{

    const double& deltaTij = preintegratedMeasurements_.deltaTij_;
    const Vector3 biasAccIncr = bias_i.accelerometer() - preintegratedMeasurements_.biasHat_.accelerometer();
    const Vector3 biasOmegaIncr = bias_i.gyroscope() - preintegratedMeasurements_.biasHat_.gyroscope();

    // we give some shorter name to rotations and translations
    const Rot3 Rot_i = pose_i.rotation();
    const Rot3 Rot_j = pose_j.rotation();
    const Vector3 pos_i = pose_i.translation().vector();
    const Vector3 pos_j = pose_j.translation().vector();

    // We compute factor's Jacobians
    /* ---------------------------------------------------------------------------------------------------- */
    const Rot3 deltaRij_biascorrected = preintegratedMeasurements_.deltaRij_.retract(preintegratedMeasurements_.delRdelBiasOmega_ * biasOmegaIncr, Rot3::EXPMAP);
    // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)

    Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);

    Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected  -
        Rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij; // Coriolis term

    const Rot3 deltaRij_biascorrected_corioliscorrected =
        Rot3::Expmap( theta_biascorrected_corioliscorrected );

    const Rot3 fRhat = deltaRij_biascorrected_corioliscorrected.between(Rot_i.between(Rot_j));

    const Matrix3 Jr_theta_bcc = Rot3::rightJacobianExpMapSO3(theta_biascorrected_corioliscorrected);

    const Matrix3 Jtheta = -Jr_theta_bcc  * skewSymmetric(Rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij);

    const Matrix3 Jrinv_fRhat = Rot3::rightJacobianExpMapSO3inverse(Rot3::Logmap(fRhat));

    if(H1) {
      H1->resize(9,6);

      Matrix3 dfPdPi;
      Matrix3 dfVdPi;
      if(use2ndOrderCoriolis_){
        dfPdPi = - Rot_i.matrix() + 0.5 * skewSymmetric(omegaCoriolis_) * skewSymmetric(omegaCoriolis_) * Rot_i.matrix() * deltaTij*deltaTij;
        dfVdPi = skewSymmetric(omegaCoriolis_) * skewSymmetric(omegaCoriolis_) * Rot_i.matrix() * deltaTij;
      }
      else{
        dfPdPi = - Rot_i.matrix();
        dfVdPi = Z_3x3;
      }

      (*H1) <<
          // dfP/dRi
          Rot_i.matrix() * skewSymmetric(preintegratedMeasurements_.deltaPij_
              + preintegratedMeasurements_.delPdelBiasOmega_ * biasOmegaIncr + preintegratedMeasurements_.delPdelBiasAcc_ * biasAccIncr),
              // dfP/dPi
              dfPdPi,
              // dfV/dRi
              Rot_i.matrix() * skewSymmetric(preintegratedMeasurements_.deltaVij_
                  + preintegratedMeasurements_.delVdelBiasOmega_ * biasOmegaIncr + preintegratedMeasurements_.delVdelBiasAcc_ * biasAccIncr),
                  // dfV/dPi
                  dfVdPi,
                  // dfR/dRi
                  Jrinv_fRhat *  (- Rot_j.between(Rot_i).matrix() - fRhat.inverse().matrix() * Jtheta),
                  // dfR/dPi
                  Z_3x3;
    }

    if(H2) {
      H2->resize(9,3);
      (*H2) <<
          // dfP/dVi
          - I_3x3 * deltaTij
          + skewSymmetric(omegaCoriolis_) * deltaTij * deltaTij,  // Coriolis term - we got rid of the 2 wrt ins paper
          // dfV/dVi
          - I_3x3
          + 2 * skewSymmetric(omegaCoriolis_) * deltaTij, // Coriolis term
          // dfR/dVi
          Z_3x3;
    }

    if(H3) {
      H3->resize(9,6);
      (*H3) <<
          // dfP/dPosej
          Z_3x3, Rot_j.matrix(),
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
      const Matrix3 Jrinv_theta_bc = Rot3::rightJacobianExpMapSO3inverse(theta_biascorrected);
      const Matrix3 Jr_JbiasOmegaIncr = Rot3::rightJacobianExpMapSO3(preintegratedMeasurements_.delRdelBiasOmega_ * biasOmegaIncr);
      const Matrix3 JbiasOmega = Jr_theta_bcc * Jrinv_theta_bc * Jr_JbiasOmegaIncr * preintegratedMeasurements_.delRdelBiasOmega_;

      H5->resize(9,6);
      (*H5) <<
          // dfP/dBias
          - Rot_i.matrix() * preintegratedMeasurements_.delPdelBiasAcc_,
          - Rot_i.matrix() * preintegratedMeasurements_.delPdelBiasOmega_,
          // dfV/dBias
          - Rot_i.matrix() * preintegratedMeasurements_.delVdelBiasAcc_,
          - Rot_i.matrix() * preintegratedMeasurements_.delVdelBiasOmega_,
          // dfR/dBias
          Matrix::Zero(3,3),
          Jrinv_fRhat * ( - fRhat.inverse().matrix() * JbiasOmega);
    }

    // Evaluate residual error, according to [3]
    /* ---------------------------------------------------------------------------------------------------- */
    const Vector3 fp =
        pos_j - pos_i
        - Rot_i.matrix() * (preintegratedMeasurements_.deltaPij_
            + preintegratedMeasurements_.delPdelBiasAcc_ * biasAccIncr
            + preintegratedMeasurements_.delPdelBiasOmega_ * biasOmegaIncr)
            - vel_i * deltaTij
            + skewSymmetric(omegaCoriolis_) * vel_i * deltaTij*deltaTij  // Coriolis term - we got rid of the 2 wrt ins paper
            - 0.5 * gravity_ * deltaTij*deltaTij;

    const Vector3 fv =
        vel_j - vel_i - Rot_i.matrix() * (preintegratedMeasurements_.deltaVij_
            + preintegratedMeasurements_.delVdelBiasAcc_ * biasAccIncr
            + preintegratedMeasurements_.delVdelBiasOmega_ * biasOmegaIncr)
            + 2 * skewSymmetric(omegaCoriolis_) * vel_i * deltaTij  // Coriolis term
            - gravity_ * deltaTij;

    const Vector3 fR = Rot3::Logmap(fRhat);

    Vector r(9); r << fp, fv, fR;
    return r;
  }

  /// Predict state at time j
  //------------------------------------------------------------------------------
  static PoseVelocityBias predict(const Pose3& pose_i, const Vector3& vel_i,
      const imuBias::ConstantBias& bias_i,
      const PreintegrationBase& preintegratedMeasurements,
      const Vector3& gravity, const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis){

    const double& deltaTij = preintegratedMeasurements.deltaTij_;
    const Vector3 biasAccIncr = bias_i.accelerometer() - preintegratedMeasurements.biasHat_.accelerometer();
    const Vector3 biasOmegaIncr = bias_i.gyroscope() - preintegratedMeasurements.biasHat_.gyroscope();

    const Rot3 Rot_i = pose_i.rotation();
    const Vector3 pos_i = pose_i.translation().vector();

    // Predict state at time j
    /* ---------------------------------------------------------------------------------------------------- */
    Vector3 pos_j =  pos_i + Rot_i.matrix() * (preintegratedMeasurements.deltaPij_
        + preintegratedMeasurements.delPdelBiasAcc_ * biasAccIncr
        + preintegratedMeasurements.delPdelBiasOmega_ * biasOmegaIncr)
        + vel_i * deltaTij
        - skewSymmetric(omegaCoriolis) * vel_i * deltaTij*deltaTij  // Coriolis term - we got rid of the 2 wrt ins paper
        + 0.5 * gravity * deltaTij*deltaTij;

    Vector3 vel_j = Vector3(vel_i + Rot_i.matrix() * (preintegratedMeasurements.deltaVij_
        + preintegratedMeasurements.delVdelBiasAcc_ * biasAccIncr
        + preintegratedMeasurements.delVdelBiasOmega_ * biasOmegaIncr)
        - 2 * skewSymmetric(omegaCoriolis) * vel_i * deltaTij  // Coriolis term
        + gravity * deltaTij);

    if(use2ndOrderCoriolis){
      pos_j += - 0.5 * skewSymmetric(omegaCoriolis) * skewSymmetric(omegaCoriolis) * pos_i * deltaTij*deltaTij;  // 2nd order coriolis term for position
      vel_j += - skewSymmetric(omegaCoriolis) * skewSymmetric(omegaCoriolis) * pos_i * deltaTij; // 2nd order term for velocity
    }

    const Rot3 deltaRij_biascorrected = preintegratedMeasurements.deltaRij_.retract(preintegratedMeasurements.delRdelBiasOmega_ * biasOmegaIncr, Rot3::EXPMAP);
    // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)
    Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);
    Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected  -
        Rot_i.inverse().matrix() * omegaCoriolis * deltaTij; // Coriolis term
    const Rot3 deltaRij_biascorrected_corioliscorrected =
        Rot3::Expmap( theta_biascorrected_corioliscorrected );
    const Rot3 Rot_j = Rot_i.compose( deltaRij_biascorrected_corioliscorrected  );

    Pose3 pose_j = Pose3( Rot_j, Point3(pos_j) );
    return PoseVelocityBias(pose_j, vel_j, bias_i); // bias is predicted as a constant
  }

};

} /// namespace gtsam
