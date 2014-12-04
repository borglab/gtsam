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
