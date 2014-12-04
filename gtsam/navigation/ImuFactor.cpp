/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ImuFactor.cpp
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#include <gtsam/navigation/ImuFactor.h>

/* External or standard includes */
#include <ostream>

namespace gtsam {

using namespace std;

//------------------------------------------------------------------------------
// Inner class PreintegratedMeasurements
//------------------------------------------------------------------------------
ImuFactor::PreintegratedMeasurements::PreintegratedMeasurements(
    const imuBias::ConstantBias& bias, const Matrix3& measuredAccCovariance,
    const Matrix3& measuredOmegaCovariance, const Matrix3& integrationErrorCovariance,
    const bool use2ndOrderIntegration) :
        PreintegrationBase(bias, use2ndOrderIntegration)
{
  measurementCovariance_.setZero();
  measurementCovariance_.block<3,3>(0,0) = integrationErrorCovariance;
  measurementCovariance_.block<3,3>(3,3) = measuredAccCovariance;
  measurementCovariance_.block<3,3>(6,6) = measuredOmegaCovariance;
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void ImuFactor::PreintegratedMeasurements::print(const string& s) const {
  PreintegrationBase::print(s);
  cout << "  measurementCovariance = \n [ " << measurementCovariance_ << " ]" << endl;
  cout << "  preintMeasCov = \n [ " << preintMeasCov_ << " ]" << endl;
}

//------------------------------------------------------------------------------
bool ImuFactor::PreintegratedMeasurements::equals(const PreintegratedMeasurements& expected, double tol) const {
  return equal_with_abs_tol(measurementCovariance_, expected.measurementCovariance_, tol)
  && equal_with_abs_tol(preintMeasCov_, expected.preintMeasCov_, tol)
  && PreintegrationBase::equals(expected, tol);
}

//------------------------------------------------------------------------------
void ImuFactor::PreintegratedMeasurements::resetIntegration(){
  PreintegrationBase::resetIntegration();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void ImuFactor::PreintegratedMeasurements::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double deltaT,
    boost::optional<const Pose3&> body_P_sensor) {

  // NOTE: order is important here because each update uses old values (i.e., we have to update
  // jacobians and covariances before updating preintegrated measurements).

  Vector3 correctedAcc, correctedOmega;
  correctMeasurementsByBiasAndSensorPose(measuredAcc, measuredOmega, correctedAcc, correctedOmega, body_P_sensor);

  const Vector3 theta_incr = correctedOmega * deltaT; // rotation vector describing rotation increment computed from the current rotation rate measurement
  const Rot3 Rincr = Rot3::Expmap(theta_incr); // rotation increment computed from the current rotation rate measurement
  const Matrix3 Jr_theta_incr = Rot3::rightJacobianExpMapSO3(theta_incr); // Right jacobian computed at theta_incr

  // Update Jacobians
  /* ----------------------------------------------------------------------------------------------------------------------- */
  updatePreintegratedJacobians(correctedAcc, Jr_theta_incr, Rincr, deltaT);

  // Update preintegrated measurements covariance
  // as in [2] we consider a first order propagation that can be seen as a prediction phase in an EKF framework
  /* ----------------------------------------------------------------------------------------------------------------------- */
  const Vector3 theta_i = Rot3::Logmap(deltaRij_); // parametrization of so(3)
  const Matrix3 Jr_theta_i = Rot3::rightJacobianExpMapSO3(theta_i);

  Rot3 Rot_j = deltaRij_ * Rincr;
  const Vector3 theta_j = Rot3::Logmap(Rot_j); // parametrization of so(3)
  const Matrix3 Jrinv_theta_j = Rot3::rightJacobianExpMapSO3inverse(theta_j);

  Matrix H_pos_pos    = I_3x3;
  Matrix H_pos_vel    = I_3x3 * deltaT;
  Matrix H_pos_angles = Z_3x3;

  Matrix H_vel_pos    = Z_3x3;
  Matrix H_vel_vel    = I_3x3;
  Matrix H_vel_angles = - deltaRij_.matrix() * skewSymmetric(correctedAcc) * Jr_theta_i * deltaT;
  // analytic expression corresponding to the following numerical derivative
  // Matrix H_vel_angles = numericalDerivative11<Vector3, Vector3>(boost::bind(&PreIntegrateIMUObservations_delta_vel, correctedOmega, correctedAcc, deltaT, _1, deltaVij), theta_i);

  Matrix H_angles_pos   = Z_3x3;
  Matrix H_angles_vel    = Z_3x3;
  Matrix H_angles_angles = Jrinv_theta_j * Rincr.inverse().matrix() * Jr_theta_i;
  // analytic expression corresponding to the following numerical derivative
  // Matrix H_angles_angles = numericalDerivative11<Vector3, Vector3>(boost::bind(&PreIntegrateIMUObservations_delta_angles, correctedOmega, deltaT, _1), thetaij);

  // overall Jacobian wrt preintegrated measurements (df/dx)
  Matrix F(9,9);
  F << H_pos_pos, H_pos_vel,  H_pos_angles,
      H_vel_pos, H_vel_vel, H_vel_angles,
      H_angles_pos, H_angles_vel, H_angles_angles;

  // first order uncertainty propagation:
  // the deltaT allows to pass from continuous time noise to discrete time noise
  // measurementCovariance_discrete = measurementCovariance_contTime * (1/deltaT)
  // Gt * Qt * G =(approx)= measurementCovariance_discrete * deltaT^2 = measurementCovariance_contTime * deltaT
  preintMeasCov_ = F * preintMeasCov_ * F.transpose() + measurementCovariance_ * deltaT ;

  // Extended version, without approximation: Gt * Qt * G =(approx)= measurementCovariance_contTime * deltaT
  // This in only kept for documentation.
  //
  // Matrix G(9,9);
  // G << I_3x3 * deltaT, Z_3x3,  Z_3x3,
  //      Z_3x3, deltaRij.matrix() * deltaT, Z_3x3,
  //      Z_3x3, Z_3x3, Jrinv_theta_j * Jr_theta_incr * deltaT;
  //
  // preintMeasCov = F * preintMeasCov * F.transpose() + G * (1/deltaT) * measurementCovariance * G.transpose();

  // Update preintegrated measurements (this has to be done after the update of covariances and jacobians!)
  /* ----------------------------------------------------------------------------------------------------------------------- */
  updatePreintegratedMeasurements(correctedAcc, Rincr, deltaT);
}

//------------------------------------------------------------------------------
// ImuFactor methods
//------------------------------------------------------------------------------
ImuFactor::ImuFactor() :
    ImuFactorBase(), preintegratedMeasurements_(imuBias::ConstantBias(), Z_3x3, Z_3x3, Z_3x3) {}

//------------------------------------------------------------------------------
ImuFactor::ImuFactor(
    Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
    const PreintegratedMeasurements& preintegratedMeasurements,
    const Vector3& gravity, const Vector3& omegaCoriolis,
    boost::optional<const Pose3&> body_P_sensor,
    const bool use2ndOrderCoriolis) :
        Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.preintMeasCov_), pose_i, vel_i, pose_j, vel_j, bias),
        ImuFactorBase(gravity, omegaCoriolis, body_P_sensor, use2ndOrderCoriolis),
        preintegratedMeasurements_(preintegratedMeasurements) {}

//------------------------------------------------------------------------------
gtsam::NonlinearFactor::shared_ptr ImuFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
void ImuFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "ImuFactor("
      << keyFormatter(this->key1()) << ","
      << keyFormatter(this->key2()) << ","
      << keyFormatter(this->key3()) << ","
      << keyFormatter(this->key4()) << ","
      << keyFormatter(this->key5()) << ")\n";
  ImuFactorBase::print("");
  preintegratedMeasurements_.print("  preintegrated measurements:");
  this->noiseModel_->print("  noise model: ");
}

//------------------------------------------------------------------------------
bool ImuFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This *e =  dynamic_cast<const This*> (&expected);
  return e != NULL && Base::equals(*e, tol)
  && preintegratedMeasurements_.equals(e->preintegratedMeasurements_, tol)
  && ImuFactorBase::equals(*e, tol);
}

//------------------------------------------------------------------------------
Vector ImuFactor::evaluateError(const Pose3& pose_i, const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias,
    boost::optional<Matrix&> H1,  boost::optional<Matrix&> H2,
    boost::optional<Matrix&> H3,  boost::optional<Matrix&> H4,
    boost::optional<Matrix&> H5) const
{

  const double& deltaTij = preintegratedMeasurements_.deltaTij_;
  const Vector3 biasAccIncr = bias.accelerometer() - preintegratedMeasurements_.biasHat_.accelerometer();
  const Vector3 biasOmegaIncr = bias.gyroscope() - preintegratedMeasurements_.biasHat_.gyroscope();

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

} /// namespace gtsam
