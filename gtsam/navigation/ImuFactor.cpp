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
                  biasHat_(bias), deltaPij_(Vector3::Zero()), deltaVij_(Vector3::Zero()),
                  deltaRij_(Rot3()), deltaTij_(0.0),
                  delPdelBiasAcc_(Z_3x3), delPdelBiasOmega_(Z_3x3),
                  delVdelBiasAcc_(Z_3x3), delVdelBiasOmega_(Z_3x3),
                  delRdelBiasOmega_(Z_3x3), use2ndOrderIntegration_(use2ndOrderIntegration)
{
  measurementCovariance_.setZero();
  measurementCovariance_.block<3,3>(0,0) = integrationErrorCovariance;
  measurementCovariance_.block<3,3>(3,3) = measuredAccCovariance;
  measurementCovariance_.block<3,3>(6,6) = measuredOmegaCovariance;
  PreintMeasCov_.setZero(9,9);
}

//------------------------------------------------------------------------------
void ImuFactor::PreintegratedMeasurements::print(const string& s) const {
  cout << s << endl;
  biasHat_.print("  biasHat");
  cout << "  deltaTij " << deltaTij_ << endl;
  cout << "  deltaPij [ " << deltaPij_.transpose() << " ]" << endl;
  cout << "  deltaVij [ " << deltaVij_.transpose() << " ]" << endl;
  deltaRij_.print("  deltaRij ");
  cout << "  measurementCovariance = \n [ " << measurementCovariance_ << " ]" << endl;
  cout << "  PreintMeasCov = \n [ " << PreintMeasCov_ << " ]" << endl;
}

//------------------------------------------------------------------------------
bool ImuFactor::PreintegratedMeasurements::equals(const PreintegratedMeasurements& expected, double tol) const {
  return biasHat_.equals(expected.biasHat_, tol)
  && equal_with_abs_tol(measurementCovariance_, expected.measurementCovariance_, tol)
  && equal_with_abs_tol(deltaPij_, expected.deltaPij_, tol)
  && equal_with_abs_tol(deltaVij_, expected.deltaVij_, tol)
  && deltaRij_.equals(expected.deltaRij_, tol)
  && fabs(deltaTij_ - expected.deltaTij_) < tol
  && equal_with_abs_tol(delPdelBiasAcc_, expected.delPdelBiasAcc_, tol)
  && equal_with_abs_tol(delPdelBiasOmega_, expected.delPdelBiasOmega_, tol)
  && equal_with_abs_tol(delVdelBiasAcc_, expected.delVdelBiasAcc_, tol)
  && equal_with_abs_tol(delVdelBiasOmega_, expected.delVdelBiasOmega_, tol)
  && equal_with_abs_tol(delRdelBiasOmega_, expected.delRdelBiasOmega_, tol);
}

//------------------------------------------------------------------------------
void ImuFactor::PreintegratedMeasurements::resetIntegration(){
  deltaPij_ = Vector3::Zero();
  deltaVij_ = Vector3::Zero();
  deltaRij_ = Rot3();
  deltaTij_ = 0.0;
  delPdelBiasAcc_ = Z_3x3;
  delPdelBiasOmega_ = Z_3x3;
  delVdelBiasAcc_ = Z_3x3;
  delVdelBiasOmega_ = Z_3x3;
  delRdelBiasOmega_ = Z_3x3;
  PreintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void ImuFactor::PreintegratedMeasurements::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double deltaT,
    boost::optional<const Pose3&> body_P_sensor) {

  // NOTE: order is important here because each update uses old values (i.e., we have to update
  // jacobians and covariances before updating preintegrated measurements).

  // First we compensate the measurements for the bias
  Vector3 correctedAcc = biasHat_.correctAccelerometer(measuredAcc);
  Vector3 correctedOmega = biasHat_.correctGyroscope(measuredOmega);

  // Then compensate for sensor-body displacement: we express the quantities (originally in the IMU frame) into the body frame
  if(body_P_sensor){
    Matrix3 body_R_sensor = body_P_sensor->rotation().matrix();
    correctedOmega = body_R_sensor * correctedOmega; // rotation rate vector in the body frame
    Matrix3 body_omega_body__cross = skewSymmetric(correctedOmega);
    correctedAcc = body_R_sensor * correctedAcc - body_omega_body__cross * body_omega_body__cross * body_P_sensor->translation().vector();
    // linear acceleration vector in the body frame
  }

  const Vector3 theta_incr = correctedOmega * deltaT; // rotation vector describing rotation increment computed from the current rotation rate measurement
  const Rot3 Rincr = Rot3::Expmap(theta_incr); // rotation increment computed from the current rotation rate measurement

  const Matrix3 Jr_theta_incr = Rot3::rightJacobianExpMapSO3(theta_incr); // Right jacobian computed at theta_incr

  // Update Jacobians
  /* ----------------------------------------------------------------------------------------------------------------------- */
  if(!use2ndOrderIntegration_){
    delPdelBiasAcc_ += delVdelBiasAcc_ * deltaT;
    delPdelBiasOmega_ += delVdelBiasOmega_ * deltaT;
  }else{
    delPdelBiasAcc_ += delVdelBiasAcc_ * deltaT - 0.5 * deltaRij_.matrix() * deltaT*deltaT;
    delPdelBiasOmega_ += delVdelBiasOmega_ * deltaT - 0.5 * deltaRij_.matrix()
                                            * skewSymmetric(biasHat_.correctAccelerometer(measuredAcc)) * deltaT*deltaT * delRdelBiasOmega_;
  }
  delVdelBiasAcc_ += -deltaRij_.matrix() * deltaT;
  delVdelBiasOmega_ += -deltaRij_.matrix() * skewSymmetric(correctedAcc) * deltaT * delRdelBiasOmega_;
  delRdelBiasOmega_ = Rincr.inverse().matrix() * delRdelBiasOmega_ - Jr_theta_incr  * deltaT;

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
  PreintMeasCov_ = F * PreintMeasCov_ * F.transpose() + measurementCovariance_ * deltaT ;

  // Extended version, without approximation: Gt * Qt * G =(approx)= measurementCovariance_contTime * deltaT
  // This in only kept for documentation.
  //
  // Matrix G(9,9);
  // G << I_3x3 * deltaT, Z_3x3,  Z_3x3,
  //      Z_3x3, deltaRij.matrix() * deltaT, Z_3x3,
  //      Z_3x3, Z_3x3, Jrinv_theta_j * Jr_theta_incr * deltaT;
  //
  // PreintMeasCov = F * PreintMeasCov * F.transpose() + G * (1/deltaT) * measurementCovariance * G.transpose();

  // Update preintegrated measurements (this has to be done after the update of covariances and jacobians!)
  /* ----------------------------------------------------------------------------------------------------------------------- */
  if(!use2ndOrderIntegration_){
    deltaPij_ += deltaVij_ * deltaT;
  }else{
    deltaPij_ += deltaVij_ * deltaT + 0.5 * deltaRij_.matrix() * biasHat_.correctAccelerometer(measuredAcc) * deltaT*deltaT;
  }
  deltaVij_ += deltaRij_.matrix() * correctedAcc * deltaT;
  deltaRij_ = deltaRij_ * Rincr;
  deltaTij_ += deltaT;
}

//------------------------------------------------------------------------------
// ImuFactor methods
//------------------------------------------------------------------------------
ImuFactor::ImuFactor() :
    preintegratedMeasurements_(imuBias::ConstantBias(), Z_3x3, Z_3x3, Z_3x3), use2ndOrderCoriolis_(false){}

//------------------------------------------------------------------------------
ImuFactor::ImuFactor(
    Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
    const PreintegratedMeasurements& preintegratedMeasurements,
    const Vector3& gravity, const Vector3& omegaCoriolis,
    boost::optional<const Pose3&> body_P_sensor,
    const bool use2ndOrderCoriolis) :
        Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.PreintMeasCov_), pose_i, vel_i, pose_j, vel_j, bias),
        preintegratedMeasurements_(preintegratedMeasurements),
        gravity_(gravity),
        omegaCoriolis_(omegaCoriolis),
        body_P_sensor_(body_P_sensor),
        use2ndOrderCoriolis_(use2ndOrderCoriolis){
}

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
  preintegratedMeasurements_.print("  preintegrated measurements:");
  cout << "  gravity: [ " << gravity_.transpose() << " ]" << endl;
  cout << "  omegaCoriolis: [ " << omegaCoriolis_.transpose() << " ]" << endl;
  this->noiseModel_->print("  noise model: ");
  if(this->body_P_sensor_)
    this->body_P_sensor_->print("  sensor pose in body frame: ");
}

//------------------------------------------------------------------------------
bool ImuFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This *e =  dynamic_cast<const This*> (&expected);
  return e != NULL && Base::equals(*e, tol)
  && preintegratedMeasurements_.equals(e->preintegratedMeasurements_, tol)
  && equal_with_abs_tol(gravity_, e->gravity_, tol)
  && equal_with_abs_tol(omegaCoriolis_, e->omegaCoriolis_, tol)
  && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
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

//------------------------------------------------------------------------------
PoseVelocity ImuFactor::Predict(const Pose3& pose_i, const Vector3& vel_i,
    const imuBias::ConstantBias& bias, const PreintegratedMeasurements preintegratedMeasurements,
    const Vector3& gravity, const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis)
{

  const double& deltaTij = preintegratedMeasurements.deltaTij_;
  const Vector3 biasAccIncr = bias.accelerometer() - preintegratedMeasurements.biasHat_.accelerometer();
  const Vector3 biasOmegaIncr = bias.gyroscope() - preintegratedMeasurements.biasHat_.gyroscope();

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
  return PoseVelocity(pose_j, vel_j);
}

} /// namespace gtsam
