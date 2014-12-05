/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  CombinedImuFactor.cpp
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#include <gtsam/navigation/CombinedImuFactor.h>

/* External or standard includes */
#include <ostream>

namespace gtsam {

using namespace std;

//------------------------------------------------------------------------------
// Inner class CombinedPreintegratedMeasurements
//------------------------------------------------------------------------------
CombinedImuFactor::CombinedPreintegratedMeasurements::CombinedPreintegratedMeasurements(
    const imuBias::ConstantBias& bias, const Matrix3& measuredAccCovariance,
    const Matrix3& measuredOmegaCovariance, const Matrix3& integrationErrorCovariance,
    const Matrix3& biasAccCovariance, const Matrix3& biasOmegaCovariance,
    const Matrix& biasAccOmegaInit, const bool use2ndOrderIntegration) :
        PreintegrationBase(bias, use2ndOrderIntegration)
{
  measurementCovariance_.setZero();
  measurementCovariance_.block<3,3>(0,0) = integrationErrorCovariance;
  measurementCovariance_.block<3,3>(3,3) = measuredAccCovariance;
  measurementCovariance_.block<3,3>(6,6) = measuredOmegaCovariance;
  measurementCovariance_.block<3,3>(9,9) = biasAccCovariance;
  measurementCovariance_.block<3,3>(12,12) = biasOmegaCovariance;
  measurementCovariance_.block<6,6>(15,15) = biasAccOmegaInit;
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void CombinedImuFactor::CombinedPreintegratedMeasurements::print(const string& s) const{
  PreintegrationBase::print(s);
  cout << "  measurementCovariance [ " << measurementCovariance_ << " ]" << endl;
  cout << "  preintMeasCov [ " << preintMeasCov_ << " ]" << endl;
}

//------------------------------------------------------------------------------
bool CombinedImuFactor::CombinedPreintegratedMeasurements::equals(const CombinedPreintegratedMeasurements& expected, double tol) const{
  return equal_with_abs_tol(measurementCovariance_, expected.measurementCovariance_, tol)
      && equal_with_abs_tol(preintMeasCov_, expected.preintMeasCov_, tol)
      && PreintegrationBase::equals(expected, tol);
}

//------------------------------------------------------------------------------
void CombinedImuFactor::CombinedPreintegratedMeasurements::resetIntegration(){
  PreintegrationBase::resetIntegration();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void CombinedImuFactor::CombinedPreintegratedMeasurements::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega,
    double deltaT, boost::optional<const Pose3&> body_P_sensor) {

  // NOTE: order is important here because each update uses old values, e.g., velocity and position updates are based on previous rotation estimate.
  // (i.e., we have to update jacobians and covariances before updating preintegrated measurements).

  Vector3 correctedAcc, correctedOmega;
  correctMeasurementsByBiasAndSensorPose(measuredAcc, measuredOmega, correctedAcc, correctedOmega, body_P_sensor);

  const Vector3 theta_incr = correctedOmega * deltaT; // rotation vector describing rotation increment computed from the current rotation rate measurement
  const Rot3 Rincr = Rot3::Expmap(theta_incr); // rotation increment computed from the current rotation rate measurement
  const Matrix3 Jr_theta_incr = Rot3::rightJacobianExpMapSO3(theta_incr); // Right jacobian computed at theta_incr

  // Update Jacobians
  /* ----------------------------------------------------------------------------------------------------------------------- */
  updatePreintegratedJacobians(correctedAcc, Jr_theta_incr, Rincr, deltaT);

  // Update preintegrated measurements covariance: as in [2] we consider a first order propagation that
  // can be seen as a prediction phase in an EKF framework. In this implementation, contrarily to [2] we
  // consider the uncertainty of the bias selection and we keep correlation between biases and preintegrated measurements
  /* ----------------------------------------------------------------------------------------------------------------------- */
  const Vector3 theta_i = thetaRij(); // super-expensive parametrization of so(3)
  const Matrix3 Jr_theta_i = Rot3::rightJacobianExpMapSO3(theta_i);

  // Update preintegrated measurements. TODO Frank moved from end of this function !!!
  updatePreintegratedMeasurements(correctedAcc, Rincr, deltaT);

  const Vector3 theta_j = thetaRij(); // super-expensive parametrization of so(3)
  const Matrix3 Jrinv_theta_j = Rot3::rightJacobianExpMapSO3inverse(theta_j);

  // Single Jacobians to propagate covariance
  Matrix3 H_pos_pos    = I_3x3;
  Matrix3 H_pos_vel    = I_3x3 * deltaT;
  Matrix3 H_pos_angles = Z_3x3;

  Matrix3 H_vel_pos    = Z_3x3;
  Matrix3 H_vel_vel    = I_3x3;
  Matrix3 H_vel_angles = - deltaRij() * skewSymmetric(correctedAcc) * Jr_theta_i * deltaT;
  // analytic expression corresponding to the following numerical derivative
  // Matrix H_vel_angles = numericalDerivative11<LieVector, LieVector>(boost::bind(&PreIntegrateIMUObservations_delta_vel, correctedOmega, correctedAcc, deltaT, _1, deltaVij), theta_i);
  Matrix3 H_vel_biasacc = - deltaRij() * deltaT;

  Matrix3 H_angles_pos   = Z_3x3;
  Matrix3 H_angles_vel    = Z_3x3;
  Matrix3 H_angles_angles = Jrinv_theta_j * Rincr.inverse().matrix() * Jr_theta_i;
  Matrix3 H_angles_biasomega =- Jrinv_theta_j * Jr_theta_incr * deltaT;
  // analytic expression corresponding to the following numerical derivative
  // Matrix H_angles_angles = numericalDerivative11<Vector3, Vector3>(boost::bind(&PreIntegrateIMUObservations_delta_angles, correctedOmega, deltaT, _1), thetaij);

  // overall Jacobian wrt preintegrated measurements (df/dx)
  Matrix F(15,15);
  F << H_pos_pos,    H_pos_vel,     H_pos_angles,          Z_3x3,                     Z_3x3,
      H_vel_pos,     H_vel_vel,     H_vel_angles,      H_vel_biasacc,              Z_3x3,
      H_angles_pos,  H_angles_vel,  H_angles_angles,   Z_3x3,                         H_angles_biasomega,
      Z_3x3,         Z_3x3,         Z_3x3,             I_3x3,                         Z_3x3,
      Z_3x3,         Z_3x3,         Z_3x3,             Z_3x3,                         I_3x3;

  // first order uncertainty propagation
  // Optimized matrix multiplication   (1/deltaT) * G * measurementCovariance * G.transpose()

  Matrix G_measCov_Gt = Matrix::Zero(15,15);
  // BLOCK DIAGONAL TERMS
  G_measCov_Gt.block<3,3>(0,0) = deltaT * measurementCovariance_.block<3,3>(0,0);

  G_measCov_Gt.block<3,3>(3,3) = (1/deltaT) * (H_vel_biasacc)  *
      (measurementCovariance_.block<3,3>(3,3)  +  measurementCovariance_.block<3,3>(15,15) ) *
      (H_vel_biasacc.transpose());

  G_measCov_Gt.block<3,3>(6,6) = (1/deltaT) *  (H_angles_biasomega) *
      (measurementCovariance_.block<3,3>(6,6)  +  measurementCovariance_.block<3,3>(18,18) ) *
      (H_angles_biasomega.transpose());

  G_measCov_Gt.block<3,3>(9,9) = deltaT * measurementCovariance_.block<3,3>(9,9);

  G_measCov_Gt.block<3,3>(12,12) = deltaT * measurementCovariance_.block<3,3>(12,12);

  // NEW OFF BLOCK DIAGONAL TERMS
  Matrix3 block23 = H_vel_biasacc * measurementCovariance_.block<3,3>(18,15) *  H_angles_biasomega.transpose();
  G_measCov_Gt.block<3,3>(3,6) = block23;
  G_measCov_Gt.block<3,3>(6,3) = block23.transpose();

  preintMeasCov_ = F * preintMeasCov_ * F.transpose() + G_measCov_Gt;
}

//------------------------------------------------------------------------------
// CombinedImuFactor methods
//------------------------------------------------------------------------------
CombinedImuFactor::CombinedImuFactor() :
    ImuFactorBase(), preintegratedMeasurements_(imuBias::ConstantBias(), Z_3x3, Z_3x3, Z_3x3, Z_3x3, Z_3x3, Matrix::Zero(6,6)) {}

//------------------------------------------------------------------------------
CombinedImuFactor::CombinedImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias_i, Key bias_j,
    const CombinedPreintegratedMeasurements& preintegratedMeasurements,
    const Vector3& gravity, const Vector3& omegaCoriolis,
    boost::optional<const Pose3&> body_P_sensor, const bool use2ndOrderCoriolis) :
          Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.preintMeasCov_), pose_i, vel_i, pose_j, vel_j, bias_i, bias_j),
          ImuFactorBase(gravity, omegaCoriolis, body_P_sensor, use2ndOrderCoriolis),
          preintegratedMeasurements_(preintegratedMeasurements) {}

//------------------------------------------------------------------------------
gtsam::NonlinearFactor::shared_ptr CombinedImuFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
void CombinedImuFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "CombinedImuFactor("
      << keyFormatter(this->key1()) << ","
      << keyFormatter(this->key2()) << ","
      << keyFormatter(this->key3()) << ","
      << keyFormatter(this->key4()) << ","
      << keyFormatter(this->key5()) << ","
      << keyFormatter(this->key6()) << ")\n";
  ImuFactorBase::print("");
  preintegratedMeasurements_.print("  preintegrated measurements:");
  this->noiseModel_->print("  noise model: ");
}

//------------------------------------------------------------------------------
bool CombinedImuFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This *e =  dynamic_cast<const This*> (&expected);
  return e != NULL && Base::equals(*e, tol)
  && preintegratedMeasurements_.equals(e->preintegratedMeasurements_, tol)
  && ImuFactorBase::equals(*e, tol);
}

//------------------------------------------------------------------------------
Vector CombinedImuFactor::evaluateError(const Pose3& pose_i, const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, const imuBias::ConstantBias& bias_j,
    boost::optional<Matrix&> H1, boost::optional<Matrix&> H2,
    boost::optional<Matrix&> H3, boost::optional<Matrix&> H4,
    boost::optional<Matrix&> H5, boost::optional<Matrix&> H6) const {

  // if we need the jacobians
  if(H1 || H2 || H3 || H4 || H5 || H6){
    Matrix H1_pvR, H2_pvR, H3_pvR, H4_pvR, H5_pvR, Hbias_i, Hbias_j; // pvR = mnemonic: position (p), velocity (v), rotation (R)

    // error wrt preintegrated measurements
    Vector r_pvR(9); r_pvR << ImuFactorBase::computeErrorAndJacobians(preintegratedMeasurements_, pose_i, vel_i, pose_j, vel_j, bias_i,
        H1_pvR, H2_pvR, H3_pvR, H4_pvR, H5_pvR);

    // error wrt bias evolution model (random walk)
    Vector6 fbias = bias_j.between(bias_i, Hbias_j, Hbias_i).vector(); // [bias_j.acc - bias_i.acc; bias_j.gyr - bias_i.gyr]

    if(H1) {
      H1->resize(15,6);
      H1->block<9,6>(0,0) = H1_pvR;
      // adding: [dBiasAcc/dPi ; dBiasOmega/dPi]
      H1->block<6,6>(0,9) = Matrix::Zero(6,6);
    }
    if(H2) {
      H2->resize(15,3);
      H2->block<9,3>(0,0) = H2_pvR;
      // adding: [dBiasAcc/dVi ; dBiasOmega/dVi]
      H2->block<6,3>(0,9) = Matrix::Zero(6,3);
    }
    if(H3) {
      H3->resize(15,6);
      H3->block<9,6>(0,0) = H3_pvR;
      // adding: [dBiasAcc/dPj ; dBiasOmega/dPj]
      H3->block<6,6>(0,9) = Matrix::Zero(6,6);
    }
    if(H4) {
      H4->resize(15,3);
      H4->block<9,3>(0,0) = H4_pvR;
      // adding: [dBiasAcc/dVi ; dBiasOmega/dVi]
      H4->block<6,3>(0,9) = Matrix::Zero(6,3);
    }
    if(H5) {
      H5->resize(15,6);
      H5->block<9,6>(0,0) = H5_pvR;
      // adding: [dBiasAcc/dBias_i ; dBiasOmega/dBias_i]
      H5->block<6,6>(0,9) = Hbias_i;
    }
    if(H6) {
      H6->resize(15,6);
      H6->block<9,6>(0,0) = Matrix::Zero(6,6);
      // adding: [dBiasAcc/dBias_j ; dBiasOmega/dBias_j]
      H6->block<6,6>(0,9) = Hbias_j;
    }
    Vector r(15); r << r_pvR, fbias; // vector of size 15
    return r;
  }
  // else, only compute the error vector:
  // error wrt preintegrated measurements
  Vector r_pvR(9); r_pvR << ImuFactorBase::computeErrorAndJacobians(preintegratedMeasurements_, pose_i, vel_i, pose_j, vel_j, bias_i,
      boost::none, boost::none, boost::none, boost::none, boost::none);
  // error wrt bias evolution model (random walk)
  Vector6 fbias = bias_j.between(bias_i).vector(); // [bias_j.acc - bias_i.acc; bias_j.gyr - bias_i.gyr]
  // overall error
  Vector r(15); r << r_pvR, fbias; // vector of size 15
  return r;
}

} /// namespace gtsam
