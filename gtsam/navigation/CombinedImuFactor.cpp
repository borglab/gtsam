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
    const Matrix3& measuredOmegaCovariance,
    const Matrix3& integrationErrorCovariance, const Matrix3& biasAccCovariance,
    const Matrix3& biasOmegaCovariance, const Matrix& biasAccOmegaInit,
    const bool use2ndOrderIntegration) :
    PreintegrationBase(bias, measuredAccCovariance, measuredOmegaCovariance,
        integrationErrorCovariance, use2ndOrderIntegration), biasAccCovariance_(
        biasAccCovariance), biasOmegaCovariance_(biasOmegaCovariance), biasAccOmegaInit_(
        biasAccOmegaInit) {
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void CombinedImuFactor::CombinedPreintegratedMeasurements::print(
    const string& s) const {
  PreintegrationBase::print(s);
  cout << "  biasAccCovariance [ " << biasAccCovariance_ << " ]" << endl;
  cout << "  biasOmegaCovariance [ " << biasOmegaCovariance_ << " ]" << endl;
  cout << "  biasAccOmegaInit [ " << biasAccOmegaInit_ << " ]" << endl;
  cout << "  preintMeasCov [ " << preintMeasCov_ << " ]" << endl;
}

//------------------------------------------------------------------------------
bool CombinedImuFactor::CombinedPreintegratedMeasurements::equals(
    const CombinedPreintegratedMeasurements& expected, double tol) const {
  return equal_with_abs_tol(biasAccCovariance_, expected.biasAccCovariance_,
      tol)
      && equal_with_abs_tol(biasOmegaCovariance_, expected.biasOmegaCovariance_,
          tol)
      && equal_with_abs_tol(biasAccOmegaInit_, expected.biasAccOmegaInit_, tol)
      && equal_with_abs_tol(preintMeasCov_, expected.preintMeasCov_, tol)
      && PreintegrationBase::equals(expected, tol);
}

//------------------------------------------------------------------------------
void CombinedImuFactor::CombinedPreintegratedMeasurements::resetIntegration() {
  PreintegrationBase::resetIntegration();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void CombinedImuFactor::CombinedPreintegratedMeasurements::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double deltaT,
    boost::optional<const Pose3&> body_P_sensor,
    boost::optional<Matrix&> F_test, boost::optional<Matrix&> G_test) {

  // NOTE: order is important here because each update uses old values, e.g., velocity and position updates are based on previous rotation estimate.
  // (i.e., we have to update jacobians and covariances before updating preintegrated measurements).

  Vector3 correctedAcc, correctedOmega;
  correctMeasurementsByBiasAndSensorPose(measuredAcc, measuredOmega,
      correctedAcc, correctedOmega, body_P_sensor);

  const Vector3 integratedOmega = correctedOmega * deltaT; // rotation vector describing rotation increment computed from the current rotation rate measurement
  Matrix3 D_Rincr_integratedOmega; // Right jacobian computed at theta_incr
  const Rot3 Rincr = Rot3::Expmap(integratedOmega, D_Rincr_integratedOmega); // rotation increment computed from the current rotation rate measurement

  // Update Jacobians
  /* ----------------------------------------------------------------------------------------------------------------------- */
  updatePreintegratedJacobians(correctedAcc, D_Rincr_integratedOmega, Rincr,
      deltaT);

  // Update preintegrated measurements covariance: as in [2] we consider a first order propagation that
  // can be seen as a prediction phase in an EKF framework. In this implementation, contrarily to [2] we
  // consider the uncertainty of the bias selection and we keep correlation between biases and preintegrated measurements
  /* ----------------------------------------------------------------------------------------------------------------------- */
  const Matrix3 R_i = deltaRij(); // store this
  // Update preintegrated measurements. TODO Frank moved from end of this function !!!
  Matrix9 F_9x9;
  updatePreintegratedMeasurements(correctedAcc, Rincr, deltaT, F_9x9);

  // Single Jacobians to propagate covariance
  Matrix3 H_vel_biasacc = -R_i * deltaT;
  Matrix3 H_angles_biasomega = -D_Rincr_integratedOmega * deltaT;

  // overall Jacobian wrt preintegrated measurements (df/dx)
  Matrix F(15, 15);
  // for documentation:
  //  F << I_3x3,    I_3x3 * deltaT,   Z_3x3,             Z_3x3,            Z_3x3,
  //       Z_3x3,    I_3x3,            H_vel_angles,      H_vel_biasacc,    Z_3x3,
  //       Z_3x3,    Z_3x3,            H_angles_angles,   Z_3x3,            H_angles_biasomega,
  //       Z_3x3,    Z_3x3,            Z_3x3,             I_3x3,            Z_3x3,
  //       Z_3x3,    Z_3x3,            Z_3x3,             Z_3x3,            I_3x3;
  F.setZero();
  F.block<9, 9>(0, 0) = F_9x9;
  F.block<6, 6>(9, 9) = I_6x6;
  F.block<3, 3>(3, 9) = H_vel_biasacc;
  F.block<3, 3>(6, 12) = H_angles_biasomega;

  // first order uncertainty propagation
  // Optimized matrix multiplication   (1/deltaT) * G * measurementCovariance * G.transpose()
  Matrix G_measCov_Gt = Matrix::Zero(15, 15);

//   BLOCK DIAGONAL TERMS
  G_measCov_Gt.block<3, 3>(0, 0) = deltaT * integrationCovariance();
  G_measCov_Gt.block<3, 3>(3, 3) = (1 / deltaT) * (H_vel_biasacc)
      * (accelerometerCovariance() + biasAccOmegaInit_.block<3, 3>(0, 0))
      * (H_vel_biasacc.transpose());
  G_measCov_Gt.block<3, 3>(6, 6) = (1 / deltaT) * (H_angles_biasomega)
      * (gyroscopeCovariance() + biasAccOmegaInit_.block<3, 3>(3, 3))
      * (H_angles_biasomega.transpose());
  G_measCov_Gt.block<3, 3>(9, 9) = (1 / deltaT) * biasAccCovariance_;
  G_measCov_Gt.block<3, 3>(12, 12) = (1 / deltaT) * biasOmegaCovariance_;
  // OFF BLOCK DIAGONAL TERMS
  Matrix3 block23 = H_vel_biasacc * biasAccOmegaInit_.block<3, 3>(3, 0)
      * H_angles_biasomega.transpose();
  G_measCov_Gt.block<3, 3>(3, 6) = block23;
  G_measCov_Gt.block<3, 3>(6, 3) = block23.transpose();
  preintMeasCov_ = F * preintMeasCov_ * F.transpose() + G_measCov_Gt;

  // F_test and G_test are used for testing purposes and are not needed by the factor
  if (F_test) {
    F_test->resize(15, 15);
    (*F_test) << F;
  }
  if (G_test) {
    G_test->resize(15, 21);
    // This is for testing & documentation
    ///< measurementCovariance_ : cov[integrationError measuredAcc measuredOmega biasAccRandomWalk biasOmegaRandomWalk biasAccInit biasOmegaInit] in R^(21 x 21)
    (*G_test) << //
        I_3x3 * deltaT, Z_3x3, Z_3x3, Z_3x3, Z_3x3, Z_3x3, Z_3x3, //
    Z_3x3, -H_vel_biasacc, Z_3x3, Z_3x3, Z_3x3, H_vel_biasacc, Z_3x3, //
    Z_3x3, Z_3x3, -H_angles_biasomega, Z_3x3, Z_3x3, Z_3x3, H_angles_biasomega, //
    Z_3x3, Z_3x3, Z_3x3, I_3x3, Z_3x3, Z_3x3, Z_3x3, //
    Z_3x3, Z_3x3, Z_3x3, Z_3x3, I_3x3, Z_3x3, Z_3x3;
  }
}

//------------------------------------------------------------------------------
// CombinedImuFactor methods
//------------------------------------------------------------------------------
CombinedImuFactor::CombinedImuFactor() :
    ImuFactorBase(), _PIM_(imuBias::ConstantBias(), Z_3x3, Z_3x3, Z_3x3, Z_3x3,
        Z_3x3, Z_6x6) {
}

//------------------------------------------------------------------------------
CombinedImuFactor::CombinedImuFactor(Key pose_i, Key vel_i, Key pose_j,
    Key vel_j, Key bias_i, Key bias_j,
    const CombinedPreintegratedMeasurements& preintegratedMeasurements,
    const Vector3& gravity, const Vector3& omegaCoriolis,
    boost::optional<const Pose3&> body_P_sensor, const bool use2ndOrderCoriolis) :
    Base(
        noiseModel::Gaussian::Covariance(
            preintegratedMeasurements.preintMeasCov_), pose_i, vel_i, pose_j,
        vel_j, bias_i, bias_j), ImuFactorBase(gravity, omegaCoriolis,
        body_P_sensor, use2ndOrderCoriolis), _PIM_(preintegratedMeasurements) {
}

//------------------------------------------------------------------------------
gtsam::NonlinearFactor::shared_ptr CombinedImuFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
void CombinedImuFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << "CombinedImuFactor(" << keyFormatter(this->key1()) << ","
      << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << ","
      << keyFormatter(this->key4()) << "," << keyFormatter(this->key5()) << ","
      << keyFormatter(this->key6()) << ")\n";
  ImuFactorBase::print("");
  _PIM_.print("  preintegrated measurements:");
  this->noiseModel_->print("  noise model: ");
}

//------------------------------------------------------------------------------
bool CombinedImuFactor::equals(const NonlinearFactor& expected,
    double tol) const {
  const This *e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && _PIM_.equals(e->_PIM_, tol)
      && ImuFactorBase::equals(*e, tol);
}

//------------------------------------------------------------------------------
Vector CombinedImuFactor::evaluateError(const Pose3& pose_i,
    const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, const imuBias::ConstantBias& bias_j,
    boost::optional<Matrix&> H1, boost::optional<Matrix&> H2,
    boost::optional<Matrix&> H3, boost::optional<Matrix&> H4,
    boost::optional<Matrix&> H5, boost::optional<Matrix&> H6) const {

  // error wrt bias evolution model (random walk)
  Matrix6 Hbias_i, Hbias_j;
  Vector6 fbias = traits<imuBias::ConstantBias>::Between(bias_j, bias_i,
      H6 ? &Hbias_j : 0, H5 ? &Hbias_i : 0).vector();

  Matrix96 D_r_pose_i, D_r_pose_j, D_r_bias_i;
  Matrix93 D_r_vel_i, D_r_vel_j;

  // error wrt preintegrated measurements
  Vector9 r_pvR = _PIM_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j,
      bias_i, gravity_, omegaCoriolis_, use2ndOrderCoriolis_,
      H1 ? &D_r_pose_i : 0, H2 ? &D_r_vel_i : 0, H3 ? &D_r_pose_j : 0,
      H4 ? &D_r_vel_j : 0, H5 ? &D_r_bias_i : 0);

  // if we need the jacobians
  if (H1) {
    H1->resize(15, 6);
    H1->block<9, 6>(0, 0) = D_r_pose_i;
    // adding: [dBiasAcc/dPi ; dBiasOmega/dPi]
    H1->block<6, 6>(9, 0) = Z_6x6;
  }
  if (H2) {
    H2->resize(15, 3);
    H2->block<9, 3>(0, 0) = D_r_vel_i;
    // adding: [dBiasAcc/dVi ; dBiasOmega/dVi]
    H2->block<6, 3>(9, 0) = Matrix::Zero(6, 3);
  }
  if (H3) {
    H3->resize(15, 6);
    H3->block<9, 6>(0, 0) = D_r_pose_j;
    // adding: [dBiasAcc/dPj ; dBiasOmega/dPj]
    H3->block<6, 6>(9, 0) = Z_6x6;
  }
  if (H4) {
    H4->resize(15, 3);
    H4->block<9, 3>(0, 0) = D_r_vel_j;
    // adding: [dBiasAcc/dVi ; dBiasOmega/dVi]
    H4->block<6, 3>(9, 0) = Matrix::Zero(6, 3);
  }
  if (H5) {
    H5->resize(15, 6);
    H5->block<9, 6>(0, 0) = D_r_bias_i;
    // adding: [dBiasAcc/dBias_i ; dBiasOmega/dBias_i]
    H5->block<6, 6>(9, 0) = Hbias_i;
  }
  if (H6) {
    H6->resize(15, 6);
    H6->block<9, 6>(0, 0) = Matrix::Zero(9, 6);
    // adding: [dBiasAcc/dBias_j ; dBiasOmega/dBias_j]
    H6->block<6, 6>(9, 0) = Hbias_j;
  }

  // overall error
  Vector r(15);
  r << r_pvR, fbias; // vector of size 15
  return r;
}

} /// namespace gtsam
