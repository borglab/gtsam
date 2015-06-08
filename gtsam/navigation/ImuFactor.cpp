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
    const Matrix3& measuredOmegaCovariance,
    const Matrix3& integrationErrorCovariance,
    const bool use2ndOrderIntegration) :
    PreintegrationBase(bias, measuredAccCovariance, measuredOmegaCovariance,
        integrationErrorCovariance, use2ndOrderIntegration) {
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void ImuFactor::PreintegratedMeasurements::print(const string& s) const {
  PreintegrationBase::print(s);
}

//------------------------------------------------------------------------------
bool ImuFactor::PreintegratedMeasurements::equals(
    const PreintegratedMeasurements& expected, double tol) const {
  return equal_with_abs_tol(preintMeasCov_, expected.preintMeasCov_, tol)
      && PreintegrationBase::equals(expected, tol);
}

//------------------------------------------------------------------------------
void ImuFactor::PreintegratedMeasurements::resetIntegration() {
  PreintegrationBase::resetIntegration();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void ImuFactor::PreintegratedMeasurements::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double deltaT,
    boost::optional<const Pose3&> body_P_sensor, OptionalJacobian<9, 9> F_test,
    OptionalJacobian<9, 9> G_test) {

  Vector3 correctedAcc, correctedOmega;
  correctMeasurementsByBiasAndSensorPose(measuredAcc, measuredOmega,
      correctedAcc, correctedOmega, body_P_sensor);

  const Vector3 integratedOmega = correctedOmega * deltaT; // rotation vector describing rotation increment computed from the current rotation rate measurement
  Matrix3 D_Rincr_integratedOmega; // Right jacobian computed at theta_incr
  const Rot3 Rincr = Rot3::Expmap(integratedOmega, D_Rincr_integratedOmega); // rotation increment computed from the current rotation rate measurement

  // Update Jacobians
  updatePreintegratedJacobians(correctedAcc, D_Rincr_integratedOmega, Rincr, deltaT);

  // Update preintegrated measurements (also get Jacobian)
  const Matrix3 R_i = deltaRij(); // store this, which is useful to compute G_test
  Matrix9 F; // overall Jacobian wrt preintegrated measurements (df/dx)
  updatePreintegratedMeasurements(correctedAcc, Rincr, deltaT, F);

  // first order covariance propagation:
  // as in [2] we consider a first order propagation that can be seen as a prediction phase in an EKF framework
  /* ----------------------------------------------------------------------------------------------------------------------- */
  // preintMeasCov = F * preintMeasCov * F.transpose() + G * (1/deltaT) * measurementCovariance * G.transpose();
  // NOTE 1: (1/deltaT) allows to pass from continuous time noise to discrete time noise
  // measurementCovariance_discrete = measurementCovariance_contTime * (1/deltaT)
  // NOTE 2: the computation of G * (1/deltaT) * measurementCovariance * G.transpose() is done block-wise,
  // as G and measurementCovariance are block-diagonal matrices
  preintMeasCov_ = F * preintMeasCov_ * F.transpose();
  preintMeasCov_.block<3, 3>(0, 0) += integrationCovariance() * deltaT;
  preintMeasCov_.block<3, 3>(3, 3) += R_i * accelerometerCovariance()
      * R_i.transpose() * deltaT;
  preintMeasCov_.block<3, 3>(6, 6) += D_Rincr_integratedOmega
      * gyroscopeCovariance() * D_Rincr_integratedOmega.transpose() * deltaT;

  Matrix3 F_pos_noiseacc;
  if (use2ndOrderIntegration()) {
    F_pos_noiseacc = 0.5 * R_i * deltaT * deltaT;
    preintMeasCov_.block<3, 3>(0, 0) += (1 / deltaT) * F_pos_noiseacc
        * accelerometerCovariance() * F_pos_noiseacc.transpose();
    Matrix3 temp = F_pos_noiseacc * accelerometerCovariance() * R_i.transpose(); // already includes 1/deltaT
    preintMeasCov_.block<3, 3>(0, 3) += temp;
    preintMeasCov_.block<3, 3>(3, 0) += temp.transpose();
  }

  // F_test and G_test are given as output for testing purposes and are not needed by the factor
  if (F_test) { // This in only for testing
    (*F_test) << F;
  }
  if (G_test) { // This in only for testing & documentation, while the actual computation is done block-wise
    if (!use2ndOrderIntegration())
      F_pos_noiseacc = Z_3x3;

    //           intNoise          accNoise           omegaNoise
    (*G_test) << I_3x3 * deltaT, F_pos_noiseacc, Z_3x3, // pos
    Z_3x3, R_i * deltaT, Z_3x3, // vel
    Z_3x3, Z_3x3, D_Rincr_integratedOmega * deltaT; // angle
  }
}

//------------------------------------------------------------------------------
// ImuFactor methods
//------------------------------------------------------------------------------
ImuFactor::ImuFactor() :
    ImuFactorBase(), _PIM_(imuBias::ConstantBias(), Z_3x3, Z_3x3, Z_3x3) {
}

//------------------------------------------------------------------------------
ImuFactor::ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
    const PreintegratedMeasurements& preintegratedMeasurements,
    const Vector3& gravity, const Vector3& omegaCoriolis,
    boost::optional<const Pose3&> body_P_sensor, const bool use2ndOrderCoriolis) :
    Base(
        noiseModel::Gaussian::Covariance(
            preintegratedMeasurements.preintMeasCov_), pose_i, vel_i, pose_j,
        vel_j, bias), ImuFactorBase(gravity, omegaCoriolis, body_P_sensor,
        use2ndOrderCoriolis), _PIM_(preintegratedMeasurements) {
}

//------------------------------------------------------------------------------
gtsam::NonlinearFactor::shared_ptr ImuFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
void ImuFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "ImuFactor(" << keyFormatter(this->key1()) << ","
      << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << ","
      << keyFormatter(this->key4()) << "," << keyFormatter(this->key5())
      << ")\n";
  ImuFactorBase::print("");
  _PIM_.print("  preintegrated measurements:");
  // Print standard deviations on covariance only
  cout << "  noise model sigmas: " << this->noiseModel_->sigmas().transpose() << endl;
}

//------------------------------------------------------------------------------
bool ImuFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This *e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && _PIM_.equals(e->_PIM_, tol)
      && ImuFactorBase::equals(*e, tol);
}

//------------------------------------------------------------------------------
Vector ImuFactor::evaluateError(const Pose3& pose_i, const Vector3& vel_i,
    const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2, boost::optional<Matrix&> H3,
    boost::optional<Matrix&> H4, boost::optional<Matrix&> H5) const {

  return _PIM_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j, bias_i,
      gravity_, omegaCoriolis_, use2ndOrderCoriolis_, H1, H2, H3, H4, H5);
}

} /// namespace gtsam
