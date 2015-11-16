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
void PreintegratedImuMeasurements::print(const string& s) const {
  PreintegrationBase::print(s);
  cout << "    preintMeasCov \n[" << preintMeasCov_ << "]" << endl;
}

//------------------------------------------------------------------------------
bool PreintegratedImuMeasurements::equals(
    const PreintegratedImuMeasurements& other, double tol) const {
  return PreintegrationBase::equals(other, tol)
      && equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol);
}

//------------------------------------------------------------------------------
void PreintegratedImuMeasurements::resetIntegration() {
  PreintegrationBase::resetIntegration();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void PreintegratedImuMeasurements::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {

  static const Matrix93 Gi = (Matrix93() << Z_3x3, I_3x3, Z_3x3).finished();

  // Update preintegrated measurements (also get Jacobian)
  Matrix9 F; // overall Jacobian wrt preintegrated measurements (df/dx)
  Matrix93 G1, G2;
  Matrix3 D_incrR_integratedOmega;
  PreintegrationBase::update(measuredAcc, measuredOmega, dt,
      &D_incrR_integratedOmega, &F, &G1, &G2);

  // first order covariance propagation:
  // as in [2] we consider a first order propagation that can be seen as a prediction phase in EKF
  /* --------------------------------------------------------------------------------------------*/
  // preintMeasCov = F * preintMeasCov * F.transpose() + G * (1/deltaT) * measurementCovariance * G'
  // NOTE 1: (1/deltaT) allows to pass from continuous time noise to discrete time noise
  // measurementCovariance_discrete = measurementCovariance_contTime * (1/deltaT)
#ifdef OLD_JACOBIAN_CALCULATION
  Matrix9 G;
  G << G1, Gi, G2;
  Matrix9 Cov;
  Cov << p().accelerometerCovariance / dt, Z_3x3, Z_3x3,
      Z_3x3, p().integrationCovariance * dt, Z_3x3,
      Z_3x3, Z_3x3, p().gyroscopeCovariance / dt;
  preintMeasCov_ = F * preintMeasCov_ * F.transpose() + G * Cov * G.transpose();
#else
  preintMeasCov_ = F * preintMeasCov_ * F.transpose()
      + Gi * (p().integrationCovariance * dt) * Gi.transpose() // NOTE(frank): (Gi*dt)*(C/dt)*(Gi'*dt)
      + G1 * (p().accelerometerCovariance / dt) * G1.transpose()
      + G2 * (p().gyroscopeCovariance / dt) * G2.transpose();
#endif
}

//------------------------------------------------------------------------------
PreintegratedImuMeasurements::PreintegratedImuMeasurements(
    const imuBias::ConstantBias& biasHat, const Matrix3& measuredAccCovariance,
    const Matrix3& measuredOmegaCovariance,
    const Matrix3& integrationErrorCovariance, bool use2ndOrderIntegration) {
  if (!use2ndOrderIntegration)
    throw("PreintegratedImuMeasurements no longer supports first-order integration: it incorrectly compensated for gravity");
  biasHat_ = biasHat;
  boost::shared_ptr<Params> p = Params::MakeSharedD();
  p->gyroscopeCovariance = measuredOmegaCovariance;
  p->accelerometerCovariance = measuredAccCovariance;
  p->integrationCovariance = integrationErrorCovariance;
  p_ = p;
  resetIntegration();
}

//------------------------------------------------------------------------------
void PreintegratedImuMeasurements::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double deltaT,
    boost::optional<Pose3> body_P_sensor) {
  // modify parameters to accommodate deprecated method:-(
  p_->body_P_sensor = body_P_sensor;
  integrateMeasurement(measuredAcc, measuredOmega, deltaT);
}

//------------------------------------------------------------------------------
// ImuFactor methods
//------------------------------------------------------------------------------
ImuFactor::ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
    const PreintegratedImuMeasurements& pim) :
    Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose_i, vel_i,
        pose_j, vel_j, bias), _PIM_(pim) {
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
  Base::print("");
  _PIM_.print("  preintegrated measurements:");
  // Print standard deviations on covariance only
  cout << "  noise model sigmas: " << this->noiseModel_->sigmas().transpose()
      << endl;
}

//------------------------------------------------------------------------------
bool ImuFactor::equals(const NonlinearFactor& other, double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  return e != NULL && Base::equals(*e, tol) && _PIM_.equals(e->_PIM_, tol)
      && Base::equals(*e, tol);
}

//------------------------------------------------------------------------------
Vector ImuFactor::evaluateError(const Pose3& pose_i, const Vector3& vel_i,
    const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2, boost::optional<Matrix&> H3,
    boost::optional<Matrix&> H4, boost::optional<Matrix&> H5) const {
  return _PIM_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j, bias_i,
      H1, H2, H3, H4, H5);
}

//------------------------------------------------------------------------------
ImuFactor::ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
    const PreintegratedMeasurements& pim, const Vector3& n_gravity,
    const Vector3& omegaCoriolis, const boost::optional<Pose3>& body_P_sensor,
    const bool use2ndOrderCoriolis) :
    Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose_i, vel_i,
        pose_j, vel_j, bias), _PIM_(pim) {
  boost::shared_ptr<PreintegratedMeasurements::Params> p = boost::make_shared<
      PreintegratedMeasurements::Params>(pim.p());
  p->n_gravity = n_gravity;
  p->omegaCoriolis = omegaCoriolis;
  p->body_P_sensor = body_P_sensor;
  p->use2ndOrderCoriolis = use2ndOrderCoriolis;
  _PIM_.p_ = p;
}

//------------------------------------------------------------------------------
void ImuFactor::Predict(const Pose3& pose_i, const Vector3& vel_i,
    Pose3& pose_j, Vector3& vel_j, const imuBias::ConstantBias& bias_i,
    PreintegratedMeasurements& pim, const Vector3& n_gravity,
    const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis) {
  // use deprecated predict
  PoseVelocityBias pvb = pim.predict(pose_i, vel_i, bias_i, n_gravity,
      omegaCoriolis, use2ndOrderCoriolis);
  pose_j = pvb.pose;
  vel_j = pvb.velocity;
}

} // namespace gtsam
