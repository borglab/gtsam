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
// Inner class PreintegratedImuMeasurements
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
  // Update preintegrated measurements (also get Jacobian)
  Matrix9 A;  // overall Jacobian wrt preintegrated measurements (df/dx)
  Matrix93 B, C;
  updatedPreintegrated(measuredAcc, measuredOmega, dt, &A, &B, &C);

  // first order covariance propagation:
  // as in [2] we consider a first order propagation that can be seen as a
  // prediction phase in EKF

  // propagate uncertainty
  // TODO(frank): use noiseModel routine so we can have arbitrary noise models.
  const Matrix3& aCov = p().accelerometerCovariance;
  const Matrix3& wCov = p().gyroscopeCovariance;
  const Matrix3& iCov = p().integrationCovariance;

  // (1/dt) allows to pass from continuous time noise to discrete time noise
  preintMeasCov_ = A * preintMeasCov_ * A.transpose();
  preintMeasCov_.noalias() += B * (aCov / dt) * B.transpose();
  preintMeasCov_.noalias() += C * (wCov / dt) * C.transpose();

  // NOTE(frank): (Gi*dt)*(C/dt)*(Gi'*dt)
  static const Matrix93 Gi = (Matrix93() << Z_3x3, I_3x3, Z_3x3).finished();
  preintMeasCov_.noalias() += Gi * (iCov * dt) * Gi.transpose();
}

//------------------------------------------------------------------------------
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
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

void PreintegratedImuMeasurements::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double deltaT,
    boost::optional<Pose3> body_P_sensor) {
  // modify parameters to accommodate deprecated method:-(
  p_->body_P_sensor = body_P_sensor;
  integrateMeasurement(measuredAcc, measuredOmega, deltaT);
}
#endif

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
std::ostream& operator<<(std::ostream& os, const ImuFactor& f) {
  os << "  preintegrated measurements:\n" << f._PIM_;
  ;
  // Print standard deviations on covariance only
  os << "  noise model sigmas: " << f.noiseModel_->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
void ImuFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "ImuFactor(" << keyFormatter(this->key1()) << ","
      << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << ","
      << keyFormatter(this->key4()) << "," << keyFormatter(this->key5())
      << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
bool ImuFactor::equals(const NonlinearFactor& other, double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  const bool base = Base::equals(*e, tol);
  const bool pim = _PIM_.equals(e->_PIM_, tol);
  return e != nullptr && base && pim;
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
PreintegratedImuMeasurements ImuFactor::Merge(
    const PreintegratedImuMeasurements& pim01,
    const PreintegratedImuMeasurements& pim12) {
  if(!pim01.matchesParamsWith(pim12))
    throw std::domain_error("Cannot merge PreintegratedImuMeasurements with different params");

  if(pim01.params()->body_P_sensor)
    throw std::domain_error("Cannot merge PreintegratedImuMeasurements with sensor pose yet");

  // the bias for the merged factor will be the bias from 01
  PreintegratedImuMeasurements pim02(pim01.params(), pim01.biasHat());

  const double& t01 = pim01.deltaTij();
  const double& t12 = pim12.deltaTij();
  pim02.deltaTij_ = t01 + t12;

  const Rot3 R01 = pim01.deltaRij(), R12 = pim12.deltaRij();
  pim02.preintegrated_ << Rot3::Logmap(R01 * R12),
      pim01.deltaPij() + pim01.deltaVij() * t12 + R01 * pim12.deltaPij(),
      pim01.deltaVij() + R01 * pim12.deltaVij();

  return pim02;
}
//------------------------------------------------------------------------------
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
ImuFactor::ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
    const PreintegratedImuMeasurements& pim, const Vector3& n_gravity,
    const Vector3& omegaCoriolis, const boost::optional<Pose3>& body_P_sensor,
    const bool use2ndOrderCoriolis) :
Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose_i, vel_i,
    pose_j, vel_j, bias), _PIM_(pim) {
  boost::shared_ptr<PreintegratedImuMeasurements::Params> p = boost::make_shared<
  PreintegratedImuMeasurements::Params>(pim.p());
  p->n_gravity = n_gravity;
  p->omegaCoriolis = omegaCoriolis;
  p->body_P_sensor = body_P_sensor;
  p->use2ndOrderCoriolis = use2ndOrderCoriolis;
  _PIM_.p_ = p;
}

void ImuFactor::Predict(const Pose3& pose_i, const Vector3& vel_i,
    Pose3& pose_j, Vector3& vel_j, const imuBias::ConstantBias& bias_i,
    PreintegratedImuMeasurements& pim, const Vector3& n_gravity,
    const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis) {
  // use deprecated predict
  PoseVelocityBias pvb = pim.predict(pose_i, vel_i, bias_i, n_gravity,
      omegaCoriolis, use2ndOrderCoriolis);
  pose_j = pvb.pose;
  vel_j = pvb.velocity;
}
#endif
//------------------------------------------------------------------------------

}
 // namespace gtsam
