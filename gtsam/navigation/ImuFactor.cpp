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
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>

/* External or standard includes */
#include <ostream>
#include <cassert>

namespace gtsam {

using namespace std;

//------------------------------------------------------------------------------
// Inner class PreintegratedImuMeasurementsT
//------------------------------------------------------------------------------
template <class PreintegrationBackend>
void PreintegratedImuMeasurementsT<PreintegrationBackend>::print(const string& s) const {
  PreintegrationBackend::print(s);
  cout << "    preintMeasCov \n[" << preintMeasCov_ << "]" << endl;
}

//------------------------------------------------------------------------------
template <class PreintegrationBackend>
bool PreintegratedImuMeasurementsT<PreintegrationBackend>::equalsConcrete(
    const PreintegratedImuMeasurementsT<PreintegrationBackend>& other, double tol) const {
  return PreintegrationBackend::equals(other, tol)
      && equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol);
}

//------------------------------------------------------------------------------
template <class PreintegrationBackend>
bool PreintegratedImuMeasurementsT<PreintegrationBackend>::equals(
    const PreintegratedImuMeasurementsInterface& other_interface, double tol) const {
  const auto* other_as_this_pim_type =
      dynamic_cast<const PreintegratedImuMeasurementsT<PreintegrationBackend>*>(&other_interface);
  if (!other_as_this_pim_type) {
      return false;
  }
  return this->equalsConcrete(*other_as_this_pim_type, tol);
}

//------------------------------------------------------------------------------
template <class PreintegrationBackend>
void PreintegratedImuMeasurementsT<PreintegrationBackend>::resetIntegration() {
  PreintegrationBackend::resetIntegration();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
template <class PreintegrationBackend>
void PreintegratedImuMeasurementsT<PreintegrationBackend>::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  if (dt <= 0) {
    throw std::runtime_error(
        "PreintegratedImuMeasurements::integrateMeasurement: dt <=0");
  }

  // Update preintegrated measurements (also get Jacobian)
  Matrix9 A;  // overall Jacobian wrt preintegrated measurements (df/dx)
  Matrix93 B, C;  // Jacobian of state wrpt accel bias and omega bias respectively.
  PreintegrationBackend::update(measuredAcc, measuredOmega, dt, &A, &B, &C);

  // first order covariance propagation:
  // as in [2] we consider a first order propagation that can be seen as a
  // prediction phase in EKF

  // propagate uncertainty
  // TODO(frank): use noiseModel routine so we can have arbitrary noise models.
  const Matrix3& aCov = this->p().accelerometerCovariance;
  const Matrix3& wCov = this->p().gyroscopeCovariance;
  const Matrix3& iCov = this->p().integrationCovariance;

  // (1/dt) allows to pass from continuous time noise to discrete time noise
  // Update the uncertainty on the state (matrix A in [4]).
  preintMeasCov_ = A * preintMeasCov_ * A.transpose();
  // These 2 updates account for uncertainty on the IMU measurement (matrix B in [4]).
  preintMeasCov_.noalias() += B * (aCov / dt) * B.transpose();
  preintMeasCov_.noalias() += C * (wCov / dt) * C.transpose();

  // NOTE(frank): (Gi*dt)*(C/dt)*(Gi'*dt), with Gi << Z_3x3, I_3x3, Z_3x3 (9x3 matrix)
  preintMeasCov_.block<3, 3>(3, 3).noalias() += iCov * dt;
}

//------------------------------------------------------------------------------
template <class PreintegrationBackend>
void PreintegratedImuMeasurementsT<PreintegrationBackend>::integrateMeasurements(
    const Matrix& measuredAccs, const Matrix& measuredOmegas,
    const Matrix& dts) {
  assert(
      measuredAccs.rows() == 3 && measuredOmegas.rows() == 3 && dts.rows() == 1);
  assert(dts.cols() >= 1);
  assert(measuredAccs.cols() == dts.cols());
  assert(measuredOmegas.cols() == dts.cols());
  size_t n = static_cast<size_t>(dts.cols());
  for (size_t j = 0; j < n; j++) {
    integrateMeasurement(measuredAccs.col(j), measuredOmegas.col(j), dts(0, j));
  }
}

//------------------------------------------------------------------------------
// ImuFactor methods
//------------------------------------------------------------------------------
ImuFactor::ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
  const std::shared_ptr<const PreintegratedImuMeasurementsInterface>& pim) :
  Base(noiseModel::Gaussian::Covariance(pim->preintMeasCov()), pose_i, vel_i,
      pose_j, vel_j, bias), _PIM_(pim) {
}

//------------------------------------------------------------------------------
NonlinearFactor::shared_ptr ImuFactor::clone() const {
return std::static_pointer_cast<NonlinearFactor>(
    NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const ImuFactor& f) {
  f.preintegratedMeasurements()->print("preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel()->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
void ImuFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? s : s + "\n") << "ImuFactor(" << keyFormatter(this->template key<1>())
       << "," << keyFormatter(this->template key<2>()) << "," << keyFormatter(this->template key<3>())
       << "," << keyFormatter(this->template key<4>()) << "," << keyFormatter(this->template key<5>())
       << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
bool ImuFactor::equals(const NonlinearFactor& other, double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  const bool base = Base::equals(*e, tol);
  const bool pim = _PIM_->equals(*(e->_PIM_), tol);
  return e != nullptr && base && pim;
}

//------------------------------------------------------------------------------
Vector ImuFactor::evaluateError(const Pose3& pose_i, const Vector3& vel_i,
    const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, OptionalMatrixType H1,
    OptionalMatrixType H2, OptionalMatrixType H3,
    OptionalMatrixType H4, OptionalMatrixType H5) const {
  return _PIM_->computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j, bias_i,
      H1, H2, H3, H4, H5);
}

//------------------------------------------------------------------------------
PreintegratedImuMeasurementsT<TangentPreintegration> ImuFactor::Merge(
    const PreintegratedImuMeasurementsT<TangentPreintegration>& pim01,
    const PreintegratedImuMeasurementsT<TangentPreintegration>& pim12) {
  if (!pim01.matchesParamsWith(pim12))
  throw std::domain_error(
      "Cannot merge PreintegratedImuMeasurements with different params");

  if (pim01.params()->body_P_sensor)
  throw std::domain_error(
      "Cannot merge PreintegratedImuMeasurements with sensor pose yet");

  // the bias for the merged factor will be the bias from 01
  PreintegratedImuMeasurementsT<TangentPreintegration> pim02 = pim01;

  Matrix9 H1, H2;
  pim02.mergeWith(pim12, &H1, &H2);

  return pim02;
}

//------------------------------------------------------------------------------
ImuFactor::shared_ptr ImuFactor::Merge(const shared_ptr& f01,
    const shared_ptr& f12) {
  // IMU bias keys must be the same.
  if (f01->key<5>() != f12->key<5>())
  throw std::domain_error("ImuFactor::Merge: IMU bias keys must be the same");

  // expect intermediate pose, velocity keys to matchup.
  if (f01->key<3>() != f12->key<1>() || f01->key<4>() != f12->key<2>())
  throw std::domain_error(
      "ImuFactor::Merge: intermediate pose, velocity keys need to match up");

  // return new factor
  auto pim02 = Merge(
    *dynamic_cast<const PreintegratedImuMeasurementsT<TangentPreintegration>*>(f01->preintegratedMeasurements().get()), 
    *dynamic_cast<const PreintegratedImuMeasurementsT<TangentPreintegration>*>(f12->preintegratedMeasurements().get())
  );
  return std::make_shared<ImuFactor>(f01->key<1>(),  // P0
      f01->key<2>(),  // V0
      f12->key<3>(),  // P2
      f12->key<4>(),  // V2
      f01->key<5>(),  // B
      std::make_shared<const PreintegratedImuMeasurementsT<TangentPreintegration>>(pim02));
}

//------------------------------------------------------------------------------
// ImuFactor2 methods
//------------------------------------------------------------------------------
ImuFactor2::ImuFactor2(Key state_i, Key state_j, Key bias,
  std::shared_ptr<const PreintegratedImuMeasurementsInterface> pim) :
  Base(noiseModel::Gaussian::Covariance(pim->preintMeasCov()), state_i, state_j,
      bias), _PIM_(pim) {
}

//------------------------------------------------------------------------------
NonlinearFactor::shared_ptr ImuFactor2::clone() const {
return std::static_pointer_cast<NonlinearFactor>(
    NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const ImuFactor2& f) {
  f.preintegratedMeasurements()->print("preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel()->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
void ImuFactor2::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? s : s + "\n") << "ImuFactor2("
       << keyFormatter(this->template key<1>()) << "," << keyFormatter(this->template key<2>()) << ","
       << keyFormatter(this->key<3>()) << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
bool ImuFactor2::equals(const NonlinearFactor& other, double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  const bool base = Base::equals(*e, tol);
  const bool pim = _PIM_->equals(*(e->_PIM_), tol);
  return e != nullptr && base && pim;
}

//------------------------------------------------------------------------------
Vector ImuFactor2::evaluateError(const NavState& state_i,
    const NavState& state_j,
    const imuBias::ConstantBias& bias_i, //
    OptionalMatrixType H1, OptionalMatrixType H2,
    OptionalMatrixType H3) const {
  return _PIM_->computeError(state_i, state_j, bias_i, H1, H2, H3);
}

//------------------------------------------------------------------------------
// Explicit instantiations
//------------------------------------------------------------------------------
template class GTSAM_EXPORT PreintegratedImuMeasurementsT<ManifoldPreintegration>;
template class GTSAM_EXPORT PreintegratedImuMeasurementsT<TangentPreintegration>;
}
// namespace gtsam
