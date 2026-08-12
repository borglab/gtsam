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
template <class PreintegrationType>
void PreintegratedImuMeasurementsT<PreintegrationType>::print(const string& s) const {
  PreintegrationType::print(s);
  cout << "    preintMeasCov \n[" << preintMeasCov_ << "]" << endl;
}

//------------------------------------------------------------------------------
template <class PreintegrationType>
bool PreintegratedImuMeasurementsT<PreintegrationType>::equals(
    const PreintegratedImuMeasurementsT<PreintegrationType>& other, double tol) const {
  return PreintegrationType::equals(other, tol)
      && equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol);
}

//------------------------------------------------------------------------------
template <class PreintegrationType>
void PreintegratedImuMeasurementsT<PreintegrationType>::resetIntegration() {
  PreintegrationType::resetIntegration();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
template <class PreintegrationType>
void PreintegratedImuMeasurementsT<PreintegrationType>::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  if (dt <= 0) {
    throw std::runtime_error(
        "PreintegratedImuMeasurements::integrateMeasurement: dt <=0");
  }

  // Update preintegrated measurements (also get Jacobian)
  Matrix9 A;  // overall Jacobian wrt preintegrated measurements (df/dx)
  Matrix93 B, C;  // Jacobian of state wrpt accel bias and omega bias respectively.
  PreintegrationType::update(measuredAcc, measuredOmega, dt, &A, &B, &C);

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
template <class PreintegrationType>
void PreintegratedImuMeasurementsT<PreintegrationType>::integrateMeasurements(
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
// ImuFactorT methods
//------------------------------------------------------------------------------
template <class PIM>
std::ostream& operator<<(std::ostream& os, const ImuFactorT<PIM>& f) {
  f.preintegratedMeasurements().print("preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel()->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
template <class PIM>
void ImuFactorT<PIM>::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? s : s + "\n") << "ImuFactor(" << keyFormatter(this->template key<1>())
       << "," << keyFormatter(this->template key<2>()) << "," << keyFormatter(this->template key<3>())
       << "," << keyFormatter(this->template key<4>()) << "," << keyFormatter(this->template key<5>())
       << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
template <class PIM>
bool ImuFactorT<PIM>::equals(const NonlinearFactor& other, double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  const bool base = Base::equals(*e, tol);
  const bool pim = pim_.equals(e->pim_, tol);
  return e != nullptr && base && pim;
}

//------------------------------------------------------------------------------
template <class PIM>
Vector ImuFactorT<PIM>::evaluateError(const Pose3& pose_i, const Vector3& vel_i,
    const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, OptionalMatrixType H1,
    OptionalMatrixType H2, OptionalMatrixType H3,
    OptionalMatrixType H4, OptionalMatrixType H5) const {
  return pim_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j, bias_i,
      H1, H2, H3, H4, H5);
}

//------------------------------------------------------------------------------
// ImuFactor2T methods
//------------------------------------------------------------------------------
template <class PIM>
std::ostream& operator<<(std::ostream& os, const ImuFactor2T<PIM>& f) {
  f.preintegratedMeasurements().print("preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel()->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
template <class PIM>
void ImuFactor2T<PIM>::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? s : s + "\n") << "ImuFactor2("
       << keyFormatter(this->template key<1>()) << "," << keyFormatter(this->template key<2>()) << ","
       << keyFormatter(this->key<3>()) << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
template <class PIM>
bool ImuFactor2T<PIM>::equals(const NonlinearFactor& other, double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  const bool base = Base::equals(*e, tol);
  const bool pim = pim_.equals(e->pim_, tol);
  return e != nullptr && base && pim;
}

//------------------------------------------------------------------------------
template <class PIM>
Vector ImuFactor2T<PIM>::evaluateError(const NavState& state_i,
    const NavState& state_j,
    const imuBias::ConstantBias& bias_i, //
    OptionalMatrixType H1, OptionalMatrixType H2,
    OptionalMatrixType H3) const {
  return pim_.computeError(state_i, state_j, bias_i, H1, H2, H3);
}

//------------------------------------------------------------------------------
// Explicit instantiations
//------------------------------------------------------------------------------
template class GTSAM_EXPORT PreintegratedImuMeasurementsT<ManifoldPreintegration>;
template class GTSAM_EXPORT PreintegratedImuMeasurementsT<TangentPreintegration>;

// ImuFactorT instantiations
template class GTSAM_EXPORT ImuFactorT<PreintegratedImuMeasurementsT<ManifoldPreintegration>>;
template class GTSAM_EXPORT ImuFactorT<PreintegratedImuMeasurementsT<TangentPreintegration>>;

// ImuFactor2T instantiations
template class GTSAM_EXPORT ImuFactor2T<PreintegratedImuMeasurementsT<ManifoldPreintegration>>;
template class GTSAM_EXPORT ImuFactor2T<PreintegratedImuMeasurementsT<TangentPreintegration>>;

// operator<< instantiations
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedImuMeasurementsT<ManifoldPreintegration>>(
    std::ostream& os, const ImuFactorT<PreintegratedImuMeasurementsT<ManifoldPreintegration>>& f);
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedImuMeasurementsT<TangentPreintegration>>(
    std::ostream& os, const ImuFactorT<PreintegratedImuMeasurementsT<TangentPreintegration>>& f);

template GTSAM_EXPORT std::ostream& operator<<<PreintegratedImuMeasurementsT<ManifoldPreintegration>>(
    std::ostream& os, const ImuFactor2T<PreintegratedImuMeasurementsT<ManifoldPreintegration>>& f);
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedImuMeasurementsT<TangentPreintegration>>(
    std::ostream& os, const ImuFactor2T<PreintegratedImuMeasurementsT<TangentPreintegration>>& f);

}
// namespace gtsam
