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

#include <gtsam/navigation/GalileanImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/LieGroupPreintegration.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>

/* External or standard includes */
#include <ostream>
#include <cassert>

namespace gtsam {

using namespace std;

namespace {

/** Propagate the symmetric PIM covariance using its fixed block sparsity. */
template <bool kTangentStructure>
void propagatePreintegratedCovariance(Matrix9* covariance, const Matrix9& A,
                                      const Matrix93& B, const Matrix93& C,
                                      const Matrix3& accelerometerCovariance,
                                      const Matrix3& gyroscopeCovariance,
                                      const Matrix3& integrationCovariance,
                                      double dt, bool hasSensorPose) {
  // Every PIM backend has the same transition sparsity in NavState ordering:
  //
  //                       [ A00   0    0  ]
  //                   A = [ A10  A11  A12 ] .
  //                       [ A20   0   A22 ]
  //
  // Form A*P by 3x3 blocks, skipping those structural zeros.
  Matrix9 AP;
  const Matrix9 oldCovariance = *covariance;
  const auto A00 = A.block<3, 3>(0, 0);
  const auto A10 = A.block<3, 3>(3, 0);
  const auto A11 = A.block<3, 3>(3, 3);
  const auto A12 = A.block<3, 3>(3, 6);
  const auto A20 = A.block<3, 3>(6, 0);
  const auto A22 = A.block<3, 3>(6, 6);
  for (int column = 0; column < 3; ++column) {
    const int offset = 3 * column;
    AP.block<3, 3>(0, offset).noalias() =
        A00 * oldCovariance.block<3, 3>(0, offset);
    if constexpr (kTangentStructure) {
      // Tangent uses A11=A22=I and A12=dt*I exactly.
      AP.block<3, 3>(3, offset).noalias() =
          A10 * oldCovariance.block<3, 3>(0, offset) +
          oldCovariance.block<3, 3>(3, offset) +
          dt * oldCovariance.block<3, 3>(6, offset);
      AP.block<3, 3>(6, offset).noalias() =
          A20 * oldCovariance.block<3, 3>(0, offset) +
          oldCovariance.block<3, 3>(6, offset);
    } else {
      AP.block<3, 3>(3, offset).noalias() =
          A10 * oldCovariance.block<3, 3>(0, offset) +
          A11 * oldCovariance.block<3, 3>(3, offset) +
          A12 * oldCovariance.block<3, 3>(6, offset);
      AP.block<3, 3>(6, offset).noalias() =
          A20 * oldCovariance.block<3, 3>(0, offset) +
          A22 * oldCovariance.block<3, 3>(6, offset);
    }
  }

  // P is symmetric, so evaluate only the six blocks on and below its diagonal.
  covariance->block<3, 3>(0, 0).noalias() =
      AP.block<3, 3>(0, 0) * A00.transpose();
  covariance->block<3, 3>(3, 0).noalias() =
      AP.block<3, 3>(3, 0) * A00.transpose();
  covariance->block<3, 3>(6, 0).noalias() =
      AP.block<3, 3>(6, 0) * A00.transpose();
  if constexpr (kTangentStructure) {
    covariance->block<3, 3>(3, 3).noalias() =
        AP.block<3, 3>(3, 0) * A10.transpose() + AP.block<3, 3>(3, 3) +
        dt * AP.block<3, 3>(3, 6);
    covariance->block<3, 3>(6, 3).noalias() =
        AP.block<3, 3>(6, 0) * A10.transpose() + AP.block<3, 3>(6, 3) +
        dt * AP.block<3, 3>(6, 6);
    covariance->block<3, 3>(6, 6).noalias() =
        AP.block<3, 3>(6, 0) * A20.transpose() + AP.block<3, 3>(6, 6);
  } else {
    covariance->block<3, 3>(3, 3).noalias() =
        AP.block<3, 3>(3, 0) * A10.transpose() +
        AP.block<3, 3>(3, 3) * A11.transpose() +
        AP.block<3, 3>(3, 6) * A12.transpose();
    covariance->block<3, 3>(6, 3).noalias() =
        AP.block<3, 3>(6, 0) * A10.transpose() +
        AP.block<3, 3>(6, 3) * A11.transpose() +
        AP.block<3, 3>(6, 6) * A12.transpose();
    covariance->block<3, 3>(6, 6).noalias() =
        AP.block<3, 3>(6, 0) * A20.transpose() +
        AP.block<3, 3>(6, 6) * A22.transpose();
  }

  // Acceleration has no direct rotation component, so its measurement noise
  // only touches the position/velocity covariance blocks.
  const Matrix3 scaledAccelerometerCovariance = accelerometerCovariance / dt;
  const auto B1 = B.middleRows<3>(3);
  const auto B2 = B.bottomRows<3>();
  Matrix3 B1Covariance, B2Covariance;
  B1Covariance.noalias() = B1 * scaledAccelerometerCovariance;
  B2Covariance.noalias() = B2 * scaledAccelerometerCovariance;
  covariance->block<3, 3>(3, 3).noalias() += B1Covariance * B1.transpose();
  covariance->block<3, 3>(6, 3).noalias() += B2Covariance * B1.transpose();
  covariance->block<3, 3>(6, 6).noalias() += B2Covariance * B2.transpose();

  const Matrix3 scaledGyroscopeCovariance = gyroscopeCovariance / dt;
  const auto C0 = C.topRows<3>();
  const auto C1 = C.middleRows<3>(3);
  const auto C2 = C.bottomRows<3>();
  Matrix3 C0Covariance;
  C0Covariance.noalias() = C0 * scaledGyroscopeCovariance;
  covariance->block<3, 3>(0, 0).noalias() += C0Covariance * C0.transpose();
  const auto addLowerGyroscopeNoise = [&] {
    Matrix3 C1Covariance, C2Covariance;
    C1Covariance.noalias() = C1 * scaledGyroscopeCovariance;
    C2Covariance.noalias() = C2 * scaledGyroscopeCovariance;
    covariance->block<3, 3>(3, 0).noalias() += C1Covariance * C0.transpose();
    covariance->block<3, 3>(3, 3).noalias() += C1Covariance * C1.transpose();
    covariance->block<3, 3>(6, 0).noalias() += C2Covariance * C0.transpose();
    covariance->block<3, 3>(6, 3).noalias() += C2Covariance * C1.transpose();
    covariance->block<3, 3>(6, 6).noalias() += C2Covariance * C2.transpose();
  };
  if constexpr (kTangentStructure) {
    // A sensor pose can couple angular velocity into acceleration, making the
    // lower two gyroscope Jacobian blocks nonzero on this rare path.
    if (hasSensorPose) addLowerGyroscopeNoise();
  } else {
    addLowerGyroscopeNoise();
  }

  covariance->block<3, 3>(3, 3).noalias() += integrationCovariance * dt;

  // Mirror the computed lower off-diagonal blocks rather than accumulating
  // cross-block round-off asymmetry over many samples.
  covariance->block<3, 3>(0, 3) = covariance->block<3, 3>(3, 0).transpose();
  covariance->block<3, 3>(0, 6) = covariance->block<3, 3>(6, 0).transpose();
  covariance->block<3, 3>(3, 6) = covariance->block<3, 3>(6, 3).transpose();
}

}  // namespace

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

  // First-order EKF covariance propagation. The fixed NavState block structure
  // is shared by every backend, even though their nonzero blocks differ.
  constexpr bool kTangentStructure =
      std::is_same_v<PreintegrationType, TangentPreintegration>;
  propagatePreintegratedCovariance<kTangentStructure>(
      &preintMeasCov_, A, B, C, this->p().accelerometerCovariance,
      this->p().gyroscopeCovariance, this->p().integrationCovariance, dt,
      static_cast<bool>(this->p().body_P_sensor));
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
Vector9 ImuFactorT<PIM>::evaluateError(
    const Pose3& pose_i, const Vector3& vel_i, const Pose3& pose_j,
    const Vector3& vel_j, const imuBias::ConstantBias& bias_i,
    OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3,
    OptionalMatrixType H4, OptionalMatrixType H5) const {
  return internal::preintegrationErrorAndJacobians(
      pim_, pose_i, vel_i, pose_j, vel_j, bias_i, H1, H2, H3, H4, H5);
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
Vector9 ImuFactor2T<PIM>::evaluateError(const NavState& state_i,
    const NavState& state_j,
    const imuBias::ConstantBias& bias_i, //
    OptionalMatrixType H1, OptionalMatrixType H2,
    OptionalMatrixType H3) const {
  return internal::preintegrationError(pim_, state_i, state_j, bias_i, H1, H2,
                                       H3);
}

//------------------------------------------------------------------------------
// Explicit instantiations
//------------------------------------------------------------------------------
template class GTSAM_EXPORT PreintegratedImuMeasurementsT<ManifoldPreintegration>;
template class GTSAM_EXPORT PreintegratedImuMeasurementsT<TangentPreintegration>;
template class GTSAM_EXPORT
    PreintegratedImuMeasurementsT<LieGroupPreintegration>;
template class GTSAM_EXPORT
    PreintegratedImuMeasurementsT<GalileanPreintegration>;

// ImuFactorT instantiations
template class GTSAM_EXPORT ImuFactorT<PreintegratedImuMeasurementsT<ManifoldPreintegration>>;
template class GTSAM_EXPORT ImuFactorT<PreintegratedImuMeasurementsT<TangentPreintegration>>;
template class GTSAM_EXPORT
    ImuFactorT<PreintegratedImuMeasurementsT<LieGroupPreintegration>>;
template class GTSAM_EXPORT ImuFactorT<PreintegratedImuMeasurementsG>;

// ImuFactor2T instantiations
template class GTSAM_EXPORT ImuFactor2T<PreintegratedImuMeasurementsT<ManifoldPreintegration>>;
template class GTSAM_EXPORT ImuFactor2T<PreintegratedImuMeasurementsT<TangentPreintegration>>;
template class GTSAM_EXPORT
    ImuFactor2T<PreintegratedImuMeasurementsT<LieGroupPreintegration>>;
template class GTSAM_EXPORT ImuFactor2T<PreintegratedImuMeasurementsG>;

// operator<< instantiations
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedImuMeasurementsT<ManifoldPreintegration>>(
    std::ostream& os, const ImuFactorT<PreintegratedImuMeasurementsT<ManifoldPreintegration>>& f);
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedImuMeasurementsT<TangentPreintegration>>(
    std::ostream& os, const ImuFactorT<PreintegratedImuMeasurementsT<TangentPreintegration>>& f);
template GTSAM_EXPORT std::ostream&
operator<< <PreintegratedImuMeasurementsT<LieGroupPreintegration>>(
    std::ostream& os,
    const ImuFactorT<PreintegratedImuMeasurementsT<LieGroupPreintegration>>& f);
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedImuMeasurementsG>(
    std::ostream& os,
    const ImuFactorT<PreintegratedImuMeasurementsG>& f);

template GTSAM_EXPORT std::ostream& operator<<<PreintegratedImuMeasurementsT<ManifoldPreintegration>>(
    std::ostream& os, const ImuFactor2T<PreintegratedImuMeasurementsT<ManifoldPreintegration>>& f);
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedImuMeasurementsT<TangentPreintegration>>(
    std::ostream& os, const ImuFactor2T<PreintegratedImuMeasurementsT<TangentPreintegration>>& f);
template GTSAM_EXPORT std::ostream&
operator<< <PreintegratedImuMeasurementsT<LieGroupPreintegration>>(
    std::ostream& os,
    const ImuFactor2T<PreintegratedImuMeasurementsT<LieGroupPreintegration>>&
        f);
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedImuMeasurementsG>(
    std::ostream& os,
    const ImuFactor2T<PreintegratedImuMeasurementsG>& f);
}
// namespace gtsam
