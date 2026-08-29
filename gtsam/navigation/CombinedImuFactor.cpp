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
 *  @author Varun Agrawal
 **/

#include <gtsam/base/MatrixConstants.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GalileanImuFactor.h>
#include <gtsam/navigation/LieGroupPreintegration.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>
#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/export.hpp>
#endif

/* External or standard includes */
#include <ostream>

namespace gtsam {

using namespace std;

//------------------------------------------------------------------------------
// Inner class PreintegrationCombinedParams
//------------------------------------------------------------------------------
void PreintegrationCombinedParams::print(const string& s) const {
  PreintegrationParams::print(s);
  cout << "biasAccCovariance:\n[\n" << biasAccCovariance << "\n]" << endl;
  cout << "biasOmegaCovariance:\n[\n" << biasOmegaCovariance << "\n]" << endl;
}

//------------------------------------------------------------------------------
bool PreintegrationCombinedParams::equals(
    const PreintegratedRotationParams& other, double tol) const {
  auto e = dynamic_cast<const PreintegrationCombinedParams*>(&other);
  return e != nullptr && PreintegrationParams::equals(other, tol) &&
         equal_with_abs_tol(biasAccCovariance, e->biasAccCovariance, tol) &&
         equal_with_abs_tol(biasOmegaCovariance, e->biasOmegaCovariance, tol);
}

//------------------------------------------------------------------------------
// Inner class PreintegratedCombinedMeasurementsT
//------------------------------------------------------------------------------
template <class PreintegrationType>
void PreintegratedCombinedMeasurementsT<PreintegrationType>::print(const string& s) const {
  PreintegrationType::print(s);
  cout << "  preintMeasCov [ " << preintMeasCov_ << " ]" << endl;
}

//------------------------------------------------------------------------------
template <class PreintegrationType>
bool PreintegratedCombinedMeasurementsT<PreintegrationType>::equals(
    const PreintegratedCombinedMeasurementsT<PreintegrationType>& other, double tol) const {
  return PreintegrationType::equals(other, tol)
      && equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol);
}

//------------------------------------------------------------------------------
template <class PreintegrationType>
void PreintegratedCombinedMeasurementsT<PreintegrationType>::resetIntegration() {
  // Base class method to reset the preintegrated measurements
  PreintegrationType::resetIntegration();
  preintMeasCov_.setZero();
}

namespace {

using Matrix15 = Eigen::Matrix<double, 15, 15>;

/** Propagate Combined PIM covariance using its sparse 5-by-5 block form. */
template <bool kTangentStructure, bool kFullGyroscopeStructure>
void propagateCombinedCovariance(Matrix15* covariance, const Matrix9& A,
                                 const Matrix93& B, const Matrix93& C,
                                 const PreintegrationCombinedParams& params,
                                 double dt, bool hasSensorPose) {
  // State blocks are ordered R,p,v,b_a,b_g. The Combined transition is
  //
  //       [ A00   0    0    0   C0 ]
  //       [ A10  A11  A12  B1   C1 ]
  //   F = [ A20   0   A22  B2   C2 ] .
  //       [  0    0    0    I    0 ]
  //       [  0    0    0    0    I ]
  // C1 and C2 are nonzero for Galilean preintegration and when a displaced
  // sensor couples angular velocity into corrected acceleration.
  const auto A00 = A.block<3, 3>(0, 0);
  const auto A10 = A.block<3, 3>(3, 0);
  const auto A11 = A.block<3, 3>(3, 3);
  const auto A12 = A.block<3, 3>(3, 6);
  const auto A20 = A.block<3, 3>(6, 0);
  const auto A22 = A.block<3, 3>(6, 6);
  const auto B1 = B.middleRows<3>(3);
  const auto B2 = B.bottomRows<3>();
  const auto C0 = C.topRows<3>();
  const auto C1 = C.middleRows<3>(3);
  const auto C2 = C.bottomRows<3>();
  const bool useFullGyroscope = kFullGyroscopeStructure || hasSensorPose;

  const Matrix15 oldCovariance = *covariance;
  Matrix15 FP;
  for (int column = 0; column < 5; ++column) {
    const int offset = 3 * column;
    FP.block<3, 3>(0, offset).noalias() =
        A00 * oldCovariance.block<3, 3>(0, offset) +
        C0 * oldCovariance.block<3, 3>(12, offset);
    if constexpr (kTangentStructure) {
      FP.block<3, 3>(3, offset).noalias() =
          A10 * oldCovariance.block<3, 3>(0, offset) +
          oldCovariance.block<3, 3>(3, offset) +
          dt * oldCovariance.block<3, 3>(6, offset) +
          B1 * oldCovariance.block<3, 3>(9, offset);
      FP.block<3, 3>(6, offset).noalias() =
          A20 * oldCovariance.block<3, 3>(0, offset) +
          oldCovariance.block<3, 3>(6, offset) +
          B2 * oldCovariance.block<3, 3>(9, offset);
    } else {
      FP.block<3, 3>(3, offset).noalias() =
          A10 * oldCovariance.block<3, 3>(0, offset) +
          A11 * oldCovariance.block<3, 3>(3, offset) +
          A12 * oldCovariance.block<3, 3>(6, offset) +
          B1 * oldCovariance.block<3, 3>(9, offset);
      FP.block<3, 3>(6, offset).noalias() =
          A20 * oldCovariance.block<3, 3>(0, offset) +
          A22 * oldCovariance.block<3, 3>(6, offset) +
          B2 * oldCovariance.block<3, 3>(9, offset);
    }
    if (useFullGyroscope) {
      FP.block<3, 3>(3, offset).noalias() +=
          C1 * oldCovariance.block<3, 3>(12, offset);
      FP.block<3, 3>(6, offset).noalias() +=
          C2 * oldCovariance.block<3, 3>(12, offset);
    }
    FP.block<3, 3>(9, offset) = oldCovariance.block<3, 3>(9, offset);
    FP.block<3, 3>(12, offset) = oldCovariance.block<3, 3>(12, offset);
  }

  // Evaluate only the lower blocks of F*P*F'. Each loop corresponds to one
  // sparse block row of F above.
  for (int row = 0; row < 5; ++row) {
    const int offset = 3 * row;
    covariance->block<3, 3>(offset, 0).noalias() =
        FP.block<3, 3>(offset, 0) * A00.transpose() +
        FP.block<3, 3>(offset, 12) * C0.transpose();
  }
  for (int row = 1; row < 5; ++row) {
    const int offset = 3 * row;
    if constexpr (kTangentStructure) {
      covariance->block<3, 3>(offset, 3).noalias() =
          FP.block<3, 3>(offset, 0) * A10.transpose() +
          FP.block<3, 3>(offset, 3) + dt * FP.block<3, 3>(offset, 6) +
          FP.block<3, 3>(offset, 9) * B1.transpose();
    } else {
      covariance->block<3, 3>(offset, 3).noalias() =
          FP.block<3, 3>(offset, 0) * A10.transpose() +
          FP.block<3, 3>(offset, 3) * A11.transpose() +
          FP.block<3, 3>(offset, 6) * A12.transpose() +
          FP.block<3, 3>(offset, 9) * B1.transpose();
    }
    if (useFullGyroscope) {
      covariance->block<3, 3>(offset, 3).noalias() +=
          FP.block<3, 3>(offset, 12) * C1.transpose();
    }
  }
  for (int row = 2; row < 5; ++row) {
    const int offset = 3 * row;
    if constexpr (kTangentStructure) {
      covariance->block<3, 3>(offset, 6).noalias() =
          FP.block<3, 3>(offset, 0) * A20.transpose() +
          FP.block<3, 3>(offset, 6) +
          FP.block<3, 3>(offset, 9) * B2.transpose();
    } else {
      covariance->block<3, 3>(offset, 6).noalias() =
          FP.block<3, 3>(offset, 0) * A20.transpose() +
          FP.block<3, 3>(offset, 6) * A22.transpose() +
          FP.block<3, 3>(offset, 9) * B2.transpose();
    }
    if (useFullGyroscope) {
      covariance->block<3, 3>(offset, 6).noalias() +=
          FP.block<3, 3>(offset, 12) * C2.transpose();
    }
  }
  covariance->block<3, 3>(9, 9) = FP.block<3, 3>(9, 9);
  covariance->block<3, 3>(12, 9) = FP.block<3, 3>(12, 9);
  covariance->block<3, 3>(12, 12) = FP.block<3, 3>(12, 12);

  // Add continuous-time measurement and bias noise to the affected blocks.
  const Matrix3 scaledAccelerometerCovariance =
      params.accelerometerCovariance / dt;
  const Matrix3 scaledGyroscopeCovariance = params.gyroscopeCovariance / dt;
  const Matrix3 B1Covariance = B1 * scaledAccelerometerCovariance;
  const Matrix3 B2Covariance = B2 * scaledAccelerometerCovariance;
  covariance->block<3, 3>(0, 0).noalias() +=
      C0 * scaledGyroscopeCovariance * C0.transpose();
  covariance->block<3, 3>(3, 3).noalias() +=
      B1Covariance * B1.transpose() + dt * params.integrationCovariance;
  covariance->block<3, 3>(6, 3).noalias() += B2Covariance * B1.transpose();
  covariance->block<3, 3>(6, 6).noalias() += B2Covariance * B2.transpose();
  if (useFullGyroscope) {
    const Matrix3 C1Covariance = C1 * scaledGyroscopeCovariance;
    const Matrix3 C2Covariance = C2 * scaledGyroscopeCovariance;
    covariance->block<3, 3>(3, 0).noalias() += C1Covariance * C0.transpose();
    covariance->block<3, 3>(3, 3).noalias() += C1Covariance * C1.transpose();
    covariance->block<3, 3>(6, 0).noalias() += C2Covariance * C0.transpose();
    covariance->block<3, 3>(6, 3).noalias() += C2Covariance * C1.transpose();
    covariance->block<3, 3>(6, 6).noalias() += C2Covariance * C2.transpose();
  }
  covariance->block<3, 3>(9, 9).noalias() += dt * params.biasAccCovariance;
  covariance->block<3, 3>(12, 12).noalias() += dt * params.biasOmegaCovariance;

  for (int row = 1; row < 5; ++row) {
    for (int column = 0; column < row; ++column) {
      covariance->block<3, 3>(3 * column, 3 * row) =
          covariance->block<3, 3>(3 * row, 3 * column).transpose();
    }
  }
}

}  // namespace

//------------------------------------------------------------------------------
template <class PreintegrationType>
void PreintegratedCombinedMeasurementsT<
    PreintegrationType>::integrateMeasurement(const Vector3& measuredAcc,
                                              const Vector3& measuredOmega,
                                              double dt) {
  if (dt <= 0) {
    throw std::runtime_error(
        "PreintegratedCombinedMeasurements::integrateMeasurement: dt <=0");
  }

  // Update preintegrated measurements.
  Matrix9 A;  // Jacobian wrt preintegrated measurements without bias (df/dx)
  Matrix93 B, C;  // Jacobian of state wrpt accel bias and omega bias.
  PreintegrationType::update(measuredAcc, measuredOmega, dt, &A, &B, &C);

  constexpr bool kTangentStructure =
      std::is_same_v<PreintegrationType, TangentPreintegration>;
  constexpr bool kFullGyroscopeStructure =
      std::is_same_v<PreintegrationType, GalileanPreintegration>;
  propagateCombinedCovariance<kTangentStructure, kFullGyroscopeStructure>(
      &preintMeasCov_, A, B, C, this->p(), dt,
      static_cast<bool>(this->p().body_P_sensor));
}

//------------------------------------------------------------------------------
// CombinedImuFactorT methods
//------------------------------------------------------------------------------
template <class PIM>
void CombinedImuFactorT<PIM>::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? s : s + "\n") << "CombinedImuFactor("
       << keyFormatter(this->template key<1>()) << "," << keyFormatter(this->template key<2>()) << ","
       << keyFormatter(this->template key<3>()) << "," << keyFormatter(this->template key<4>()) << ","
       << keyFormatter(this->template key<5>()) << "," << keyFormatter(this->template key<6>())
       << ")\n";
  pim_.print("  preintegrated measurements:");
  this->noiseModel_->print("  noise model: ");
}

//------------------------------------------------------------------------------
template <class PIM>
bool CombinedImuFactorT<PIM>::equals(const NonlinearFactor& other, double tol) const {
  const This* e = dynamic_cast<const This*>(&other);
  return e != nullptr && Base::equals(*e, tol) && pim_.equals(e->pim_, tol);
}

//------------------------------------------------------------------------------
template <class PIM>
Vector CombinedImuFactorT<PIM>::evaluateError(const Pose3& pose_i,
    const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, const imuBias::ConstantBias& bias_j,
    OptionalMatrixType H1, OptionalMatrixType H2,
    OptionalMatrixType H3, OptionalMatrixType H4,
    OptionalMatrixType H5, OptionalMatrixType H6) const {
  return internal::combinedImuError(pim_, pose_i, vel_i, pose_j, vel_j,
                                    bias_i, bias_j, pim_.params()->n_gravity,
                                    H1, H2, H3, H4, H5, H6, nullptr);
}

//------------------------------------------------------------------------------
template <class PIM>
std::ostream& operator<<(std::ostream& os, const CombinedImuFactorT<PIM>& f) {
  f.preintegratedMeasurements().print("combined preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel()->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
// Explicit instantiations
//------------------------------------------------------------------------------
template class GTSAM_EXPORT PreintegratedCombinedMeasurementsT<ManifoldPreintegration>;
template class GTSAM_EXPORT PreintegratedCombinedMeasurementsT<TangentPreintegration>;
template class GTSAM_EXPORT
    PreintegratedCombinedMeasurementsT<LieGroupPreintegration>;
template class GTSAM_EXPORT
    PreintegratedCombinedMeasurementsT<GalileanPreintegration>;

template class GTSAM_EXPORT CombinedImuFactorT<PreintegratedCombinedMeasurementsT<ManifoldPreintegration>>;
template class GTSAM_EXPORT CombinedImuFactorT<PreintegratedCombinedMeasurementsT<TangentPreintegration>>;
template class GTSAM_EXPORT CombinedImuFactorT<
    PreintegratedCombinedMeasurementsT<LieGroupPreintegration>>;
template class GTSAM_EXPORT
    CombinedImuFactorT<PreintegratedCombinedMeasurementsG>;

// Instantiate operator<<
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedCombinedMeasurementsT<ManifoldPreintegration>>(
    std::ostream& os, const CombinedImuFactorT<PreintegratedCombinedMeasurementsT<ManifoldPreintegration>>& f);
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedCombinedMeasurementsT<TangentPreintegration>>(
    std::ostream& os, const CombinedImuFactorT<PreintegratedCombinedMeasurementsT<TangentPreintegration>>& f);
template GTSAM_EXPORT std::ostream&
operator<< <PreintegratedCombinedMeasurementsT<LieGroupPreintegration>>(
    std::ostream& os,
    const CombinedImuFactorT<
        PreintegratedCombinedMeasurementsT<LieGroupPreintegration>>& f);
template GTSAM_EXPORT std::ostream&
operator<< <PreintegratedCombinedMeasurementsG>(
    std::ostream& os,
    const CombinedImuFactorT<PreintegratedCombinedMeasurementsG>& f);

}  // namespace gtsam
