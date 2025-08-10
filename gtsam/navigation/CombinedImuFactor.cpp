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

#include <gtsam/navigation/CombinedImuFactor.h>
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

//------------------------------------------------------------------------------
// sugar for derivative blocks
#define D_R_R(H) (H)->block<3,3>(0,0)
#define D_R_t(H) (H)->block<3,3>(0,3)
#define D_R_v(H) (H)->block<3,3>(0,6)
#define D_t_R(H) (H)->block<3,3>(3,0)
#define D_t_t(H) (H)->block<3,3>(3,3)
#define D_t_v(H) (H)->block<3,3>(3,6)
#define D_v_R(H) (H)->block<3,3>(6,0)
#define D_v_t(H) (H)->block<3,3>(6,3)
#define D_v_v(H) (H)->block<3,3>(6,6)
#define D_a_a(H) (H)->block<3,3>(9,9)
#define D_g_g(H) (H)->block<3,3>(12,12)

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

  // Update preintegrated measurements covariance: as in [2] we consider a first
  // order propagation that can be seen as a prediction phase in an EKF
  // framework. In this implementation, in contrast to [2], we consider the
  // uncertainty of the bias selection and we keep correlation between biases
  // and preintegrated measurements

  // Single Jacobians to propagate covariance
  Matrix3 theta_H_omega = C.topRows<3>();
  Matrix3 pos_H_acc = B.middleRows<3>(3);
  Matrix3 vel_H_acc = B.bottomRows<3>();

  // overall Jacobian wrt preintegrated measurements (df/dx)
  Eigen::Matrix<double, 15, 15> F;
  F.setZero();
  F.block<9, 9>(0, 0) = A;
  F.block<3, 3>(0, 12) = theta_H_omega;
  F.block<3, 3>(3, 9) = pos_H_acc;
  F.block<3, 3>(6, 9) = vel_H_acc;
  F.block<6, 6>(9, 9) = I_6x6;

  // Update the uncertainty on the state (matrix F in [4]).
  preintMeasCov_ = F * preintMeasCov_ * F.transpose();

  // propagate uncertainty
  // TODO(frank): use noiseModel routine so we can have arbitrary noise models.
  const Matrix3& aCov = this->p().accelerometerCovariance;
  const Matrix3& wCov = this->p().gyroscopeCovariance;
  const Matrix3& iCov = this->p().integrationCovariance;

  // first order uncertainty propagation
  // Optimized matrix mult: (1/dt) * G * measurementCovariance * G.transpose()
  Eigen::Matrix<double, 15, 15> G_measCov_Gt;
  G_measCov_Gt.setZero(15, 15);

  // BLOCK DIAGONAL TERMS
  D_R_R(&G_measCov_Gt) =
      (theta_H_omega * (wCov / dt) * theta_H_omega.transpose());

  D_t_t(&G_measCov_Gt) =
      (pos_H_acc * (aCov / dt) * pos_H_acc.transpose()) + (dt * iCov);

  D_v_v(&G_measCov_Gt) = (vel_H_acc * (aCov / dt) * vel_H_acc.transpose());

  D_a_a(&G_measCov_Gt) = dt * this->p().biasAccCovariance;
  D_g_g(&G_measCov_Gt) = dt * this->p().biasOmegaCovariance;

  // OFF BLOCK DIAGONAL TERMS
  D_t_v(&G_measCov_Gt) = (pos_H_acc * (aCov / dt) * vel_H_acc.transpose());
  D_v_t(&G_measCov_Gt) = (vel_H_acc * (aCov / dt) * pos_H_acc.transpose());

  preintMeasCov_.noalias() += G_measCov_Gt;
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

  // error wrt bias evolution model (random walk)
  Matrix6 Hbias_i, Hbias_j;
  Vector6 fbias = traits<imuBias::ConstantBias>::Between(bias_j, bias_i,
      H6 ? &Hbias_j : 0, H5 ? &Hbias_i : 0).vector();

  Matrix96 D_r_pose_i, D_r_pose_j, D_r_bias_i;
  Matrix93 D_r_vel_i, D_r_vel_j;

  // error wrt preintegrated measurements
  Vector9 r_Rpv = pim_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j,
      bias_i, H1 ? &D_r_pose_i : 0, H2 ? &D_r_vel_i : 0, H3 ? &D_r_pose_j : 0,
      H4 ? &D_r_vel_j : 0, H5 ? &D_r_bias_i : 0);

  // if we need the jacobians
  if (H1) {
    H1->resize(15, 6);
    H1->block<9, 6>(0, 0) = D_r_pose_i;
    // adding: [dBiasAcc/dPi ; dBiasOmega/dPi]
    H1->block<6, 6>(9, 0).setZero();
  }
  if (H2) {
    H2->resize(15, 3);
    H2->block<9, 3>(0, 0) = D_r_vel_i;
    // adding: [dBiasAcc/dVi ; dBiasOmega/dVi]
    H2->block<6, 3>(9, 0).setZero();
  }
  if (H3) {
    H3->resize(15, 6);
    H3->block<9, 6>(0, 0) = D_r_pose_j;
    // adding: [dBiasAcc/dPj ; dBiasOmega/dPj]
    H3->block<6, 6>(9, 0).setZero();
  }
  if (H4) {
    H4->resize(15, 3);
    H4->block<9, 3>(0, 0) = D_r_vel_j;
    // adding: [dBiasAcc/dVi ; dBiasOmega/dVi]
    H4->block<6, 3>(9, 0).setZero();
  }
  if (H5) {
    H5->resize(15, 6);
    H5->block<9, 6>(0, 0) = D_r_bias_i;
    // adding: [dBiasAcc/dBias_i ; dBiasOmega/dBias_i]
    H5->block<6, 6>(9, 0) = Hbias_i;
  }
  if (H6) {
    H6->resize(15, 6);
    H6->block<9, 6>(0, 0).setZero();
    // adding: [dBiasAcc/dBias_j ; dBiasOmega/dBias_j]
    H6->block<6, 6>(9, 0) = Hbias_j;
  }

  // overall error
  Vector r(15);
  r << r_Rpv, fbias; // vector of size 15
  return r;
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

template class GTSAM_EXPORT CombinedImuFactorT<PreintegratedCombinedMeasurementsT<ManifoldPreintegration>>;
template class GTSAM_EXPORT CombinedImuFactorT<PreintegratedCombinedMeasurementsT<TangentPreintegration>>;

// Instantiate operator<<
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedCombinedMeasurementsT<ManifoldPreintegration>>(
    std::ostream& os, const CombinedImuFactorT<PreintegratedCombinedMeasurementsT<ManifoldPreintegration>>& f);
template GTSAM_EXPORT std::ostream& operator<<<PreintegratedCombinedMeasurementsT<TangentPreintegration>>(
    std::ostream& os, const CombinedImuFactorT<PreintegratedCombinedMeasurementsT<TangentPreintegration>>& f);


}  // namespace gtsam