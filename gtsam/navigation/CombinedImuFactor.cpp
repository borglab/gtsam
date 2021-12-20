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
#include <boost/serialization/export.hpp>

/* External or standard includes */
#include <ostream>

namespace gtsam {

using namespace std;

//------------------------------------------------------------------------------
// Inner class PreintegrationCombinedParams
//------------------------------------------------------------------------------
void PreintegrationCombinedParams::print(const string& s) const {
  PreintegrationParams::print(s);
  cout << "biasAccCovariance:\n[\n" << biasAccCovariance << "\n]"
       << endl;
  cout << "biasOmegaCovariance:\n[\n" << biasOmegaCovariance << "\n]"
       << endl;
  cout << "biasAccOmegaInt:\n[\n" << biasAccOmegaInt << "\n]"
       << endl;
}

//------------------------------------------------------------------------------
bool PreintegrationCombinedParams::equals(const PreintegratedRotationParams& other,
                                  double tol) const {
  auto e = dynamic_cast<const PreintegrationCombinedParams*>(&other);
  return e != nullptr && PreintegrationParams::equals(other, tol) &&
         equal_with_abs_tol(biasAccCovariance, e->biasAccCovariance,
                            tol) &&
         equal_with_abs_tol(biasOmegaCovariance, e->biasOmegaCovariance,
                            tol) &&
         equal_with_abs_tol(biasAccOmegaInt, e->biasAccOmegaInt, tol);
}

//------------------------------------------------------------------------------
// Inner class PreintegratedCombinedMeasurements
//------------------------------------------------------------------------------
void PreintegratedCombinedMeasurements::print(const string& s) const {
  PreintegrationType::print(s);
  cout << "  preintMeasCov [ " << preintMeasCov_ << " ]" << endl;
}

//------------------------------------------------------------------------------
bool PreintegratedCombinedMeasurements::equals(
    const PreintegratedCombinedMeasurements& other, double tol) const {
  return PreintegrationType::equals(other, tol)
      && equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol);
}

//------------------------------------------------------------------------------
void PreintegratedCombinedMeasurements::resetIntegration() {
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
void PreintegratedCombinedMeasurements::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  // Update preintegrated measurements.
  Matrix9 A; // overall Jacobian wrt preintegrated measurements (df/dx)
  Matrix93 B, C;
  PreintegrationType::update(measuredAcc, measuredOmega, dt, &A, &B, &C);

  // Update preintegrated measurements covariance: as in [2] we consider a first
  // order propagation that can be seen as a prediction phase in an EKF
  // framework. In this implementation, in contrast to [2], we consider the
  // uncertainty of the bias selection and we keep correlation between biases
  // and preintegrated measurements

  // Single Jacobians to propagate covariance
  // TODO(frank): should we not also account for bias on position?
  Matrix3 theta_H_biasOmega = -C.topRows<3>();
  Matrix3 vel_H_biasAcc = -B.bottomRows<3>();

  // overall Jacobian wrt preintegrated measurements (df/dx)
  Eigen::Matrix<double, 15, 15> F;
  F.setZero();
  F.block<9, 9>(0, 0) = A;
  F.block<3, 3>(0, 12) = theta_H_biasOmega;
  F.block<3, 3>(6, 9) = vel_H_biasAcc;
  F.block<6, 6>(9, 9) = I_6x6;

  // propagate uncertainty
  // TODO(frank): use noiseModel routine so we can have arbitrary noise models.
  const Matrix3& aCov = p().accelerometerCovariance;
  const Matrix3& wCov = p().gyroscopeCovariance;
  const Matrix3& iCov = p().integrationCovariance;

  // first order uncertainty propagation
  // Optimized matrix multiplication   (1/dt) * G * measurementCovariance *
  // G.transpose()
  Eigen::Matrix<double, 15, 15> G_measCov_Gt;
  G_measCov_Gt.setZero(15, 15);

  // BLOCK DIAGONAL TERMS
  D_t_t(&G_measCov_Gt) = dt * iCov;
  D_v_v(&G_measCov_Gt) = (1 / dt) * vel_H_biasAcc
      * (aCov + p().biasAccOmegaInt.block<3, 3>(0, 0))
      * (vel_H_biasAcc.transpose());
  D_R_R(&G_measCov_Gt) = (1 / dt) * theta_H_biasOmega
      * (wCov + p().biasAccOmegaInt.block<3, 3>(3, 3))
      * (theta_H_biasOmega.transpose());
  D_a_a(&G_measCov_Gt) = dt * p().biasAccCovariance;
  D_g_g(&G_measCov_Gt) = dt * p().biasOmegaCovariance;

  // OFF BLOCK DIAGONAL TERMS
  Matrix3 temp = vel_H_biasAcc * p().biasAccOmegaInt.block<3, 3>(3, 0)
      * theta_H_biasOmega.transpose();
  D_v_R(&G_measCov_Gt) = temp;
  D_R_v(&G_measCov_Gt) = temp.transpose();
  preintMeasCov_ = F * preintMeasCov_ * F.transpose() + G_measCov_Gt;
}

//------------------------------------------------------------------------------
// CombinedImuFactor methods
//------------------------------------------------------------------------------
CombinedImuFactor::CombinedImuFactor(Key pose_i, Key vel_i, Key pose_j,
    Key vel_j, Key bias_i, Key bias_j,
    const PreintegratedCombinedMeasurements& pim) :
    Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose_i, vel_i,
        pose_j, vel_j, bias_i, bias_j), _PIM_(pim) {
}

//------------------------------------------------------------------------------
gtsam::NonlinearFactor::shared_ptr CombinedImuFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
void CombinedImuFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? s : s + "\n") << "CombinedImuFactor("
       << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << ","
       << keyFormatter(this->key3()) << "," << keyFormatter(this->key4()) << ","
       << keyFormatter(this->key5()) << "," << keyFormatter(this->key6())
       << ")\n";
  _PIM_.print("  preintegrated measurements:");
  this->noiseModel_->print("  noise model: ");
}

//------------------------------------------------------------------------------
bool CombinedImuFactor::equals(const NonlinearFactor& other, double tol) const {
  const This* e = dynamic_cast<const This*>(&other);
  return e != nullptr && Base::equals(*e, tol) && _PIM_.equals(e->_PIM_, tol);
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
  Vector9 r_Rpv = _PIM_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j,
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
std::ostream& operator<<(std::ostream& os, const CombinedImuFactor& f) {
  f._PIM_.print("combined preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel_->sigmas().transpose();
  return os;
}
}
 /// namespace gtsam

/// Boost serialization export definition for derived class
BOOST_CLASS_EXPORT_IMPLEMENT(gtsam::PreintegrationCombinedParams)

