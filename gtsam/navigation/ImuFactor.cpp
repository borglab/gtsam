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
//------------------------------------------------------------------------------
void PreintegratedImuMeasurements::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double deltaT,
    OptionalJacobian<9, 9> F_test, OptionalJacobian<9, 9> G_test) {

  const Matrix3 dRij = deltaRij_.matrix(); // store this, which is useful to compute G_test

  // Update preintegrated measurements (also get Jacobian)
  Matrix3 D_incrR_integratedOmega; // Right jacobian computed at theta_incr
  Matrix9 F; // overall Jacobian wrt preintegrated measurements (df/dx)
  updatePreintegratedMeasurements(measuredAcc, measuredOmega, deltaT,
      &D_incrR_integratedOmega, &F);

  // first order covariance propagation:
  // as in [2] we consider a first order propagation that can be seen as a prediction phase in EKF
  /* --------------------------------------------------------------------------------------------*/
  // preintMeasCov = F * preintMeasCov * F.transpose() + G * (1/deltaT) * measurementCovariance * G'
  // NOTE 1: (1/deltaT) allows to pass from continuous time noise to discrete time noise
  // measurementCovariance_discrete = measurementCovariance_contTime * (1/deltaT)
  // NOTE 2: computation of G * (1/deltaT) * measurementCovariance * G.transpose() done block-wise,
  // as G and measurementCovariance are block-diagonal matrices
  preintMeasCov_ = F * preintMeasCov_ * F.transpose();
  D_t_t(&preintMeasCov_) += p().integrationCovariance * deltaT;
  D_v_v(&preintMeasCov_) += dRij * p().accelerometerCovariance * dRij.transpose() * deltaT;
  D_R_R(&preintMeasCov_) += D_incrR_integratedOmega * p().gyroscopeCovariance
      * D_incrR_integratedOmega.transpose() * deltaT;

  Matrix3 F_pos_noiseacc;
  F_pos_noiseacc = 0.5 * dRij * deltaT * deltaT;
  D_t_t(&preintMeasCov_) += (1 / deltaT) * F_pos_noiseacc
      * p().accelerometerCovariance * F_pos_noiseacc.transpose();
  Matrix3 temp = F_pos_noiseacc * p().accelerometerCovariance
      * dRij.transpose(); // has 1/deltaT
  D_t_v(&preintMeasCov_) += temp;
  D_v_t(&preintMeasCov_) += temp.transpose();

  // F_test and G_test are given as output for testing purposes and are not needed by the factor
  if (F_test) {
    (*F_test) << F;
  }
  if (G_test) {
    // This in only for testing & documentation, while the actual computation is done block-wise
    //           omegaNoise               intNoise        accNoise
    (*G_test) <<
        D_incrR_integratedOmega * deltaT, Z_3x3,          Z_3x3, // angle
        Z_3x3,                            I_3x3 * deltaT, F_pos_noiseacc, // pos
        Z_3x3,                            Z_3x3,          dRij * deltaT; // vel
  }
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
    const Pose3& body_P_sensor) {
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
