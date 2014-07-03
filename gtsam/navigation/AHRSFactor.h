/*
 * ImuFactor.h
 *
 *  Created on: Jun 29, 2014
 *      Author: krunal
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/debug.h>

#include <ostream>

namespace gtsam {

class AHRSFactor: public NoiseModelFactor3<Rot3, Rot3, imuBias::ConstantBias>{
public:
  class PreintegratedMeasurements{
  public:
    imuBias::ConstantBias biasHat;
    Matrix measurementCovariance;

    Rot3 deltaRij;
    double deltaTij;
    Matrix3 delRdelBiasOmega;
    Matrix PreintMeasCov;
    bool use2ndOrderIntegration_;


    PreintegratedMeasurements(
        const imuBias::ConstantBias& bias,
        const Matrix3& measurementAccCovariance,
        const Matrix3& measurementGyroCovariance,
        const Matrix3& integrationErrorCovariance,
        const bool use2ndOrderIntegration =false
    ): biasHat(bias), measurementCovariance(9,9), delRdelBiasOmega(Matirx3::Zero()), PreintMeasCov(9,9),
        use2ndOrderIntegration_(use2ndOrderIntegration)
    {
      measurementCovariance << integrationErrorCovariance , Matrix3::Zero(), Matrix3::Zero(),
          Matrix3::Zero(), measuredAccCovariance,  Matrix3::Zero(),
          Matrix3::Zero(),   Matrix3::Zero(), measuredOmegaCovariance;
      PreintMeasCov = Matrix::Zero(9,9);
    }

    PreintegratedMeasurements():
      biasHat(imuBias::ConstantBias()), measurementCovariance(9,9),
      delRdelBiasOmega(Matrix3::Zero()), PreintMeasCov(9,9)
    {
      measurementCovariance = Matrix::Zero(9,9);
      PreintMeasCov = Matrix::Zero(9,9);
    }

    void print (const std::string& s = "Preintegrated Measurements: ") const {
      std::cout<<s<<std::endl;
      biasHat.print(" biasHat");
      deltaRij.print(" deltaRij ");
      std::cout<<" measurementCovariance [" <<measurementCovariance<<" ]"<< std::endl;
      std::cout<<" PreintMeasCov [ "<<PreintMeasCov << " ]"<< std::endl;
    }

    bool equals (const PreintegratedMeasurements& expected, double tol = 1e-9) const {
      return biasHat.equals(expected.biasHat, tol)
          && equal_with_abs_tol(measurementCovariance, expected.measurementCovariancem, tol)
          && deltaRij.equals(expected.deltaRij, tol)
          && std::fabs(deltaTij - expected.deltaTij)<tol
          && equal_with_abs_tol(delRdelBiasOmega, expected.delRdelBiasOmega, tol);
    }

    void resetIntegration(){
      deltaRij = Rot3();
      deltaTij = 0.0;
      delRdelBiasOmega = Matrix3::Zero();
      PreintMeasCov = Matrix::Zero(9,9);
    }

    void integrateMeasurement (
        const Vector3& measuredAcc,
        const Vector3& measuredOmega,
        double deltaT,
        boost::optional<const Pose3&> body_P_sensor = boost::none
    ){
      Vector3 correctedAcc = biasHat.correctAccelerometer(measuredAcc);
      Vector3 correctedOmega = biasHat.correctGyroscope(measuredOmega);

      if(body_P_sensor) {
        Matrix3 body_R_sensor = body_P_sensor->rotation().matrix();
        correctedOmega = body_R_sensor * correctedOmega;
        Matrix3 body_omega_body_cross = skewSymmetric(correctedOmega);
        correctedAcc = body_R_sensor * correctedAcc - body_omega_body_cross * body_omega_body_cross * body_P_sensor->translation.vector();
      }
      const Vector3 theta_incr = correctedOmega * deltaT;
      const Rot3 Rincr = Rot3::Expmap(theta_incr);
      const Matrix3 Jr_theta_incr = Rot3::rightJacobianExpMapSO3(theta_incr);

      delRdelBiasOmega = Rincr.inverse().matrix() * delRdelBiasOmega - Jr_theta_incr * deltaT;

      Matrix3 Z_3x3 = Matrix::Zero();
      Matrix3 I_3x3 = Matrix::Identity();
      const Vector3 theta_i = Rot3::Logmap(deltaRij);
      const Matrix3 Jr_theta_i = Rot3::rightJacobianExpMapSO3inverse(theta_j);

      Matrix H_angles_angles = Jrinv_theta_j * Rincr.inverse().matrix() * Jr_theta_i;
      Matrix F(3,3);
      F<<H_angles_angles;

      PreintMeasCov = F* PreintMeasCov * F.transpose() + measurementCovariance * deltaT;
      deltaRij = deltaRij * Rincr;
      deltaTij += deltaT;
    }
  };

private:
  typedef AHRSFactor This;
  typedef NoiseModelFactor3<Rot3, Rot3, imuBias::ConstantBias> Base;

  PreintegratedMeasurements preintegratedMeasurements_;
  Vector3 gravity_;
  Vector3 omegaCoriolis_;
  boost::optional<Pose3> body_P_sensor_;
  bool use2ndOrderCoriolis_;

public:

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename boost::shared_ptr<ImuFactor> shared_ptr;
#else
  typedef boost::shared_ptr<ImuFactor> shared_ptr;
#endif

  /** Default constructor - only use for serialization */
  AHRSFactor() : preintegratedMeasurements_(imuBias::ConstantBias(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero()) {}

  AHRSFactor(
      Key rot_i,
      Key rot_j,
      Key bias,
      const PreintegratedMeasurements& preintegratedMeasurements,
      const Vector3& gravity,
      const Vector3& omegaCoriolis,
      boost::optional<const Pose3&> body_P_sensor = boost::none,
      const bool use2ndOrderCoriolis = false
  ):
    Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.PreintMeasCov), Rot_i, bias),
    preintegratedMeasurements_(preintegratedMeasurements),
    gravity_(gravity),
    omegaCoriolis_(omegaCoriolis),
    body_P_sensor_(body_P_sensor),
    use2ndOrderCoriolis_(use2ndOrderCoriolis){}


  virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout<<s<<"AHRSFactor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ","
        << keyFormatter(this->key3()) << ",";
    preintegratedMeasurements_.print("  preintegrated measurements:");
    std::cout << "  gravity: [ " << gravity_.transpose() << " ]" << std::endl;
    std::cout << "  omegaCoriolis: [ " << omegaCoriolis_.transpose() << " ]" << std::endl;
    this->noiseModel_->print("  noise model: ");
    if(this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
  }

  virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol)
    && preintegratedMeasurements_.equals(e->preintegratedMeasurements_, tol)
    && equal_with_abs_tol(gravity_, e->gravity_, tol)
    && equal_with_abs_tol(omegaCoriolis_, e->omegaCoriolis_, tol)
    && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
  }
  /** Access the preintegrated measurements. */
  const PreintegratedMeasurements& preintegratedMeasurements() const {
    return preintegratedMeasurements_; }

  const Vector3& gravity() const { return gravity_; }

  const Vector3& omegaCoriolis() const { return omegaCoriolis_; }


  Vector evaluateError(const Rot3& rot_i, const Rot3& rot_j, const imuBias::ConstantBias& bias,
      boost::optional<Matrix&> H1 = boost::none) const
  {
    const double& deltaTij = preintegratedMeasurements_.deltaTij;
    const Vector3 biasAccIncr = bias.accelerometer() - preintegratedMeasurements_.biasHat.accelerometer();
    const Vector3 biasOmegaIncr = bias.gyroscope() - preintegratedMeasurements_.biasHat.gyroscope();

    // we give some shorter name to rotations and translations
    const Rot3 Rot_i = pose_i.rotation();
    const Rot3 Rot_j = pose_j.rotation();
    // We compute factor's Jacobians
    /* ---------------------------------------------------------------------------------------------------- */
    const Rot3 deltaRij_biascorrected = preintegratedMeasurements_.deltaRij.retract(preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr, Rot3::EXPMAP);
    // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)

    Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);

    Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected  -
        Rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij; // Coriolis term

    const Rot3 deltaRij_biascorrected_corioliscorrected =
        Rot3::Expmap( theta_biascorrected_corioliscorrected );

    const Rot3 fRhat = deltaRij_biascorrected_corioliscorrected.between(Rot_i.between(Rot_j));

    const Matrix3 Jr_theta_bcc = Rot3::rightJacobianExpMapSO3(theta_biascorrected_corioliscorrected);

    const Matrix3 Jtheta = -Jr_theta_bcc  * skewSymmetric(Rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij);

    const Matrix3 Jrinv_fRhat = Rot3::rightJacobianExpMapSO3inverse(Rot3::Logmap(fRhat));

    if(H1) {
      H1->resize(3,3);
      (*H1)<<// dfR/dRi
          Jrinv_fRhat *  (- Rot_j.between(Rot_i).matrix() - fRhat.inverse().matrix() * Jtheta);
    }

    if(H2) {

      const Matrix3 Jrinv_theta_bc = Rot3::rightJacobianExpMapSO3inverse(theta_biascorrected);
      const Matrix3 Jr_JbiasOmegaIncr = Rot3::rightJacobianExpMapSO3(preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr);
      const Matrix3 JbiasOmega = Jr_theta_bcc * Jrinv_theta_bc * Jr_JbiasOmegaIncr * preintegratedMeasurements_.delRdelBiasOmega;

      H5->resize(3,3);
      (*H5) <<
          // dfR/dBias
          Jrinv_fRhat * ( - fRhat.inverse().matrix() * JbiasOmega);
    }


    const Vector3 fR = Rot3::Logmap(fRhat);

    Vector r(3); r << fR;
    return r;
  }
}; // AHRSFactor
typedef AHRSFactor::PreintegratedMeasurements AHRSFactorPreintegratedMeasurements;
} //namespace gtsam
