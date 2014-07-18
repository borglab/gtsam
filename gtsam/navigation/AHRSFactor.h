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

class AHRSFactor: public NoiseModelFactor3<Rot3, Rot3, imuBias::ConstantBias> {
public:
  class PreintegratedMeasurements {
  public:
    imuBias::ConstantBias biasHat;
    Matrix measurementCovariance;

    Rot3 deltaRij;
    double deltaTij;
    Matrix3 delRdelBiasOmega;
    Matrix PreintMeasCov;

    PreintegratedMeasurements(const imuBias::ConstantBias& bias,
        const Matrix3& measuredOmegaCovariance) :
          biasHat(bias), measurementCovariance(3,3), delRdelBiasOmega(
              Matrix3::Zero()), PreintMeasCov(3,3) {
//      measurementCovariance << integrationErrorCovariance, Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), measurementAccCovariance, Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), measuredOmegaCovariance;
      measurementCovariance <<measuredOmegaCovariance;
      PreintMeasCov = Matrix::Zero(3,3);
    }

    PreintegratedMeasurements() :
      biasHat(imuBias::ConstantBias()), measurementCovariance(3,3), delRdelBiasOmega(
          Matrix3::Zero()), PreintMeasCov(3,3) {
      measurementCovariance = Matrix::Zero(3,3);
      PreintMeasCov = Matrix::Zero(3,3);
    }

    void print(const std::string& s = "Preintegrated Measurements: ") const {
      std::cout << s << std::endl;
      biasHat.print(" biasHat");
      deltaRij.print(" deltaRij ");
      std::cout << " measurementCovariance [" << measurementCovariance << " ]"
          << std::endl;
      std::cout << " PreintMeasCov [ " << PreintMeasCov << " ]" << std::endl;
    }

    bool equals(const PreintegratedMeasurements& expected,
        double tol = 1e-9) const {
      return biasHat.equals(expected.biasHat, tol)
          && equal_with_abs_tol(measurementCovariance,
              expected.measurementCovariance, tol)
              && deltaRij.equals(expected.deltaRij, tol)
              && std::fabs(deltaTij - expected.deltaTij) < tol
              && equal_with_abs_tol(delRdelBiasOmega, expected.delRdelBiasOmega,
                  tol);
    }
    Matrix MeasurementCovariance(){
      return measurementCovariance;
    }

    void resetIntegration() {
      deltaRij = Rot3();
      deltaTij = 0.0;
      delRdelBiasOmega = Matrix3::Zero();
      PreintMeasCov = Matrix::Zero(9, 9);
    }

    void integrateMeasurement(
        const Vector3& measuredOmega, double deltaT,
        boost::optional<const Pose3&> body_P_sensor = boost::none) {
      Vector3 correctedOmega = biasHat.correctGyroscope(measuredOmega);

      if (body_P_sensor) {
        Matrix3 body_R_sensor = body_P_sensor->rotation().matrix();
        correctedOmega = body_R_sensor * correctedOmega;
        Matrix3 body_omega_body_cross = skewSymmetric(correctedOmega);
      }
      const Vector3 theta_incr = correctedOmega * deltaT;
      const Rot3 Rincr = Rot3::Expmap(theta_incr);
      const Matrix3 Jr_theta_incr = Rot3::rightJacobianExpMapSO3(theta_incr);

      delRdelBiasOmega = Rincr.inverse().matrix() * delRdelBiasOmega
          - Jr_theta_incr * deltaT;

      //      Matrix3 Z_3x3 = Matrix::Zero();
      //      Matrix3 I_3x3 = Matrix::Identity();
      const Vector3 theta_i = Rot3::Logmap(deltaRij);
      const Matrix3 Jr_theta_i = Rot3::rightJacobianExpMapSO3inverse(theta_i);

      Rot3 Rot_j = deltaRij * Rincr;
      const Vector3 theta_j = Rot3::Logmap(Rot_j);
      const Matrix3 Jrinv_theta_j = Rot3::rightJacobianExpMapSO3inverse(
          theta_j);

      Matrix H_angles_angles = Jrinv_theta_j * Rincr.inverse().matrix()
                  * Jr_theta_i;
      Matrix F(3, 3);
      F << H_angles_angles;

      PreintMeasCov = F * PreintMeasCov * F.transpose()
                  + measurementCovariance * deltaT;
      deltaRij = deltaRij * Rincr;
      deltaTij += deltaT;
    }
    /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
    // This function is only used for test purposes (compare numerical derivatives wrt analytic ones)
    static inline Vector PreIntegrateIMUObservations_delta_angles(const Vector& msr_gyro_t, const double msr_dt,
        const Vector3& delta_angles){

      // Note: all delta terms refer to an IMU\sensor system at t0

      // Calculate the corrected measurements using the Bias object
      Vector body_t_omega_body= msr_gyro_t;

      Rot3 R_t_to_t0 = Rot3::Expmap(delta_angles);

      R_t_to_t0    = R_t_to_t0 * Rot3::Expmap( body_t_omega_body*msr_dt );
      return Rot3::Logmap(R_t_to_t0);
    }
    /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(biasHat);
      ar & BOOST_SERIALIZATION_NVP(measurementCovariance);
      ar & BOOST_SERIALIZATION_NVP(deltaRij);
      ar & BOOST_SERIALIZATION_NVP(deltaTij);
      ar & BOOST_SERIALIZATION_NVP(delRdelBiasOmega);
    }
  };

private:
  typedef AHRSFactor This;
  typedef NoiseModelFactor3<Rot3, Rot3, imuBias::ConstantBias> Base;

  PreintegratedMeasurements preintegratedMeasurements_;
  Vector3 gravity_;
  Vector3 omegaCoriolis_;
  boost::optional<Pose3> body_P_sensor_;

public:

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename boost::shared_ptr<AHRSFactor> shared_ptr;
#else
  typedef boost::shared_ptr<AHRSFactor> shared_ptr;
#endif

  /** Default constructor - only use for serialization */
  AHRSFactor() :
    preintegratedMeasurements_(imuBias::ConstantBias(), Matrix3::Zero()) {}

  AHRSFactor(Key rot_i, Key rot_j, Key bias,
      const PreintegratedMeasurements& preintegratedMeasurements,
      const Vector3& omegaCoriolis,
      boost::optional<const Pose3&> body_P_sensor = boost::none) :
        Base(
            noiseModel::Gaussian::Covariance(
                preintegratedMeasurements.PreintMeasCov), rot_i, rot_j, bias), preintegratedMeasurements_(
                    preintegratedMeasurements), omegaCoriolis_(
                        omegaCoriolis), body_P_sensor_(body_P_sensor) {
  }

  virtual ~AHRSFactor() {}


  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(
            new This(*this)
            )
        );
    }

  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "AHRSFactor(" << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << "," << keyFormatter(this->key3())
        << ",";
    preintegratedMeasurements_.print("  preintegrated measurements:");
    std::cout << "  omegaCoriolis: [ " << omegaCoriolis_.transpose() << " ]"
        << std::endl;
    this->noiseModel_->print("  noise model: ");
    if (this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
  }

  virtual bool equals(const NonlinearFactor& expected,
      double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol)
    && preintegratedMeasurements_.equals(e->preintegratedMeasurements_, tol)
    && equal_with_abs_tol(omegaCoriolis_, e->omegaCoriolis_, tol)
    && ((!body_P_sensor_ && !e->body_P_sensor_)
        || (body_P_sensor_ && e->body_P_sensor_
            && body_P_sensor_->equals(*e->body_P_sensor_)));
  }
  /** Access the preintegrated measurements. */
  const PreintegratedMeasurements& preintegratedMeasurements() const {
    return preintegratedMeasurements_;
  }


  const Vector3& omegaCoriolis() const {
    return omegaCoriolis_;
  }

  Vector evaluateError(const Rot3& rot_i, const Rot3& rot_j,
      const imuBias::ConstantBias& bias,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const
  {

    const double& deltaTij = preintegratedMeasurements_.deltaTij;
//    const Vector3 biasAccIncr = bias.accelerometer()
                - preintegratedMeasurements_.biasHat.accelerometer();
    const Vector3 biasOmegaIncr = bias.gyroscope()
                - preintegratedMeasurements_.biasHat.gyroscope();

    // we give some shorter name to rotations and translations
    const Rot3 Rot_i = rot_i;
    const Rot3 Rot_j = rot_j;
    // We compute factor's Jacobians
    /* ---------------------------------------------------------------------------------------------------- */
    const Rot3 deltaRij_biascorrected =
        preintegratedMeasurements_.deltaRij.retract(
            preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr,
            Rot3::EXPMAP);
    // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)

    Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);

    Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected
        - Rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij; // Coriolis term

    const Rot3 deltaRij_biascorrected_corioliscorrected = Rot3::Expmap(
        theta_biascorrected_corioliscorrected);

    const Rot3 fRhat = deltaRij_biascorrected_corioliscorrected.between(
        Rot_i.between(Rot_j));

    const Matrix3 Jr_theta_bcc = Rot3::rightJacobianExpMapSO3(
        theta_biascorrected_corioliscorrected);

    const Matrix3 Jtheta = -Jr_theta_bcc
        * skewSymmetric(Rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij);

    const Matrix3 Jrinv_fRhat = Rot3::rightJacobianExpMapSO3inverse(
        Rot3::Logmap(fRhat));

    if (H1) {
      H1->resize(3, 3);
      (*H1) << // dfR/dRi
          Jrinv_fRhat
          * (-Rot_j.between(Rot_i).matrix()
              - fRhat.inverse().matrix() * Jtheta);
    }
    if(H2) {

      H2->resize(3,3);
      (*H2) <<
          // dfR/dPosej
          Jrinv_fRhat *  ( Matrix3::Identity() );
    }

    if (H3) {

      const Matrix3 Jrinv_theta_bc = Rot3::rightJacobianExpMapSO3inverse(
          theta_biascorrected);
      const Matrix3 Jr_JbiasOmegaIncr = Rot3::rightJacobianExpMapSO3(
          preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr);
      const Matrix3 JbiasOmega = Jr_theta_bcc * Jrinv_theta_bc
          * Jr_JbiasOmegaIncr * preintegratedMeasurements_.delRdelBiasOmega;

      H3->resize(3, 6);
      (*H3) <<
          // dfR/dBias
          Matrix::Zero(3,3),
          Jrinv_fRhat * (-fRhat.inverse().matrix() * JbiasOmega);
    }

    const Vector3 fR = Rot3::Logmap(fRhat);

    Vector r(3);
    r << fR;
    return r;
  }

  /** predicted states from IMU */
  static void Predict(const Rot3& rot_i, const Rot3& rot_j,
      const imuBias::ConstantBias& bias,
      const PreintegratedMeasurements preintegratedMeasurements,
      const Vector3& omegaCoriolis,
      boost::optional<const Pose3&> body_P_sensor = boost::none
      ) {

    const double& deltaTij = preintegratedMeasurements.deltaTij;
//    const Vector3 biasAccIncr = bias.accelerometer()
                - preintegratedMeasurements.biasHat.accelerometer();
    const Vector3 biasOmegaIncr = bias.gyroscope()
                - preintegratedMeasurements.biasHat.gyroscope();

    const Rot3 Rot_i = rot_i;

    // Predict state at time j
    /* ---------------------------------------------------------------------------------------------------- */

    const Rot3 deltaRij_biascorrected =
        preintegratedMeasurements.deltaRij.retract(
            preintegratedMeasurements.delRdelBiasOmega * biasOmegaIncr,
            Rot3::EXPMAP);
    // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)
    Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);
    Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected
        - Rot_i.inverse().matrix() * omegaCoriolis * deltaTij; // Coriolis term
    const Rot3 deltaRij_biascorrected_corioliscorrected = Rot3::Expmap(
        theta_biascorrected_corioliscorrected);
    const Rot3 Rot_j = Rot_i.compose(deltaRij_biascorrected_corioliscorrected);

  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar
    & boost::serialization::make_nvp("NoiseModelFactor3",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(preintegratedMeasurements_);
    ar & BOOST_SERIALIZATION_NVP(omegaCoriolis_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
};
// AHRSFactor
typedef AHRSFactor::PreintegratedMeasurements AHRSFactorPreintegratedMeasurements;
} //namespace gtsam
