/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  AHRSFactor.h
 *  @author Krunal Chande
 *  @author Luca Carlone
 *  @date   July 2014
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/ImuBias.h>

namespace gtsam {

class AHRSFactor: public NoiseModelFactor3<Rot3, Rot3, imuBias::ConstantBias> {
public:

  /**
   * CombinedPreintegratedMeasurements accumulates (integrates) the Gyroscope
   * measurements (rotation rates) and the corresponding covariance matrix.
   * The measurements are then used to build the Preintegrated AHRS factor.
   * Can be built incrementally so as to avoid costly integration at time of
   * factor construction.
   */
  class PreintegratedMeasurements {

    friend class AHRSFactor;

  protected:
    imuBias::ConstantBias biasHat_;///< Acceleration and angular rate bias values used during preintegration. Note that we won't be using the accelerometer
    Matrix3 measurementCovariance_;///< (Raw measurements uncertainty) Covariance of the vector [measuredOmega] in R^(3X3)

    Rot3 deltaRij_; ///< Preintegrated relative orientation (in frame i)
    double deltaTij_; ///< Time interval from i to j
    Matrix3 delRdelBiasOmega_; ///< Jacobian of preintegrated rotation w.r.t. angular rate bias
    Matrix3 preintMeasCov_; ///< Covariance matrix of the preintegrated measurements (first-order propagation from *measurementCovariance*)

  public:

    /// Default constructor
    PreintegratedMeasurements();

    /**
     *  Default constructor, initialize with no measurements
     *  @param bias Current estimate of acceleration and rotation rate biases
     *  @param measuredOmegaCovariance Covariance matrix of measured angular rate
     */
    PreintegratedMeasurements(const imuBias::ConstantBias& bias,
        const Matrix3& measuredOmegaCovariance);

    /// print
    void print(const std::string& s = "Preintegrated Measurements: ") const;

    /// equals
    bool equals(const PreintegratedMeasurements&, double tol = 1e-9) const;

    const Matrix3& measurementCovariance() const {return measurementCovariance_;}
    Matrix3        deltaRij()              const {return deltaRij_.matrix();}
    double         deltaTij()              const {return deltaTij_;}
    Vector6        biasHat()               const {return biasHat_.vector();}
    const Matrix3& delRdelBiasOmega()      const {return delRdelBiasOmega_;}
    const Matrix3& preintMeasCov()         const {return preintMeasCov_;}

    /// TODO: Document
    void resetIntegration();

    /**
     * Add a single Gyroscope measurement to the preintegration.
     * @param measureOmedga Measured angular velocity (in body frame)
     * @param deltaT Time step
     * @param body_P_sensor Optional sensor frame
     */
    void integrateMeasurement(const Vector3& measuredOmega, double deltaT,
        boost::optional<const Pose3&> body_P_sensor = boost::none);

    // This function is only used for test purposes
    // (compare numerical derivatives wrt analytic ones)
    static Vector PreIntegrateIMUObservations_delta_angles(
        const Vector& msr_gyro_t, const double msr_dt,
        const Vector3& delta_angles);

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(biasHat_);
      ar & BOOST_SERIALIZATION_NVP(measurementCovariance_);
      ar & BOOST_SERIALIZATION_NVP(deltaRij_);
      ar & BOOST_SERIALIZATION_NVP(deltaTij_);
      ar & BOOST_SERIALIZATION_NVP(delRdelBiasOmega_);
    }
  };

private:
  typedef AHRSFactor This;
  typedef NoiseModelFactor3<Rot3, Rot3, imuBias::ConstantBias> Base;

  PreintegratedMeasurements preintegratedMeasurements_;
  Vector3 gravity_;
  Vector3 omegaCoriolis_; ///< Controls whether higher order terms are included when calculating the Coriolis Effect
  boost::optional<Pose3> body_P_sensor_;///< The pose of the sensor in the body frame

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

  AHRSFactor(
      Key rot_i, ///< previous rot key
      Key rot_j,  ///< current rot key
      Key bias,///< previous bias key
      const PreintegratedMeasurements& preintegratedMeasurements, ///< preintegrated measurements
      const Vector3& omegaCoriolis, ///< rotation rate of the inertial frame
      boost::optional<const Pose3&> body_P_sensor = boost::none  ///< The Pose of the sensor frame in the body frame
      ) :
      Base(
          noiseModel::Gaussian::Covariance(
              preintegratedMeasurements.preintMeasCov_), rot_i, rot_j, bias), preintegratedMeasurements_(
          preintegratedMeasurements), omegaCoriolis_(omegaCoriolis), body_P_sensor_(
          body_P_sensor) {
  }

  virtual ~AHRSFactor() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(
            new This(*this)
            )
        );
    }

  /** implement functions needed for Testable */

  /** print */
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

  /** equals */
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

  /** implement functions needed to derive from Factor */

  /** vector of errors */
  Vector evaluateError(const Rot3& rot_i, const Rot3& rot_j,
      const imuBias::ConstantBias& bias,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const
  {

    double deltaTij = preintegratedMeasurements_.deltaTij_;

    Vector3 biasOmegaIncr = bias.gyroscope()
                - preintegratedMeasurements_.biasHat_.gyroscope();

    // We compute factor's Jacobians
    /* ---------------------------------------------------------------------------------------------------- */
    Rot3 deltaRij_biascorrected =
        preintegratedMeasurements_.deltaRij_.retract(
            preintegratedMeasurements_.delRdelBiasOmega_ * biasOmegaIncr,
            Rot3::EXPMAP);

    Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);

    Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected
        - rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij; // Coriolis term

    Rot3 deltaRij_biascorrected_corioliscorrected = Rot3::Expmap(
        theta_biascorrected_corioliscorrected);

    Rot3 fRhat = deltaRij_biascorrected_corioliscorrected.between(
        rot_i.between(rot_j));

    Matrix3 Jr_theta_bcc = Rot3::rightJacobianExpMapSO3(
        theta_biascorrected_corioliscorrected);

    Matrix3 Jtheta = -Jr_theta_bcc
        * skewSymmetric(rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij);

    Matrix3 Jrinv_fRhat = Rot3::rightJacobianExpMapSO3inverse(
        Rot3::Logmap(fRhat));

    if (H1) {
      H1->resize(3, 3);
      (*H1) << // dfR/dRi
          Jrinv_fRhat
          * (-rot_j.between(rot_i).matrix()
              - fRhat.inverse().matrix() * Jtheta);
    }
    if(H2) {

      H2->resize(3,3);
      (*H2) <<
          // dfR/dPosej
          Jrinv_fRhat *  ( Matrix3::Identity() );
    }

    if (H3) {

      Matrix3 Jrinv_theta_bc = Rot3::rightJacobianExpMapSO3inverse(
          theta_biascorrected);
      Matrix3 Jr_JbiasOmegaIncr = Rot3::rightJacobianExpMapSO3(
          preintegratedMeasurements_.delRdelBiasOmega_ * biasOmegaIncr);
      Matrix3 JbiasOmega = Jr_theta_bcc * Jrinv_theta_bc
          * Jr_JbiasOmegaIncr * preintegratedMeasurements_.delRdelBiasOmega_;

      H3->resize(3, 6);
      (*H3) <<
          // dfR/dBias
          Matrix::Zero(3,3),
          Jrinv_fRhat * (-fRhat.inverse().matrix() * JbiasOmega);
    }

    Vector3 fR = Rot3::Logmap(fRhat);

    Vector r(3);
    r << fR;
    return r;
  }

  /** predicted states from IMU */
  static Rot3 predict(const Rot3& rot_i,
      const imuBias::ConstantBias& bias,
      const PreintegratedMeasurements preintegratedMeasurements,
      const Vector3& omegaCoriolis,
      boost::optional<const Pose3&> body_P_sensor = boost::none
      ) {

    const double& deltaTij = preintegratedMeasurements.deltaTij_;
//    const Vector3 biasAccIncr = bias.accelerometer()
                - preintegratedMeasurements.biasHat_.accelerometer();
    const Vector3 biasOmegaIncr = bias.gyroscope()
                - preintegratedMeasurements.biasHat_.gyroscope();

    const Rot3 Rot_i = rot_i;

    // Predict state at time j
    /* ---------------------------------------------------------------------------------------------------- */
    const Rot3 deltaRij_biascorrected =
        preintegratedMeasurements.deltaRij_.retract(
            preintegratedMeasurements.delRdelBiasOmega_ * biasOmegaIncr,
            Rot3::EXPMAP);
    // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)
    Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);
    Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected
        - Rot_i.inverse().matrix() * omegaCoriolis * deltaTij; // Coriolis term
    const Rot3 deltaRij_biascorrected_corioliscorrected = Rot3::Expmap(
        theta_biascorrected_corioliscorrected);
//    const Rot3 Rot_j =
    return (Rot_i.compose(deltaRij_biascorrected_corioliscorrected));

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
}; // AHRSFactor
typedef AHRSFactor::PreintegratedMeasurements AHRSFactorPreintegratedMeasurements;
} //namespace gtsam
