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
 *  @author Frank Dellaert
 *  @date   July 2014
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

class GTSAM_EXPORT AHRSFactor: public NoiseModelFactor3<Rot3, Rot3, Vector3> {
public:

  /**
   * CombinedPreintegratedMeasurements accumulates (integrates) the Gyroscope
   * measurements (rotation rates) and the corresponding covariance matrix.
   * The measurements are then used to build the Preintegrated AHRS factor.
   * Can be built incrementally so as to avoid costly integration at time of
   * factor construction.
   */
  class GTSAM_EXPORT PreintegratedMeasurements : public PreintegratedRotation {

    friend class AHRSFactor;

  protected:
    Vector3 biasHat_; ///< Acceleration and angular rate bias values used during preintegration. Note that we won't be using the accelerometer
    Matrix3 preintMeasCov_; ///< Covariance matrix of the preintegrated measurements (first-order propagation from *measurementCovariance*)

  public:

    /// Default constructor
    PreintegratedMeasurements();

    /**
     *  Default constructor, initialize with no measurements
     *  @param bias Current estimate of acceleration and rotation rate biases
     *  @param measuredOmegaCovariance Covariance matrix of measured angular rate
     */
    PreintegratedMeasurements(const Vector3& bias,
        const Matrix3& measuredOmegaCovariance);

    Vector3 biasHat() const {
      return biasHat_;
    }
    const Matrix3& preintMeasCov() const {
      return preintMeasCov_;
    }

    /// print
    void print(const std::string& s = "Preintegrated Measurements: ") const;

    /// equals
    bool equals(const PreintegratedMeasurements&, double tol = 1e-9) const;

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

    /// Predict bias-corrected incremental rotation
    /// TODO: The matrix Hbias is the derivative of predict? Unit-test?
    Vector3 predict(const Vector3& bias, boost::optional<Matrix&> H =
        boost::none) const;

    // This function is only used for test purposes
    // (compare numerical derivatives wrt analytic ones)
    static Vector DeltaAngles(const Vector& msr_gyro_t, const double msr_dt,
        const Vector3& delta_angles);

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotation);
      ar & BOOST_SERIALIZATION_NVP(biasHat_);
    }
  };

private:
  typedef AHRSFactor This;
  typedef NoiseModelFactor3<Rot3, Rot3, Vector3> Base;

  PreintegratedMeasurements _PIM_;
  Vector3 gravity_;
  Vector3 omegaCoriolis_; ///< Controls whether higher order terms are included when calculating the Coriolis Effect
  boost::optional<Pose3> body_P_sensor_; ///< The pose of the sensor in the body frame

public:

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename boost::shared_ptr<AHRSFactor> shared_ptr;
#else
  typedef boost::shared_ptr<AHRSFactor> shared_ptr;
#endif

  /** Default constructor - only use for serialization */
  AHRSFactor();

  /**
   * Constructor
   * @param rot_i previous rot key
   * @param rot_j current rot key
   * @param bias  previous bias key
   * @param preintegratedMeasurements preintegrated measurements
   * @param omegaCoriolis rotation rate of the inertial frame
   * @param body_P_sensor Optional pose of the sensor frame in the body frame
   */
  AHRSFactor(Key rot_i, Key rot_j, Key bias,
      const PreintegratedMeasurements& preintegratedMeasurements,
      const Vector3& omegaCoriolis,
      boost::optional<const Pose3&> body_P_sensor = boost::none);

  virtual ~AHRSFactor() {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const;

  /// print
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const;

  /// equals
  virtual bool equals(const NonlinearFactor&, double tol = 1e-9) const;

  /// Access the preintegrated measurements.
  const PreintegratedMeasurements& preintegratedMeasurements() const {
    return _PIM_;
  }

  const Vector3& omegaCoriolis() const {
    return omegaCoriolis_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Rot3& rot_i, const Rot3& rot_j,
      const Vector3& bias, boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none, boost::optional<Matrix&> H3 =
          boost::none) const;

  /// predicted states from IMU
  static Rot3 predict(const Rot3& rot_i, const Vector3& bias,
      const PreintegratedMeasurements preintegratedMeasurements,
      const Vector3& omegaCoriolis,
      boost::optional<const Pose3&> body_P_sensor = boost::none);

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor3",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(_PIM_);
    ar & BOOST_SERIALIZATION_NVP(omegaCoriolis_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }

};
// AHRSFactor

typedef AHRSFactor::PreintegratedMeasurements AHRSFactorPreintegratedMeasurements;

} //namespace gtsam
