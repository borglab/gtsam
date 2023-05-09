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

/**
 * PreintegratedAHRSMeasurements accumulates (integrates) the Gyroscope
 * measurements (rotation rates) and the corresponding covariance matrix.
 * Can be built incrementally so as to avoid costly integration at time of factor construction.
 */
class GTSAM_EXPORT PreintegratedAhrsMeasurements : public PreintegratedRotation {

 protected:

  Vector3 biasHat_; ///< Angular rate bias values used during preintegration.
  Matrix3 preintMeasCov_; ///< Covariance matrix of the preintegrated measurements (first-order propagation from *measurementCovariance*)

  friend class AHRSFactor;

 public:

  /// Default constructor, only for serialization and wrappers
  PreintegratedAhrsMeasurements() {}

  /**
   *  Default constructor, initialize with no measurements
   *  @param bias Current estimate of acceleration and rotation rate biases
   */
  PreintegratedAhrsMeasurements(const boost::shared_ptr<Params>& p,
      const Vector3& biasHat) :
      PreintegratedRotation(p), biasHat_(biasHat) {
    resetIntegration();
  }

  /**
   *  Non-Default constructor, initialize with measurements
   *  @param p: Parameters for AHRS pre-integration
   *  @param bias_hat: Current estimate of acceleration and rotation rate biases
   *  @param deltaTij: Delta time in pre-integration
   *  @param deltaRij: Delta rotation in pre-integration
   *  @param delRdelBiasOmega: Jacobian of rotation wrt. to gyro bias
   *  @param preint_meas_cov: Pre-integration covariance
   */
  PreintegratedAhrsMeasurements(
      const boost::shared_ptr<Params>& p,
      const Vector3& bias_hat,
      double deltaTij,
      const Rot3& deltaRij,
      const Matrix3& delRdelBiasOmega,
      const Matrix3& preint_meas_cov) :
      PreintegratedRotation(p, deltaTij, deltaRij, delRdelBiasOmega),
      biasHat_(bias_hat),
      preintMeasCov_(preint_meas_cov) {}

  Params& p() const { return *boost::static_pointer_cast<Params>(p_);}
  const Vector3& biasHat() const { return biasHat_; }
  const Matrix3& preintMeasCov() const { return preintMeasCov_; }

  /// print
  void print(const std::string& s = "Preintegrated Measurements: ") const;

  /// equals
  bool equals(const PreintegratedAhrsMeasurements&, double tol = 1e-9) const;

  /// Reset inetgrated quantities to zero
  void resetIntegration();

  /**
   * Add a single Gyroscope measurement to the preintegration.
   * @param measureOmedga Measured angular velocity (in body frame)
   * @param deltaT Time step
   */
  void integrateMeasurement(const Vector3& measuredOmega, double deltaT);

  /// Predict bias-corrected incremental rotation
  /// TODO: The matrix Hbias is the derivative of predict? Unit-test?
  Vector3 predict(const Vector3& bias, OptionalJacobian<3,3> H = boost::none) const;

  // This function is only used for test purposes
  // (compare numerical derivatives wrt analytic ones)
  static Vector DeltaAngles(const Vector& msr_gyro_t, const double msr_dt,
      const Vector3& delta_angles);

  /// @deprecated constructor
  PreintegratedAhrsMeasurements(const Vector3& biasHat,
                                const Matrix3& measuredOmegaCovariance)
      : PreintegratedRotation(boost::make_shared<Params>()),
        biasHat_(biasHat) {
    p_->gyroscopeCovariance = measuredOmegaCovariance;
    resetIntegration();
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotation);
    ar & BOOST_SERIALIZATION_NVP(p_);
    ar & BOOST_SERIALIZATION_NVP(biasHat_);
  }
};

class GTSAM_EXPORT AHRSFactor: public NoiseModelFactor3<Rot3, Rot3, Vector3> {

  typedef AHRSFactor This;
  typedef NoiseModelFactor3<Rot3, Rot3, Vector3> Base;

  PreintegratedAhrsMeasurements _PIM_;

  /** Default constructor - only use for serialization */
  AHRSFactor() {}

public:

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename boost::shared_ptr<AHRSFactor> shared_ptr;
#else
  typedef boost::shared_ptr<AHRSFactor> shared_ptr;
#endif

  /**
   * Constructor
   * @param rot_i previous rot key
   * @param rot_j current rot key
   * @param bias  previous bias key
   * @param preintegratedMeasurements preintegrated measurements
   */
  AHRSFactor(Key rot_i, Key rot_j, Key bias,
      const PreintegratedAhrsMeasurements& preintegratedMeasurements);

  ~AHRSFactor() override {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  /// print
  void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor&, double tol = 1e-9) const override;

  /// Access the preintegrated measurements.
  const PreintegratedAhrsMeasurements& preintegratedMeasurements() const {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Rot3& rot_i, const Rot3& rot_j,
      const Vector3& bias, boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none, boost::optional<Matrix&> H3 =
          boost::none) const override;

  /// predicted states from IMU
  /// TODO(frank): relationship with PIM predict ??
  static Rot3 Predict(
      const Rot3& rot_i, const Vector3& bias,
      const PreintegratedAhrsMeasurements preintegratedMeasurements);

  /// @deprecated name
  typedef PreintegratedAhrsMeasurements PreintegratedMeasurements;

  /// @deprecated constructor
  AHRSFactor(Key rot_i, Key rot_j, Key bias,
      const PreintegratedMeasurements& preintegratedMeasurements,
      const Vector3& omegaCoriolis,
      const boost::optional<Pose3>& body_P_sensor = boost::none);

  /// @deprecated static function
  static Rot3 predict(const Rot3& rot_i, const Vector3& bias,
      const PreintegratedMeasurements preintegratedMeasurements,
      const Vector3& omegaCoriolis,
      const boost::optional<Pose3>& body_P_sensor = boost::none);

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor3",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(_PIM_);
  }

};
// AHRSFactor

} //namespace gtsam
