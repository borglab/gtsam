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
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <optional>

namespace gtsam {

/**
 * PreintegratedAHRSMeasurements accumulates (integrates) the gyroscope
 * measurements (rotation rates) and the corresponding covariance matrix.
 * Can be built incrementally so as to avoid costly integration at time of
 * factor construction.
 *
 * @section math_notes Mathematical Formulation
 *
 * The preintegrated rotation is updated incrementally with each gyroscope
 * measurement. Given a gyroscope measurement \f$ \omega_k \f$ at time \f$ t_k \f$,
 * the preintegrated rotation \f$ \Delta R_{ij} \f$ from time \f$ t_i \f$ to \f$ t_j \f$
 * is the product of many small rotations:
 * \f[
 * \Delta R_{ij} = \prod_{k=i}^{j-1} \text{Exp}((\omega_k - b_g) \Delta t)
 * \f]
 * where \f$ b_g \f$ is the gyroscope bias, and \f$ \text{Exp}(\cdot) \f$ is the
 * exponential map from \f$ \mathbb{R}^3 \f$ to SO(3).
 *
 * This class also propagates the covariance of the preintegrated rotation.
 */
class GTSAM_EXPORT PreintegratedAhrsMeasurements
    : public PreintegratedRotation {
 protected:
  Vector3 biasHat_;  ///< Angular rate bias values used during preintegration.
  Matrix3 preintMeasCov_;  ///< Covariance matrix of the preintegrated
                           ///< measurements (first-order propagation from
                           ///< *measurementCovariance*)

  friend class AHRSFactor;

 public:
  /// Default constructor, only for serialization and wrappers
  PreintegratedAhrsMeasurements() {}

  /**
   *  Default constructor, initialize with no measurements
   *  @param bias Current estimate of rotation rate biases
   */
  PreintegratedAhrsMeasurements(const std::shared_ptr<Params>& p,
                                const Vector3& biasHat)
      : PreintegratedRotation(p), biasHat_(biasHat) {
    resetIntegration();
  }

  /**
   *  Non-Default constructor, initialize with measurements
   *  @param p: Parameters for AHRS pre-integration
   *  @param bias_hat: Current estimate of rotation rate biases
   *  @param deltaTij: Delta time in pre-integration
   *  @param deltaRij: Delta rotation in pre-integration
   *  @param delRdelBiasOmega: Jacobian of rotation wrt. to gyro bias
   *  @param preint_meas_cov: Pre-integration covariance
   */
  PreintegratedAhrsMeasurements(const std::shared_ptr<Params>& p,
                                const Vector3& bias_hat, double deltaTij,
                                const Rot3& deltaRij,
                                const Matrix3& delRdelBiasOmega,
                                const Matrix3& preint_meas_cov)
      : PreintegratedRotation(p, deltaTij, deltaRij, delRdelBiasOmega),
        biasHat_(bias_hat),
        preintMeasCov_(preint_meas_cov) {}

  Params& p() const { return *std::static_pointer_cast<Params>(p_); }
  const Vector3& biasHat() const { return biasHat_; }
  const Matrix3& preintMeasCov() const { return preintMeasCov_; }

  /// print
  void print(const std::string& s = "Preintegrated Measurements: ") const;

  /// equals
  bool equals(const PreintegratedAhrsMeasurements& expected,
              double tol = 1e-9) const;

  /// Reset integrated quantities to zero
  void resetIntegration();

  /**
   * Add a single gyroscope measurement to the preintegration.
   * Measurements are taken to be in the sensor
   * frame and conversion to the body frame is handled by `body_P_sensor` in
   * `PreintegratedRotationParams` (if provided).
   *
   * @param measuredOmega Measured angular velocity (as given by the sensor)
   * @param deltaT Time step
   */
  void integrateMeasurement(const Vector3& measuredOmega, double deltaT);

  /**
   * Predict the orientation at time j, given orientation and bias at time i.
   * @param Ri orientation at time i
   * @param bias gyroscope bias
   * @param H1 optional 3x3 Jacobian wrt Ri
   * @param H2 optional 3x3 Jacobian wrt bias
   * @return predicted orientation at time j
   */
  Rot3 predict(const Rot3& Ri, const Vector3& bias,
               gtsam::OptionalJacobian<3, 3> H1 = {},
               gtsam::OptionalJacobian<3, 3> H2 = {}) const;

  /**
   * Calculate the error between the predicted and actual rotation.
   * @param Ri The orientation at time i
   * @param Rj The orientation at time j
   * @param bias The gyroscope bias
   * @param H1 Optional Jacobian of the error with respect to Ri
   * @param H2 Optional Jacobian of the error with respect to Rj
   * @param H3 Optional Jacobian of the error with respect to bias
   * @return A 3D vector containing the rotation error.
   */
  Vector3 computeError(const Rot3& Ri, const Rot3& Rj, const Vector3& bias,
                       gtsam::OptionalJacobian<3, 3> H1 = {},
                       gtsam::OptionalJacobian<3, 3> H2 = {},
                       gtsam::OptionalJacobian<3, 3> H3 = {}) const;

  /// @deprecated constructor, but used in tests.
  PreintegratedAhrsMeasurements(const Vector3& biasHat,
                                const Matrix3& measuredOmegaCovariance)
      : PreintegratedRotation(std::make_shared<Params>()), biasHat_(biasHat) {
    p_->gyroscopeCovariance = measuredOmegaCovariance;
    resetIntegration();
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotation);
    ar& BOOST_SERIALIZATION_NVP(p_);
    ar& BOOST_SERIALIZATION_NVP(biasHat_);
  }
#endif
};

/**
 * An AHRSFactor is a three-way factor that is based on the preintegrated
 * gyroscope measurements.
 *
 * @section math_notes Mathematical Formulation
 *
 * The factor relates the orientation at two time steps, \f$ R_i \f$ and \f$ R_j \f$,
 * and the gyroscope bias \f$ b_g \f$. The error function is given by:
 * \f[
 * e(R_i, R_j, b_g) = \text{Log}\left( (\Delta \tilde{R}_{ij}(b_g))^{-1} R_i^{-1} R_j \right)
 * \f]
 * where \f$ \Delta \tilde{R}_{ij}(b_g) \f$ is the preintegrated rotation corrected
 * for the current estimate of the gyroscope bias, and \f$ \text{Log}(\cdot) \f$ is
 * the logarithmic map from SO(3) to \f$ \mathbb{R}^3 \f$.
 *
 * The preintegrated rotation \f$ \Delta R_{ij} \f$ is calculated as:
 * \f[
 * \Delta R_{ij} = \prod_{k=i}^{j-1} \text{Exp}((\omega_k - \hat{b}_g) \Delta t)
 * \f]
 * where \f$ \hat{b}_g \f$ is the bias estimate used for preintegration. The
 * bias-corrected preintegrated rotation \f$ \Delta \tilde{R}_{ij}(b_g) \f$ is
 * then approximated using a first-order expansion:
 * \f[
 * \Delta \tilde{R}_{ij}(b_g) \approx \Delta R_{ij} \text{Exp}(J_b (b_g - \hat{b}_g))
 * \f]
 * where \f$ J_b \f$ is the Jacobian of the preintegrated rotation with respect
 * to the gyroscope bias.
 */
class GTSAM_EXPORT AHRSFactor : public NoiseModelFactorN<Rot3, Rot3, Vector3> {
  typedef AHRSFactor This;
  typedef NoiseModelFactorN<Rot3, Rot3, Vector3> Base;

  PreintegratedAhrsMeasurements _PIM_;

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename std::shared_ptr<AHRSFactor> shared_ptr;
#else
  typedef std::shared_ptr<AHRSFactor> shared_ptr;
#endif

  /** Default constructor - only use for serialization */
  AHRSFactor() {}

  /**
   * Constructor
   * @param rot_i previous rot key
   * @param rot_j current rot key
   * @param bias  previous bias key
   * @param pim preintegrated measurements
   */
  AHRSFactor(Key rot_i, Key rot_j, Key bias,
             const PreintegratedAhrsMeasurements& pim);

  ~AHRSFactor() override {}

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
  Vector evaluateError(const Rot3& Ri, const Rot3& Rj, const Vector3& bias,
                       OptionalMatrixType H1, OptionalMatrixType H2,
                       OptionalMatrixType H3) const override;

  /// @deprecated constructor, but used in tests.
  AHRSFactor(Key rot_i, Key rot_j, Key bias,
             const PreintegratedAhrsMeasurements& pim,
             const Vector3& omegaCoriolis,
             const std::optional<Pose3>& body_P_sensor = {});

  /// @deprecated static function, but used in tests.
  static Rot3 predict(const Rot3& Ri, const Vector3& bias,
                      const PreintegratedAhrsMeasurements& pim,
                      const Vector3& omegaCoriolis,
                      const std::optional<Pose3>& body_P_sensor = {});

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // NoiseModelFactor3 instead of NoiseModelFactorN for backward compatibility
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(_PIM_);
  }
#endif
};
// AHRSFactor

}  // namespace gtsam
