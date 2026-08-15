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
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <iostream>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/version.hpp>
#endif

namespace gtsam {

namespace internal {

/** Shared AHRS orientation prediction for compatible preintegration types. */
template <class PIM>
Rot3 predictAhrs(const PIM& pim, const Rot3& Ri, const Vector3& bias,
                 OptionalJacobian<3, 3> H1 = {},
                 OptionalJacobian<3, 3> H2 = {}) {
  Rot3 biasCorrected =
      pim.biascorrectedDeltaRij(bias - pim.biasHat(), H2);

  // The common case needs no Earth-rotation correction. The compose Jacobian
  // with respect to its second argument is identity, so H2 is already final.
  if (!pim.p().omegaCoriolis || pim.p().omegaCoriolis->isZero(0.0))
    return Ri.compose(biasCorrected, H1);

  // Exact rotating-frame attitude transition from Brossard et al.:
  // Rj = Exp(-Omega * dt) * Ri * DeltaR.
  const Rot3 gammaRotation =
      Rot3::Expmap(-(*pim.p().omegaCoriolis) * pim.deltaTij());
  Matrix3 D_gammaRi_Ri, D_Rj_gammaRi, D_Rj_delta;
  const Rot3 gammaRi =
      gammaRotation.compose(Ri, {}, H1 ? &D_gammaRi_Ri : nullptr);
  const Rot3 Rj = gammaRi.compose(biasCorrected, H1 ? &D_Rj_gammaRi : nullptr,
                                  H2 ? &D_Rj_delta : nullptr);

  if (H1) *H1 = D_Rj_gammaRi * D_gammaRi_Ri;
  if (H2) *H2 = D_Rj_delta * (*H2);
  return Rj;
}

}  // namespace internal

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

 public:
  /// Default constructor, only for serialization and wrappers
  PreintegratedAhrsMeasurements() {}

  /**
   *  Default constructor, initialize with no measurements
   *  @param bias Current estimate of rotation rate biases
   */
  PreintegratedAhrsMeasurements(const std::shared_ptr<Params>& p,
                                const Vector3& biasHat = Vector3::Zero())
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
               gtsam::OptionalJacobian<3, 3> H2 = {}) const {
    return internal::predictAhrs(*this, Ri, bias, H1, H2);
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotation);
    ar& BOOST_SERIALIZATION_NVP(p_);
    ar& BOOST_SERIALIZATION_NVP(biasHat_);
    if (version > 0) {
      ar& BOOST_SERIALIZATION_NVP(preintMeasCov_);
    } else if (ARCHIVE::is_loading::value) {
      preintMeasCov_.setZero();
    }
  }
#endif
};

/**
 * An AHRSFactor is a three-way factor that is based on the preintegrated
 * gyroscope measurements.
 *
 * @section ahrs_factor_math_notes Mathematical Formulation
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
template <class PIM>
class AHRSFactorT
    : public NoiseModelFactorT<Vector3, Rot3, Rot3, Vector3> {
  using This = AHRSFactorT<PIM>;
  using Base = NoiseModelFactorT<Vector3, Rot3, Rot3, Vector3>;

  PIM pim_;

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename std::shared_ptr<This> shared_ptr;
#else
  typedef std::shared_ptr<This> shared_ptr;
#endif

  /** Default constructor - only use for serialization */
  AHRSFactorT() = default;

  /**
   * Constructor
   * @param rot_i previous rot key
   * @param rot_j current rot key
   * @param bias  previous bias key
   * @param pim preintegrated measurements
   */
  AHRSFactorT(Key rot_i, Key rot_j, Key bias, const PIM& pim)
      : Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov()), rot_i,
             rot_j, bias),
        pim_(pim) {}

  ~AHRSFactorT() override = default;

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  /// print
  void print(const std::string& s,
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "AHRSFactor(" << keyFormatter(this->template key<1>())
              << "," << keyFormatter(this->template key<2>()) << ","
              << keyFormatter(this->template key<3>()) << ")\n";
    pim_.print("  preintegrated measurements:");
    this->noiseModel_->print("  noise model: ");
  }

  /// equals
  bool equals(const NonlinearFactor& other, double tol = 1e-9) const override {
    const auto* expected = dynamic_cast<const This*>(&other);
    return expected != nullptr && Base::equals(*expected, tol) &&
           pim_.equals(expected->pim_, tol);
  }

  /// Access the preintegrated measurements.
  const PIM& preintegratedMeasurements() const { return pim_; }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector3 evaluateError(const Rot3& Ri, const Rot3& Rj, const Vector3& bias,
                        OptionalMatrixType H1, OptionalMatrixType H2,
                        OptionalMatrixType H3) const override {
    Matrix3 D_predict_Ri, D_predict_bias;
    const Rot3 predictedRj = internal::predictAhrs(
        pim_, Ri, bias, H1 ? &D_predict_Ri : nullptr,
        H3 ? &D_predict_bias : nullptr);

    Matrix3 D_error_Rj, D_error_predict;
    const Vector3 error = Rj.logmap(
        predictedRj, H2 ? &D_error_Rj : nullptr,
        H1 || H3 ? &D_error_predict : nullptr);

    if (H1) *H1 = D_error_predict * D_predict_Ri;
    if (H2) *H2 = D_error_Rj;
    if (H3) *H3 = D_error_predict * D_predict_bias;
    return error;
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // NoiseModelFactor3 instead of NoiseModelFactorN for backward compatibility
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
    ar& boost::serialization::make_nvp("_PIM_", pim_);
  }
#endif
};
using AHRSFactor = AHRSFactorT<PreintegratedAhrsMeasurements>;

extern template class AHRSFactorT<PreintegratedAhrsMeasurements>;

template <>
struct traits<PreintegratedAhrsMeasurements>
    : public Testable<PreintegratedAhrsMeasurements> {};

template <class PIM>
struct traits<AHRSFactorT<PIM>> : public Testable<AHRSFactorT<PIM>> {};

}  // namespace gtsam

#if GTSAM_ENABLE_BOOST_SERIALIZATION
BOOST_CLASS_VERSION(gtsam::PreintegratedAhrsMeasurements, 1)
#endif
