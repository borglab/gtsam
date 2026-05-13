/**
 * Yaw measurement and factor.
 *
 * Useful for high-school robotics competitions,
 * which run robots on the floor.
 *
 * @see https://www.firstinspires.org/
 *
 * @file PlanarGyroFactor.h
 * @author joel@truher.org
 * @date May 1, 2026
 */

#pragma once
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <optional>

#include "gtsam/dllexport.h"

namespace gtsam {
/**
 * Modeled parameters of the gyro.
 *
 * https://rpg.ifi.uzh.ch/docs/UFFC16_Hidalgo.pdf
 * https://telesens.co/wp-content/uploads/2017/05/AllanVariance5087-1.pdf
 */
struct GTSAM_EXPORT PlanarGyroParams {
  /**
   * White noise in omega, which results in "angle random walk" (ARW) in the
   * integrated rotation measurement.
   *
   * Sometimes described as "sigma_n" or using the coefficient, "N"
   *
   * The usual published unit is stddev (σ), rad/s/√Hz.
   */
  const double arwSigma;
  /**
   * Minimum of Allan variance curve, describes variation in the bias.  Also
   * called "in-run stability" or "flicker noise".
   *
   * Sometimes described as "sigma_ug" or using the coefficient, "B".
   *
   * The usual published unit is stddev (σ), rad/s.
   */
  const double biasInstabilitySigma;

  PlanarGyroParams(double arwSigma, double biasInstabilitySigma)
      : arwSigma(arwSigma), biasInstabilitySigma(biasInstabilitySigma) {}

  bool operator==(const PlanarGyroParams& other) const {
    return arwSigma == other.arwSigma &&
           biasInstabilitySigma == other.biasInstabilitySigma;
  }
};

/**
 * Measurement of a one-dimensional gyro, handles bias.
 */
class GTSAM_EXPORT PlanarGyroMeasurement {
 private:
  const std::shared_ptr<PlanarGyroParams> p_;
  /** Incremental rotation */
  const Rot2 deltaR_;
  /** Measurement time interval (s) */
  const double deltaT_;

  PlanarGyroMeasurement(const std::shared_ptr<PlanarGyroParams>& p, Rot2 dr,
                        double dt)
      : p_(p), deltaR_(dr), deltaT_(dt) {}

 public:
  /**
   * @param p gyro parameters
   * @param omega average rotation rate during dt (rad/s)
   * @param dt incremental time (s)
   */
  static inline PlanarGyroMeasurement fromRate(
      const std::shared_ptr<PlanarGyroParams>& p, double omega, double dt) {
    return PlanarGyroMeasurement(p, Rot2::fromAngle(omega * dt), dt);
  }

  /**
   * @param p gyro parameters
   * @param dr incremental rotation during dt
   * @param dt incremental time (s)
   */
  static inline PlanarGyroMeasurement fromRotation(
      const std::shared_ptr<PlanarGyroParams>& p, Rot2 dr, double dt) {
    return PlanarGyroMeasurement(p, dr, dt);
  }

  /**
   * Std dev of the measurement (rad)
   */
  double sigma() const {
    // Integrated white noise => variance scales linearly with time.
    // Std dev scales with sqrt(time).
    return p_->arwSigma * sqrt(deltaT_);
  }

  void print(const std::string& s = "Measurements: ") const;
  bool equals(const PlanarGyroMeasurement& expected, double tol = 1e-9) const;

  /**
   * Bias-corrected rotation.
   *
   * @param bias rate (rad/s)
   * @param H derivative of rotation wrt bias.
   */
  Rot2 deltaR(double bias, OptionalJacobian<1, 1> H = {}) const;

  /**
   * Predicted rotation at time j, given rotation and bias at time i.
   *
   * @param Ri rotation at time i (rad)
   * @param bias rate (rad/s)
   * @param H1 dRj/dRi
   * @param H2 dRj/dBias
   */
  Rot2 predict(const Rot2& Ri, double bias, OptionalJacobian<1, 1> H1 = {},
               OptionalJacobian<1, 1> H2 = {}) const;

  /**
   * The error between the predicted and actual rotation (rad)
   *
   * @param Ri rotation at time i (rad)
   * @param Rj rotation at time j (rad)
   * @param bias rate (rad/s)
   * @param H1 dErr/dRi
   * @param H2 dErr/dRj
   * @param H3 dErr/dBias
   */
  double computeError(const Rot2& Ri, const Rot2& Rj, double bias,
                      OptionalJacobian<1, 1> H1 = {},
                      OptionalJacobian<1, 1> H2 = {},
                      OptionalJacobian<1, 1> H3 = {}) const;
};

/**
 * A "between" factor for Pose2 rotation, with variable bias.
 */
class GTSAM_EXPORT PlanarGyroFactor
    : public NoiseModelFactorN<Pose2, Pose2, double> {
  typedef NoiseModelFactorN<Pose2, Pose2, double> Base;

  const PlanarGyroMeasurement measurement_;

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename std::shared_ptr<PlanarGyroFactor> shared_ptr;
#else
  typedef std::shared_ptr<PlanarGyroFactor> shared_ptr;
#endif

  PlanarGyroFactor(Key pose_i, Key pose_j, Key bias,
                   const PlanarGyroMeasurement& measurement);
  ~PlanarGyroFactor() override {}

  gtsam::NonlinearFactor::shared_ptr clone() const override;
  void print(const std::string& s, const KeyFormatter& keyFormatter =
                                       DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor&, double tol = 1e-9) const override;

  /**
   * @param Pi pose estimate at i
   * @param Pj pose estimate at j
   * @param bias estimate between i and j
   * @param H1 dErr/dPi (3x3)
   * @param H2 dErr/dPj (3x3)
   * @param H3 dErr/dBias (3x1)
   * @return Vector3 err
   */
  Vector evaluateError(const Pose2& Pi, const Pose2& Pj, const double& bias,
                       OptionalMatrixType H1, OptionalMatrixType H2,
                       OptionalMatrixType H3) const override;
};
}  // namespace gtsam