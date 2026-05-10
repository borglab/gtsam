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
 * Measurement of a one-dimensional gyro, handles bias.
 */
class GTSAM_EXPORT PlanarGyroMeasurement {
 private:
  /**
   * Published or measured variance of gyroscope measurements.
   * This is white noise in omega, which results in "angle random walk"
   * (ARW) in the integrated rotation measurement.
   * The usual published unit is stddev (σ), rad/s/√Hz.
   * This is variance (σ^2), so the unit is (rad/s)^2/Hz, or rad^2/s.
   */
  const double ARW_;
  /** Incremental rotation */
  const Rot2 deltaR_;
  /** Measurement time interval (s) */
  const double deltaT_;

  PlanarGyroMeasurement(double ARW, Rot2 dr, double dt)
      : ARW_(ARW), deltaR_(dr), deltaT_(dt) {}

 public:
  /**
   * @param ARW "angle random walk" instrument variance (rad^2/s)
   * @param omega average rotation rate during dt (rad/s)
   * @param dt incremental time (s)
   */
  static inline PlanarGyroMeasurement fromRate(double ARW, double omega,
                                               double dt) {
    return PlanarGyroMeasurement(ARW, Rot2::fromAngle(omega * dt), dt);
  }

  /**
   * @param ARW "angle random walk" instrument variance (rad^2/s)
   * @param dr incremental rotation during dt
   * @param dt incremental time (s)
   */
  static inline PlanarGyroMeasurement fromRotation(double ARW, Rot2 dr,
                                                   double dt) {
    return PlanarGyroMeasurement(ARW, dr, dt);
  }

  /**
   * Variance of the measurement (rad^2)
   */
  double variance() const {
    // Integrated white noise => variance scales linearly with time.
    return ARW_ * deltaT_;
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