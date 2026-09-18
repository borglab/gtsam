/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * One-dimensional ("planar") gyro factors.
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
#include <gtsam/slam/BetweenFactor.h>

#include <optional>

#include "gtsam/dllexport.h"

namespace gtsam {
using noiseModel::Diagonal;
/**
 * Modeled parameters of the gyro.
 *
 * https://rpg.ifi.uzh.ch/docs/UFFC16_Hidalgo.pdf
 * https://telesens.co/wp-content/uploads/2017/05/AllanVariance5087-1.pdf
 * https://github.com/borglab/gtsam/issues/213
 * https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
 */
class GTSAM_EXPORT PlanarGyroParams {
 private:
  /**
   * White noise in measured omega, which results in "angle random walk" (ARW)
   * in the integrated rotation measurement.
   *
   * Sometimes described using the coefficient, "N".
   *
   * The usual published measurement is stddev (σ), rad/√s.
   */
  const double arw;
  /**
   * Minimum of Allan variance curve, describes variation in the bias.  Also
   * called "in-run stability" or "flicker noise".
   *
   * Sometimes described using the coefficient, "B".
   *
   * The usual published measurement is stddev (σ), rad/s.
   */
  const double biasInstability;

 public:
  PlanarGyroParams(double arw, double biasInstability)
      : arw(arw), biasInstability(biasInstability) {}
  /**
   * Std dev of the integrated measurement (rad), for the specified duration
   * (sec).
   *
   * Integrated gaussian white noise is also gaussian, scaled with duration, so
   * std dev scales with sqrt(time), so the unit here is rad.
   */
  double arwSigma(double deltaT);
  /**
   * Bias instability is the "zero slope" part of the Allan curve, so it is
   * not dependent on sample rate.  The unit here is rad/s (same as the unit of
   * bias itself).
   */
  double biasInstabilitySigma();
  bool operator==(const PlanarGyroParams& other) const;
  void print(const std::string& s) const;
};

/**
 * Models the evolution of the bias itself as a random walk.
 */
class GTSAM_EXPORT PlanarGyroBiasFactor : public BetweenFactor<double> {
  typedef BetweenFactor<double> Base;

 public:
  PlanarGyroBiasFactor(Key bias_i, Key bias_j,
                       const std::shared_ptr<PlanarGyroParams>& p);
  ~PlanarGyroBiasFactor() override {}
};

/**
 * A "between" factor for Pose2 rotation, with variable bias.
 */
class GTSAM_EXPORT PlanarGyroFactor
    : public NoiseModelFactorT<Vector3, Pose2, Pose2, double> {
  typedef NoiseModelFactorT<Vector3, Pose2, Pose2, double> Base;

 private:
  const std::shared_ptr<PlanarGyroParams> p_;
  /** Incremental rotation */
  const Rot2 deltaR_;
  /** Measurement time interval (s) */
  const double deltaT_;

  PlanarGyroFactor(Key pose_i, Key pose_j, Key bias,
                   const std::shared_ptr<PlanarGyroParams>& p, Rot2 dr,
                   double dt);

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /**
   * @param pose_i Pose2 variable at i
   * @param pose_j Pose2 variable at j
   * @param bias double variable between i and j
   * @param p gyro parameters
   * @param dr rotation between i and j
   * @param dt time between i and j (s)
   */
  static inline PlanarGyroFactor FromRotation(
      Key pose_i, Key pose_j, Key bias,
      const std::shared_ptr<PlanarGyroParams>& p, Rot2 dr, double dt) {
    return PlanarGyroFactor(pose_i, pose_j, bias, p, dr, dt);
  }
  /**
   * @param pose_i Pose2 variable at i
   * @param pose_j Pose2 variable at j
   * @param bias double variable between i and j
   * @param p gyro parameters
   * @param omega average rotation rate between i and j (rad/s)
   * @param dt time between i and j (s)
   */
  static inline PlanarGyroFactor FromRate(
      Key pose_i, Key pose_j, Key bias,
      const std::shared_ptr<PlanarGyroParams>& p, double omega, double dt) {
    return PlanarGyroFactor(pose_i, pose_j, bias, p,
                            Rot2::fromAngle(omega * dt), dt);
  }

  ~PlanarGyroFactor() override {}

  gtsam::NonlinearFactor::shared_ptr clone() const override;
  void print(const std::string& s, const KeyFormatter& keyFormatter =
                                       DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor&, double tol = 1e-9) const override;

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

  /**
   * The error between the predicted and actual incremental pose (Vector3).
   *
   * @param Pi pose estimate at i
   * @param Pj pose estimate at j
   * @param bias estimate between i and j
   * @param H1 dErr/dPi (3x3)
   * @param H2 dErr/dPj (3x3)
   * @param H3 dErr/dBias (3x1)
   * @return Vector3 err
   */
  Vector3 evaluateError(const Pose2& Pi, const Pose2& Pj, const double& bias,
                        OptionalMatrixType H1, OptionalMatrixType H2,
                        OptionalMatrixType H3) const override;
};
}  // namespace gtsam
