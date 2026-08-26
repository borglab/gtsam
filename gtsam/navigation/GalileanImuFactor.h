/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GalileanImuFactor.h
 * @brief   Left-invariant Galilean IMU preintegration with bias correction
 *
 * This class implements the Galilean preintegration model described in:
 *
 *   Giulio Delama, Alessandro Fornasier, Robert Mahony, Stephan Weiss,
 *   "Equivariant IMU Preintegration with Biases: a Galilean Group Approach,"
 *   IEEE Robotics and Automation Letters, 2025.
 *
 * Unlike the paper's right-invariant error, this implementation uses GTSAM's
 * right retraction and therefore a left-invariant local error. See
 * navigation/doc/GalileanImuFactor.ipynb for the conventions and derivation.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationBase.h>

namespace gtsam {

/**
 * IMU preintegration on Gal(3) using GTSAM's left-invariant local error.
 *
 * The internal tangent ordering is `(rotation, velocity, position, time)`. The
 * public factor contract projects this to NavState ordering
 * `(rotation, position, velocity)` and removes deterministic time. Biases use
 * GTSAM's `(accelerometer, gyroscope)` ordering and are corrected on the right.
 */
class GTSAM_EXPORT PreintegratedImuMeasurementsG : public PreintegrationBase {
 public:
  using Base = PreintegrationBase;
  using Params = PreintegrationBase::Params;
  using Matrix10 = Eigen::Matrix<double, 10, 10>;
  using Matrix106 = Eigen::Matrix<double, 10, 6>;
  using Matrix910 = Eigen::Matrix<double, 9, 10>;

 protected:
  Gal3 preintMatrix_;       ///< Mean increment in Gal(3).
  Matrix106 biasJacobian_;  ///< Right correction wrt (accel, gyro) bias.
  Matrix10 preintMeasCov_;  ///< Covariance in Gal(3) LI local coordinates.

  /** Project Gal(3) tangent order to NavState tangent order. */
  static Matrix910 NavStateProjector();

  /** Lift GTSAM input order `(acceleration, angular rate)` into Gal(3). */
  static Matrix106 LiftJacobian();

  /** Update the Gal(3) mean and return its transition and input Jacobian. */
  void updateGal3(const Vector3& measuredAcc, const Vector3& measuredOmega,
                  double dt, Matrix10* A, Matrix106* B);

 public:
  /** Construct and reset a Galilean preintegrated measurement. */
  explicit PreintegratedImuMeasurementsG(
      const std::shared_ptr<Params>& p = std::make_shared<Params>(),
      const imuBias::ConstantBias& biasHat = {});

  Rot3 deltaRij() const override { return preintMatrix_.rotation(); }
  Vector3 deltaPij() const override { return preintMatrix_.translation(); }
  Vector3 deltaVij() const override { return preintMatrix_.velocity(); }
  NavState deltaXij() const override {
    return NavState(preintMatrix_.rotation(), preintMatrix_.translation(),
                    preintMatrix_.velocity());
  }

  /** Return covariance in the NavState factor residual chart. */
  Matrix9 preintMeasCov() const;

  /** Return covariance used by ImuFactorT. */
  Matrix9 residualCovariance() const { return preintMeasCov(); }

  /** Apply the first-order physical bias correction on the right. */
  Vector9 biasCorrectedDelta(const imuBias::ConstantBias& bias_i,
                             OptionalJacobian<9, 6> H = {}) const override;

  /**
   * Update the mean and bias Jacobian, returning NavState-chart Jacobians.
   * Covariance propagation is performed by integrateMeasurement().
   */
  void update(const Vector3& measuredAcc, const Vector3& measuredOmega,
              double dt, Matrix9* A, Matrix93* B, Matrix93* C) override;

  /** Integrate one measurement and propagate its conditional covariance. */
  void integrateMeasurement(const Vector3& measuredAcc,
                            const Vector3& measuredOmega, double dt) override;

  /** Reset the mean, elapsed time, covariance, and bias Jacobian. */
  void resetIntegration() override;

  void print(
      const std::string& s = "PreintegratedImuMeasurementsG") const override;
  bool equals(const PreintegratedImuMeasurementsG& other,
              double tol = 1e-9) const;
};

template <>
struct traits<PreintegratedImuMeasurementsG>
    : public Testable<PreintegratedImuMeasurementsG> {};

using GalileanImuFactor = ImuFactorT<PreintegratedImuMeasurementsG>;

}  // namespace gtsam
