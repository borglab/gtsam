/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GalileanPreintegration.h
 * @brief   Left-invariant Galilean IMU preintegration with bias correction
 *
 * This class implements the direct-product model developed in
 * "Left-Invariant Galilean IMU Preintegration in Inertial and Rotating
 * Frames." It combines exact held-input Gal(3) increments with the physical
 * six-axis IMU bias on Gal(3) x R^6, using the standard left-invariant error
 * and a right-applied first-order bias correction.
 *
 * The model retains Delama et al.'s held-input Galilean composition and uses
 * the endpoint and rotating-frame structure of Brossard et al., but it is not
 * either prior formulation. See navigation/doc/GalileanImuFactor.ipynb for
 * the conventions, derivation, and detailed comparison.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/navigation/ImuBias.h>
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
class GTSAM_EXPORT GalileanPreintegration : public PreintegrationBase {
 public:
  using Base = PreintegrationBase;
  using Params = PreintegrationBase::Params;
  using Matrix10 = Eigen::Matrix<double, 10, 10>;
  using Matrix106 = Eigen::Matrix<double, 10, 6>;
  using Matrix910 = Eigen::Matrix<double, 9, 10>;

  /// Select the SE_2(3) logarithm in Legacy factor-error mode.
  inline static constexpr bool kLegacyUsesLogmap = true;

 protected:
  Gal3 preintMatrix_;       ///< Mean increment in Gal(3).
  Matrix106 biasJacobian_;  ///< Right correction wrt (accel, gyro) bias.

  /** Extract the factor tangent and its Jacobian from a Gal(3) element. */
  static Vector9 NavStateTangent(const Gal3& galilean,
                                 OptionalJacobian<9, 10> H = {});

  /** Update the Gal(3) mean and return its transition and input Jacobian. */
  void updateGal3(const Vector3& measuredAcc, const Vector3& measuredOmega,
                  double dt, Matrix10* A, Matrix106* B);

  /// Default constructor for serialization.
  GalileanPreintegration() { resetIntegration(); }

 public:
  /** Construct and reset a Galilean preintegrated measurement. */
  explicit GalileanPreintegration(
      const std::shared_ptr<Params>& p,
      const imuBias::ConstantBias& biasHat = {});

  Rot3 deltaRij() const override { return preintMatrix_.rotation(); }
  Vector3 deltaPij() const override { return preintMatrix_.translation(); }
  Vector3 deltaVij() const override { return preintMatrix_.velocity(); }
  NavState deltaXij() const override {
    return NavState(preintMatrix_.rotation(), preintMatrix_.translation(),
                    preintMatrix_.velocity());
  }

  /// Return the Galilean delta in NavState tangent ordering.
  Vector9 preintegrated() const { return biasCorrectedDelta(biasHat_); }

  /** Apply the first-order physical bias correction on the right. */
  Vector9 biasCorrectedDelta(const imuBias::ConstantBias& bias_i,
                             OptionalJacobian<9, 6> H = {}) const override;

  /**
   * Update the mean and bias Jacobian, returning NavState-chart Jacobians.
   * Covariance propagation is performed by PreintegratedImuMeasurementsT.
   */
  void update(const Vector3& measuredAcc, const Vector3& measuredOmega,
              double dt, Matrix9* A, Matrix93* B, Matrix93* C) override;

  /** Reset the mean, elapsed time, and bias Jacobian. */
  void resetIntegration() override;

  void print(const std::string& s = "GalileanPreintegration") const override;
  bool equals(const GalileanPreintegration& other, double tol = 1e-9) const;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& archive, const unsigned int /*version*/) {
    archive& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationBase);
    archive& BOOST_SERIALIZATION_NVP(preintMatrix_);
    archive& BOOST_SERIALIZATION_NVP(biasJacobian_);
  }
#endif
};

}  // namespace gtsam
