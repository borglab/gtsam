/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GalileanImuFactor.h
 * @brief   Equivariant IMU Preintegration with Biases: a Galilean Group
 * Approach
 *
 * This class implements a new approach for Inertial Measurement Unit (IMU)
 * preintegration based on the Galilean group, as described in:
 *
 *   Giulio Delama, Alessandro Fornasier, Robert Mahony, Stephan Weiss,
 *   "Equivariant IMU Preintegration with Biases: a Galilean Group Approach,"
 *   IEEE Robotics and Automation Letters, 2024.
 *
 * Inspired by recent advances in equivariant theory applied to biased inertial
 * navigation systems (INS), this approach derives a discrete-time formulation
 * of IMU preintegration on the Galilean group, geometrically coupling
 * navigation states and biases for improved consistency and lower linearization
 * error compared to traditional methods.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/navigation/PreintegrationCombinedParams.h>

#include <optional>
#include <vector>

namespace gtsam {

/// IMU preintegration based on the Galilean group (see class docs above).
class GTSAM_EXPORT PreintegratedImuMeasurementsG : public PreintegrationBase {
  using Base = PreintegrationBase;
  using Params = PreintegrationBase::Params;
  using Matrix10 = Eigen::Matrix<double, 10, 10>;
  using Matrix20 = Eigen::Matrix<double, 20, 20>;

 protected:
  std::shared_ptr<Params> p_;  ///< Preintegration parameters
  Gal3 preintMatrix_;  ///< Preintegration Matrix (Galilean group element)
  Matrix20 preintBiasJacobian_;  ///< Jacobian w.r.t. bias states
  Matrix10 preintMeasCov_;       ///< Preintegration covariance
  ///< The dimension of the preintegration covariance matrix is 10x10
  ///< Because it includes Galilean Group (n=10)
  ///< preintROTATION, preintVELOCITY, preintPOSITION, preintTIME

 public:
  // Constructors (stubs)
  explicit PreintegratedImuMeasurementsG(
      const std::shared_ptr<Params>& p = std::make_shared<Params>(),
      const imuBias::ConstantBias& biasHat = {});

  // Public (const) accessors – stubs
  double deltaTij() const { return preintMatrix_.time(); }
  Rot3 deltaRij() const override { return preintMatrix_.rotation(); }
  Vector3 deltaPij() const override { return preintMatrix_.translation(); }
  Vector3 deltaVij() const override { return preintMatrix_.velocity(); }
  NavState deltaXij() const override {
    return NavState(preintMatrix_.rotation(), preintMatrix_.translation(),
                    preintMatrix_.velocity());
  }
  Matrix9 preintMeasCov() const { return preintMeasCov_.block<9, 9>(0, 0); }

  // Bias correction (first‑order) – stub
  Vector9 biasCorrectedDelta(const imuBias::ConstantBias& bias_i,
                             OptionalJacobian<9, 6> H = {}) const override {
    const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
    Gal3 correctedPreintMatrix =
        Gal3::Expmap(preintBiasJacobian_.block<10, 6>(0, 0) * biasIncr.vector())
            .compose(preintMatrix_);

    Vector9 xi;
    NavState::dR(xi) = Rot3::Logmap(correctedPreintMatrix.rotation());
    NavState::dP(xi) = correctedPreintMatrix.translation();
    NavState::dV(xi) = correctedPreintMatrix.velocity();

    if (H) {
      throw std::runtime_error("Not implemented: biasCorrectedDelta Jacobian");
    }
    return xi;
  }

  // integration API
  void integrateMeasurement(const Vector3& measuredAcc,
                            const Vector3& measuredOmega, double dt) override;

  void resetIntegration() override;

  // Testable – stubs
  void print(
      const std::string& s = "PreintegratedImuMeasurementsG") const override;
  bool equals(const PreintegratedImuMeasurementsG& other,
              double tol = 1e-9) const;

  // void update(const Vector3& measuredAcc, const Vector3& measuredOmega,
  //             const double dt, Matrix9* A, Matrix93* B, Matrix93* C) override
  //             {}
};

// Tell GTSAM about PreintegratedImuMeasurementsG print/equals:
template <>
struct traits<PreintegratedImuMeasurementsG>
    : public Testable<PreintegratedImuMeasurementsG> {};

using GalileanImuFactor = ImuFactorT<PreintegratedImuMeasurementsG>;

}  // namespace gtsam
