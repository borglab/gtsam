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
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/navigation/PreintegrationParams.h>

#include <optional>
#include <vector>

namespace gtsam {

/// IMU preintegration based on the Galilean group (see class docs above).
class GTSAM_EXPORT PreintegratedImuMeasurementsG : public PreintegrationBase {
  using Base = PreintegrationBase;
  using Params = PreintegrationBase::Params;

  std::shared_ptr<Params> p_;

 public:
  // Constructors (stubs)
  explicit PreintegratedImuMeasurementsG(
      const std::shared_ptr<Params>& p = std::make_shared<Params>(),
      const imuBias::ConstantBias& biasHat = {});

  // Public (const) accessors – stubs
  double deltaTij() const {
    throw std::runtime_error("Not implemented: deltaTij");
  }
  Rot3 deltaRij() const override {
    throw std::runtime_error("Not implemented: deltaRij");
  }
  Vector3 deltaPij() const override {
    throw std::runtime_error("Not implemented: deltaPij");
  }
  Vector3 deltaVij() const override {
    throw std::runtime_error("Not implemented: deltaVij");
  }
  NavState deltaXij() const override {
    throw std::runtime_error("Not implemented: deltaXij");
  }
  Matrix9 preintMeasCov() const {
    throw std::runtime_error("Not implemented: preintMeasCov");
  }

  // Bias correction (first‑order) – stub
  Vector9 biasCorrectedDelta(const imuBias::ConstantBias& bias_i,
                             OptionalJacobian<9, 6> H = {}) const override {
    throw std::runtime_error("Not implemented: biasCorrectedDelta");
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

  void update(const Vector3& measuredAcc, const Vector3& measuredOmega,
              const double dt, Matrix9* A, Matrix93* B, Matrix93* C) override {
    throw std::runtime_error("Not implemented: update");
  }
};

// Tell GTSAM about PreintegratedImuMeasurementsG print/equals:
template <>
struct traits<PreintegratedImuMeasurementsG>
    : public Testable<PreintegratedImuMeasurementsG> {};

using GalileanImuFactor = ImuFactorT<PreintegratedImuMeasurementsG>;

}  // namespace gtsam
