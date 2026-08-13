/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved.
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieGroupPreintegration.h
 * @brief IMU preintegration using the SE_2(3) group structure of NavState.
 * @author Martin Brossard
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/navigation/ManifoldPreintegration.h>

namespace gtsam {

/**
 * IMU preintegration using the SE_2(3) Lie-group structure of NavState.
 *
 * Unlike ManifoldPreintegration, each increment is applied with the group
 * exponential map. NavState's legacy retract/localCoordinates chart remains
 * unchanged and is still used by optimization and PreintegrationBase.
 */
class GTSAM_EXPORT LieGroupPreintegration : public ManifoldPreintegration {
 protected:
  /// Default constructor for serialization.
  LieGroupPreintegration() = default;

 public:
  /** Construct an empty preintegrator with parameters and a bias linearization
   * point. */
  LieGroupPreintegration(
      const std::shared_ptr<Params>& params,
      const imuBias::ConstantBias& biasHat = imuBias::ConstantBias());

  /** Return the bias-corrected delta in NavState's legacy tangent ordering. */
  Vector9 biasCorrectedDelta(const imuBias::ConstantBias& bias,
                             OptionalJacobian<9, 6> H = {}) const override;

 protected:
  /** Apply one unbiased IMU increment with NavState's group exponential. */
  void updateFactor(const Vector3& bodyAcceleration, const Vector3& bodyOmega,
                    double dt, OptionalJacobian<9, 9> F = {},
                    OptionalJacobian<9, 3> G1 = {},
                    OptionalJacobian<9, 3> G2 = {}) override;

  void updateBiasJacobians(const Rot3& oldRotation,
                           const Vector3& bodyAcceleration,
                           const Vector3& bodyOmega, double dt,
                           const Matrix9& stateTransition,
                           const Matrix93& accelerationJacobian,
                           const Matrix93& omegaJacobian) override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& archive, const unsigned int /*version*/) {
    archive& BOOST_SERIALIZATION_BASE_OBJECT_NVP(ManifoldPreintegration);
  }
#endif
};

}  // namespace gtsam
