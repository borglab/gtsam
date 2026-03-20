/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EqVIOSymmetry.h
 * @brief EqVIO symmetry actions and lift helpers.
 * @author Rohan Bansal
 */

#pragma once

#include <gtsam/base/GroupAction.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam_unstable/navigation/EqVIOCommon.h>
#include <gtsam_unstable/navigation/EqVIOState.h>
#include <gtsam_unstable/dllexport.h>

#include <functional>

namespace gtsam {
namespace eqvio {

/// Right group action on the sensor-only block.
GTSAM_UNSTABLE_EXPORT SensorState sensorStateGroupAction(
    const VioGroup& X, const SensorState& sensor);
/// Right group action on full state.
GTSAM_UNSTABLE_EXPORT State stateGroupAction(const VioGroup& X,
                                                const State& state);
/// Right group action on vision measurements.
GTSAM_UNSTABLE_EXPORT VisionMeasurement outputGroupAction(
    const VioGroup& X, const VisionMeasurement& measurement,
    const std::shared_ptr<const CameraModel>& camera);

/// Continuous-time lift map from IMU velocity to VioGroup tangent.
GTSAM_UNSTABLE_EXPORT Vector liftVelocity(const State& state,
                                          const IMUInput& velocity);
/// Discrete-time lift map from IMU velocity to VioGroup increment.
GTSAM_UNSTABLE_EXPORT VioGroup liftVelocityDiscrete(const State& state,
                                                    const IMUInput& velocity,
                                                    double dt);

/// Integrate system dynamics forward by dt.
GTSAM_UNSTABLE_EXPORT State integrateSystemFunction(
    const State& state, const IMUInput& velocity, double dt);
/// Generate ideal camera measurements from state.
GTSAM_UNSTABLE_EXPORT VisionMeasurement measureSystemState(
    const State& state, const std::shared_ptr<const CameraModel>& camera);

/// InvDepth EqF coordinate suite and associated matrices/lifts.
struct GTSAM_UNSTABLE_EXPORT EqFCoordinateSuite {
  std::function<Vector(const State&, const State&)> stateChart;
  std::function<State(const Vector&, const State&)> stateChartInv;
  std::function<Matrix(const VioGroup&, const State&, const IMUInput&)>
      stateMatrixA;
  std::function<Matrix(const VioGroup&, const State&)> inputMatrixB;
  std::function<Matrix23(const Point3&, const SOT3&,
                         const std::shared_ptr<const CameraModel>&,
                         const Point2&)>
      outputMatrixCiStar;
  std::function<Vector(const Vector&, const State&)> liftInnovation;
  std::function<VioGroup(const Vector&, const State&)> liftInnovationDiscrete;

  Matrix outputMatrixC(const State& xi0, const VioGroup& X,
                       const VisionMeasurement& y,
                       const std::shared_ptr<const CameraModel>& camera,
                       bool useEquivariance = true) const;

  Matrix stateMatrixADiscrete(const VioGroup& X, const State& xi0,
                              const IMUInput& imuVel, double dt) const;

  Matrix23 outputMatrixCi(const Point3& q0, const SOT3& QHat,
                          const std::shared_ptr<const CameraModel>& camera)
      const;
};

extern const GTSAM_UNSTABLE_EXPORT EqFCoordinateSuite EqFCoordinateSuite_invdepth;

/// Right action phi(xi, X) = stateGroupAction(X, xi).
struct GTSAM_UNSTABLE_EXPORT Symmetry
    : public GroupAction<Symmetry, VioGroup, State> {
  static constexpr ActionType type = ActionType::Right;

  /// Evaluate right action phi(xi, X).
  State operator()(const State& xi, const VioGroup& X,
                      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_xi = {},
                      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_X = {})
      const;
};

}  // namespace eqvio
}  // namespace gtsam
