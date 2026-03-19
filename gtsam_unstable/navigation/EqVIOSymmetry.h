/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOSymmetry.h
/// @brief EqVIO symmetry actions and lift helpers.
/// @author Rohan Bansal

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
GTSAM_UNSTABLE_EXPORT VIOSensorState sensorStateGroupAction(
    const VIOGroup& X, const VIOSensorState& sensor);
/// Right group action on full state.
GTSAM_UNSTABLE_EXPORT VIOState stateGroupAction(const VIOGroup& X,
                                                const VIOState& state);
/// Right group action on vision measurements.
GTSAM_UNSTABLE_EXPORT VisionMeasurement outputGroupAction(
    const VIOGroup& X, const VisionMeasurement& measurement,
    const std::shared_ptr<const VIOCameraModel>& camera);

/// Continuous-time lift map from IMU velocity to VIOGroup tangent.
GTSAM_UNSTABLE_EXPORT Vector liftVelocity(const VIOState& state,
                                          const IMUInput& velocity);
/// Discrete-time lift map from IMU velocity to VIOGroup increment.
GTSAM_UNSTABLE_EXPORT VIOGroup liftVelocityDiscrete(const VIOState& state,
                                                    const IMUInput& velocity,
                                                    double dt);

/// Integrate system dynamics forward by dt.
GTSAM_UNSTABLE_EXPORT VIOState integrateSystemFunction(
    const VIOState& state, const IMUInput& velocity, double dt);
/// Generate ideal camera measurements from state.
GTSAM_UNSTABLE_EXPORT VisionMeasurement measureSystemState(
    const VIOState& state, const std::shared_ptr<const VIOCameraModel>& camera);

/// InvDepth EqF coordinate suite and associated matrices/lifts.
struct GTSAM_UNSTABLE_EXPORT EqFCoordinateSuite {
  std::function<Vector(const VIOState&, const VIOState&)> stateChart;
  std::function<VIOState(const Vector&, const VIOState&)> stateChartInv;
  std::function<Matrix(const VIOGroup&, const VIOState&, const IMUInput&)>
      stateMatrixA;
  std::function<Matrix(const VIOGroup&, const VIOState&)> inputMatrixB;
  std::function<Matrix23(const Point3&, const SOT3&,
                         const std::shared_ptr<const VIOCameraModel>&,
                         const Point2&)>
      outputMatrixCiStar;
  std::function<Vector(const Vector&, const VIOState&)> liftInnovation;
  std::function<VIOGroup(const Vector&, const VIOState&)> liftInnovationDiscrete;

  Matrix outputMatrixC(const VIOState& xi0, const VIOGroup& X,
                       const VisionMeasurement& y,
                       const std::shared_ptr<const VIOCameraModel>& camera,
                       bool useEquivariance = true) const;

  Matrix stateMatrixADiscrete(const VIOGroup& X, const VIOState& xi0,
                              const IMUInput& imuVel, double dt) const;

  Matrix23 outputMatrixCi(const Point3& q0, const SOT3& QHat,
                          const std::shared_ptr<const VIOCameraModel>& camera)
      const;
};

extern const GTSAM_UNSTABLE_EXPORT EqFCoordinateSuite EqFCoordinateSuite_invdepth;

/// Right action phi(xi, X) = stateGroupAction(X, xi).
struct GTSAM_UNSTABLE_EXPORT VIOSymmetry
    : public GroupAction<VIOSymmetry, VIOGroup, VIOState> {
  static constexpr ActionType type = ActionType::Right;

  /// Evaluate right action phi(xi, X).
  VIOState operator()(const VIOState& xi, const VIOGroup& X,
                      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_xi = {},
                      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_X = {})
      const;
};

}  // namespace eqvio
}  // namespace gtsam
