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
#include <gtsam_unstable/dllexport.h>
#include <gtsam_unstable/navigation/EqVIOCommon.h>
#include <gtsam_unstable/navigation/EqVIOState.h>

namespace gtsam {
namespace eqvio {

/**
 * @brief Right action of `VioGroup` on full EqVIO state.
 *
 * Applies the sensor-side right action to `kinematics`, `bias`, and
 * `cameraOffset`, and the SOT3 inverse action to each landmark state.
 */
GTSAM_UNSTABLE_EXPORT State stateGroupAction(const VioGroup& X,
                                             const State& state);

/**
 * @brief Discrete lift map from IMU input to a `VioGroup` increment.
 *
 * This is the discrete propagation primitive used by
 * `EqVIOFilter::propagateState`.
 */
GTSAM_UNSTABLE_EXPORT VioGroup liftVelocityDiscrete(const State& state,
                                                    const IMUInput& velocity,
                                                    double dt);

/**
 * @brief Generate ideal image measurements by projecting all landmarks.
 *
 * `landmarkIds[i]` is the key assigned to `state.cameraLandmarks[i]`.
 *
 * @throws std::invalid_argument if `camera` is null or id count mismatches
 * landmarks.
 */
GTSAM_UNSTABLE_EXPORT VisionMeasurement
measureSystemState(const State& state, const KeyVector& landmarkIds,
                   const std::shared_ptr<const CameraModel>& camera);

/**
 * @brief EqF linearization blocks used by EqVIO.
 *
 * EqVIO uses an inverse-depth landmark error chart to preserve the
 * equivariant output linearization while remaining numerically stable for
 * distant points. Other coordinate systems (Euclid, Polar) are not yet 
 * implemented. van Goor et al. propose Polar as the new coordinate system
 * for this use case, which is the next TODO. See arXiv:2205.01980v3, eq. 16.
 */

/// EqF state matrix that maps IMU input to state change.
GTSAM_UNSTABLE_EXPORT Matrix EqFStateMatrixA(const VioGroup& X,
                                             const State& xi0,
                                             const IMUInput& imuVel);
/// EqF input matrix that maps IMU driving noise into chart error coordinates.
GTSAM_UNSTABLE_EXPORT Matrix EqFInputMatrixB(const VioGroup& X,
                                             const State& xi0);
/// Per-landmark equivariant output Jacobian using observed measurement `y`.
GTSAM_UNSTABLE_EXPORT Matrix23 EqFoutputMatrixCiStar(
    const Point3& q0, const SOT3& QHat,
    const std::shared_ptr<const CameraModel>& camera, const Point2& y);
/// Stacked output matrix for all currently observed landmarks.
GTSAM_UNSTABLE_EXPORT Matrix
EqFoutputMatrixC(const State& xi0, const KeyVector& landmarkIds,
                 const VioGroup& X, const VisionMeasurement& y,
                 const std::shared_ptr<const CameraModel>& camera);

/**
 * @brief Lift EqVIO correction from state-chart coordinates to group tangent
 * coordinates.
 *
 * EqVIO uses a specialized error chart (including inverse-depth landmark
 * coordinates) for linearization. The resulting correction `delta_xi` from the
 * Kalman step lives in state-chart coordinates (`21 + 3N`), while the group
 * update uses `VioGroup` tangent coordinates (`21 + 4N`).
 *
 * A direct base-class lift via a fixed pseudo-inverse of `Dphi0` is not
 * sufficient here because:
 * 1. each landmark block must be mapped from a 3D chart perturbation to a 4D
 *    SOT3 tangent perturbation (rotation + log-scale), and
 * 2. the sensor blocks require EqVIO-specific coupling terms consistent with
 *    the right-action formulation and chosen coordinates.
 *
 * This function applies that EqVIO-specific mapping so the correction can be
 * integrated as a valid group increment.
 *
 * @param totalInnovation Kalman correction in EqVIO chart coordinates (`21 +
 * 3N`).
 * @param xi0 Reference state used by the chart conversion.
 * @return Group-tangent correction (`21 + 4N`) for `VioGroup::Expmap`.
 *
 * @throws std::invalid_argument if `totalInnovation` has inconsistent
 * dimension.
 */
GTSAM_UNSTABLE_EXPORT Vector liftInnovation(const Vector& totalInnovation,
                                            const State& xi0);

/// Right action phi(xi, X) = stateGroupAction(X, xi).
struct GTSAM_UNSTABLE_EXPORT Symmetry
    : public GroupAction<Symmetry, VioGroup, State> {
  static constexpr ActionType type = ActionType::Right;

  /**
   * @brief Evaluate right action `phi(xi, X)`.
   * @param xi State argument.
   * @param X Group argument.
   * @param H_xi Optional Jacobian w.r.t. state argument.
   * @param H_X Optional Jacobian w.r.t. group argument.
   */
  State operator()(
      const State& xi, const VioGroup& X,
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_xi = {},
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_X = {}) const;
};

}  // namespace eqvio
}  // namespace gtsam
