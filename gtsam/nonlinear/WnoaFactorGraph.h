/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  WnoaFactorGraph.h
 *  @brief Factor graph that handles computation of interpolated states for WNOA
 *  @author Sven Lilge
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/WnoaInterpFactor.h>
#include <gtsam/nonlinear/WnoaInterpolator.h>
#include <gtsam/nonlinear/WnoaStateData.h>

#include <array>
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace gtsam {

/**
 * @brief Factor graph specialized for WNOA interpolation-aware computation.
 *
 * `WnoaFactorGraph` wraps standard factor graph functionalities with utilities
 * to compute interpolated pose/velocity states under a
 * White-Noise-on-Acceleration (WNOA) motion prior. It stores the mapping from
 * interpolated query times to their left/right bordering estimated states and
 * precomputes interpolation helpers for efficient repeated evaluation.
 *
 * The graph provides optimized linearization and error computation routines
 * that can exploit precomputed interpolation batches to reduce repeated
 * work when evaluating many wrapper factors using the same query times.
 *
 * @tparam PoseType Pose group/type (e.g. `Pose2`, `Pose3`) used by the
 * interpolator.
 */
template <typename PoseType>
class GTSAM_EXPORT WnoaFactorGraph : public ExpressionFactorGraph {
 private:
  using This = WnoaFactorGraph<PoseType>;
  using Base = ExpressionFactorGraph;
  using VelocityType = typename gtsam::traits<PoseType>::TangentVector;
  static constexpr int dim = traits<PoseType>::dimension;

  // Convenient matrices
  using Matrix2N = Eigen::Matrix<double, 2 * dim, 2 * dim>;
  using MatrixN = Eigen::Matrix<double, dim, dim>;
  using VectorN = Eigen::Matrix<double, dim, 1>;

  // Interpolator class
  const Interpolator<PoseType> interpolator_;

  using LambdaPsiMats = typename Interpolator<PoseType>::LambdaPsiMats;
  using LocalStateVecs = typename Interpolator<PoseType>::LocalStateVecs;
  using StateJacobians = typename Interpolator<PoseType>::StateJacobians;

  // map interpolated state to border states
  std::unordered_map<StateData, std::pair<StateData, StateData>>
      interp_to_borders_map_;
  std::vector<std::pair<StateData, std::pair<StateData, StateData>>>
      interp_to_borders_vec_;
  std::vector<std::pair<StateData, std::shared_ptr<const LambdaPsiMats>>>
      interp_to_LambdaPsi_vec_;

  // Precomputed batches of borders -> indices in interp_to_borders_vec_
  // Each entry contains the border pair and the list of interp indices that
  // share those borders.
  std::vector<std::pair<std::pair<StateData, StateData>, std::vector<size_t>>>
      border_batches_;

  bool fixed_noise_model_ = false;

  std::unordered_set<Key> border_pose_keys_;
  std::unordered_set<Key> border_vel_keys_;

  // Efficient storage for indices of WnoaInterpFactors
  std::unordered_set<size_t> wnoa_interp_factor_indices_;

  /**
   * @brief Evaluate interpolated states from bordering estimated states.
   *
   * Builds a `Values` container containing interpolated pose and velocity
   * entries for every `StateData` known in `interp_to_borders_map_`. When
   * requested, also fills `InterpJacobians` with flattened Jacobian blocks
   * (ordered [LPose,LVel,RPose,RVel]) and `InterpCondCovs` with the
   * conditional covariance for each interpolated state.
   *
   * @param values Outer `Values` providing the bordering estimated states.
   * @param InterpJacobians Optional output pointer populated with flattened
   * jacobian blocks.
   * @param InterpCondCovs Optional output pointer populated with
   * per-interpolated-state covariances.
   * @return Values Container with interpolated pose and velocity entries.
   */
  Values getInterpolatedValues(
      const Values& values,
      std::unordered_map<Key, std::array<Matrix, 4>>* InterpJacobians,
      std::unordered_map<StateData, Matrix2N>* InterpCondCovs = nullptr) const;

 public:
  /**
   * @brief Linearize the graph into a GaussianFactorGraph.
   *
   * This routine produces a linearized Gaussian factor graph evaluated at
   * `linearizationPoint`. It exploits precomputed interpolation data and
   * batching to reduce duplicated interpolation work across wrapper
   * factors.
   *
   * @param linearizationPoint Values at which to linearize the nonlinear graph.
   * @return std::shared_ptr<GaussianFactorGraph> Linearized Gaussian factor
   * graph.
   */
  std::shared_ptr<GaussianFactorGraph> linearize(
      const Values& linearizationPoint) const override;

  /**
   * @brief Compute the unnormalized graph error (sum of factor losses).
   *
   * Computes the scalar error over all factors in the graph. When the
   * graph contains interpolation wrapper factors this method uses the
   * interpolator to evaluate interpolated states as part of the residual
   * computation and can exploit precomputation to improve throughput.
   *
   * @param values Current `Values` used to evaluate the error.
   * @return double Scalar unnormalized error (sum of factor losses).
   */
  double error(const Values& values) const override;

  /// Clone into a shared pointer while preserving WnoaFactorGraph behavior.
  std::shared_ptr<const NonlinearFactorGraph> cloneShared() const override {
    return std::make_shared<WnoaFactorGraph<PoseType>>(*this);
  }

  /**
   * @brief Construct a `WnoaFactorGraph` with interpolation metadata. Note that
   * the interpolation information must be known a priori to properly construct
   * the graph. It is therefore recommended to build a standard
   * `NonlinearFactorGraph` first, then use the interpolateFactorGraph function.
   * Adding factors to the graph after construction is not recommended, but if
   * done, the user is responsible for ensuring that the interpolation metadata
   * is consistent with the factors in the graph.
   *
   * @param interp_map Mapping from each interpolated `StateData` to its
   *        left/right bordering estimated `StateData`.
   * @param q_psd_diag Diagonal PSD vector for the WNOA interpolator (size must
   * match PoseType dimension).
   * @param fixed_noise_model If true, the graph will not augment measurement
   * noise for interpolation.
   */
  WnoaFactorGraph(
      std::unordered_map<StateData, std::pair<StateData, StateData>> interp_map,
      const VectorN q_psd_diag, bool fixed_noise_model = false);
};

/// Utility functions for working with Factor Graphs containing interpolated
/// variables.

/**
 * @brief Convert a factor graph by removing interpolated states.
 *
 * This helper replaces factors that reference interpolated states with
 * equivalent wrapper factors that act on the bordering estimated states.
 * Additionally, WNOA motion prior factors are added between successive
 * estimated states. Factors that do not reference interpolated states are
 * copied unchanged into the returned graph. Note that only a single type of
 * pose is currently supported.
 *
 * @tparam PoseType Pose type used in the graph (e.g. `Pose2`, `Pose3`).
 * @tparam FactorGraphType Type of factor graph to return. Defaults to
 * `NonlinearFactorGraph`, but can be set to `WnoaFactorGraph` to
 * NonlinearFactorGraph or WnoaFactorGraph. Note that the user is responsible
 * for ensuring that the WnoaFactorGraph is defined with the same PoseType as
 * the template parameter.
 * @param graph Input factor graph possibly containing factors on interpolated
 * states.
 * @param estimated_states Ordered set of estimated `StateData` (main-solve
 * states).
 * @param interp_states Ordered set of `StateData` entries to be
 * interpolated/removed.
 * @param q_psd_diag Diagonal PSD vector for the WNOA motion prior (dimension
 * must match PoseType).
 * @param fixed_noise If true, do not augment measurement noise models for
 * interpolation.
 * @return FactorGraphType factor graph with interpolated states removed and
 * factors updated.
 */
template <class PoseType, class FactorGraphType = NonlinearFactorGraph>
FactorGraphType interpolateFactorGraph(
    const NonlinearFactorGraph& graph,
    const std::set<StateData>& estimated_states,
    const std::set<StateData>& interp_states, Vector q_psd_diag,
    bool fixed_noise = false) {
  // assert that the pose is the right kind of variable
  static_assert(
      std::is_same_v<typename traits<PoseType>::structure_category,
                     lie_group_tag> ||
          std::is_same_v<typename traits<PoseType>::structure_category,
                         vector_space_tag>,
      "Pose type must be either a Lie group or vector space");
  // check dimension on the power spectral density matrix
  assert(traits<PoseType>::dimension == q_psd_diag.size());
  // Get map from keys to interpolated state, and interpolated state to
  // estimated state.
  std::unordered_map<Key, StateData> key_to_interp_;
  std::unordered_map<StateData, std::pair<StateData, StateData>>
      interp_to_borders_;
  for (const StateData& state : interp_states) {
    // search for estimated state that upper bound current interpolated state
    auto iter_est_state = estimated_states.lower_bound(state);
    if (iter_est_state == estimated_states.begin()) {
      throw std::runtime_error(
          "Interpolated state time is before all estimated state times");
    } else if (iter_est_state == estimated_states.end()) {
      throw std::runtime_error(
          "Interpolated state time is after all estimated state times");
    } else {
      // decrement iterator (point to left border)
      iter_est_state--;
      // map interp to left border index
      interp_to_borders_[state] =
          std::pair(*iter_est_state, *std::next(iter_est_state));
      // map keys to interp state
      key_to_interp_[state.pose] = state;
      key_to_interp_[state.velocity] = state;
    }
  }
  // Create new factor graph (we use a lambda to handle the case where we need
  // to pass additional info to the constructor, e.g. for WnoaFactorGraph)
  FactorGraphType new_graph = [&]() {
    if constexpr (std::is_same_v<FactorGraphType, WnoaFactorGraph<PoseType>>) {
      return FactorGraphType(interp_to_borders_, q_psd_diag, fixed_noise);
    } else {
      return FactorGraphType();
    }
  }();
  // Add WNOA prior between all estimated states
  auto iter_state = estimated_states.begin();
  while (std::next(iter_state) != estimated_states.end()) {
    StateData state_k = *iter_state;
    StateData state_kp1 = *std::next(iter_state);
    // get time diff
    double delta_t = state_kp1.time - state_k.time;
    // add factor
    auto motion_factor = std::make_shared<WnoaMotionFactor<PoseType>>(
        state_k.pose, state_k.velocity, state_kp1.pose, state_kp1.velocity,
        delta_t, q_psd_diag);
    new_graph.add(motion_factor);
    iter_state++;
  }
  // loop through factors and wrap factors on interpolated states
  for (auto& factor : graph) {
    // handle null factor
    if (!factor) continue;
    // if the factor is a WNOA motion factor, do not add it
    if (std::dynamic_pointer_cast<WnoaMotionFactor<PoseType>>(factor)) continue;
    // get ordered sets of interpolated and estimated states
    std::set<StateData> factor_interp_states;
    std::set<StateData> factor_estimated_states;
    for (Key& key : factor->keys()) {
      // check if key is an interpolated value
      if (key_to_interp_.count(key) > 0) {
        // add indices
        StateData interp_state = key_to_interp_[key];
        factor_interp_states.insert(interp_state);
        auto [left, right] = interp_to_borders_.at(interp_state);
        factor_estimated_states.insert(left);
        factor_estimated_states.insert(right);
      }
    }
    // add factor to new graph
    if (factor_interp_states.size() == 0) {
      // factor does not require interpolation, just add factor as is
      new_graph.add(factor);
    } else {
      // Downcast the NonlinearFactor to a NoiseModelFactor
      auto nmfactor = std::dynamic_pointer_cast<NoiseModelFactor>(factor);
      assert(nmfactor &&
             "Defined factors must be NoiseModelFactor or derivative class");

      // Define and add factor to new graph
      const auto wrapped_factor = std::make_shared<WnoaInterpFactor<PoseType>>(
          nmfactor, factor_estimated_states, factor_interp_states, q_psd_diag,
          fixed_noise);
      new_graph.add(wrapped_factor);
    }
  }

  return new_graph;
}

/**
 * @brief WnoaFactorGraph specialization of `interpolateFactorGraph` for use in
 * Python bindings. Convert a factor graph by removing interpolated states.
 *
 * This helper replaces factors that reference interpolated states with
 * equivalent wrapper factors that act on the bordering estimated states.
 * Additionally, WNOA motion prior factors are added between successive
 * estimated states. Factors that do not reference interpolated states are
 * copied unchanged into the returned graph. Note that only a single type of
 * pose is currently supported.
 *
 * @tparam PoseType Pose type used in the graph (e.g. `Pose2`, `Pose3`).
 * @tparam FactorGraphType Type of factor graph to return. Defaults to
 * `NonlinearFactorGraph`, but can be set to `WnoaFactorGraph` to
 * NonlinearFactorGraph or WnoaFactorGraph. Note that the user is responsible
 * for ensuring that the WnoaFactorGraph is defined with the same PoseType as
 * the template parameter.
 * @param graph Input factor graph possibly containing factors on interpolated
 * states.
 * @param estimated_states Ordered set of estimated `StateData` (main-solve
 * states).
 * @param interp_states Ordered set of `StateData` entries to be
 * interpolated/removed.
 * @param q_psd_diag Diagonal PSD vector for the WNOA motion prior (dimension
 * must match PoseType).
 * @param fixed_noise If true, do not augment measurement noise models for
 * interpolation.
 * @return FactorGraphType factor graph with interpolated states removed and
 * factors updated.
 */
template <class PoseType>
WnoaFactorGraph<PoseType> interpolateWnoaFactorGraph(
    const NonlinearFactorGraph& graph,
    const std::set<StateData>& estimated_states,
    const std::set<StateData>& interp_states, Vector q_psd_diag,
    bool fixed_noise = false) {
  return interpolateFactorGraph<PoseType, WnoaFactorGraph<PoseType>>(
      graph, estimated_states, interp_states, q_psd_diag, fixed_noise);
}

/**
 * @brief Update a `Values` with interpolated pose and velocity entries.
 *
 * Given an interpolated factor graph and the current estimated `Values`,
 * evaluate the interpolator to produce interpolated pose/velocity entries
 * and merge them into a copy of `values` which is returned.
 *
 * @tparam PoseType Pose type used by the interpolator.
 * @param interp_graph Factor graph containing interpolation metadata
 * (borders).
 * @param values Current estimated `Values` (outer states used as borders).
 * @param estim_states Ordered set of estimated `StateData` used by the main
 * solve.
 * @param interp_states Ordered set of `StateData` entries to interpolate.
 * @param q_psd_diag Diagonal PSD vector for the WNOA motion prior.
 * @param covarianceMapOut Optional output pointer to receive
 * per-interpolated-state covariances.
 * @return Values A copy of `values` updated with interpolated pose and
 * velocity entries.
 */
template <class PoseType>
Values updateInterpValues(
    const NonlinearFactorGraph& interp_graph, const Values& values,
    const std::set<StateData>& estim_states,
    const std::set<StateData>& interp_states, const Vector q_psd_diag,
    std::shared_ptr<InterpCovarianceMap> covarianceMapOut = nullptr) {
  // assert that the pose is the right kind of variable
  static_assert(
      std::is_same_v<typename traits<PoseType>::structure_category,
                     lie_group_tag> ||
          std::is_same_v<typename traits<PoseType>::structure_category,
                         vector_space_tag>,
      "Pose type must be either a Lie group or vector space");
  // check dimension on the power spectral density matrix
  assert(traits<PoseType>::dimension == q_psd_diag.size());
  // Define interpolator
  Interpolator<PoseType> interpolator(q_psd_diag);
  // get interpolated values
  Values interp_vals = interpolator.interpolatePosesAndVelocities(
      interp_graph, values, estim_states, interp_states, covarianceMapOut);
  // update values
  Values values_updated(values);
  values_updated.insert(interp_vals);
  return values_updated;
}

/**
 * @brief Update `Values` with interpolated states and return covariances.
 *
 * Convenience wrapper around `updateInterpValues` that always computes
 * interpolation covariances and returns them alongside the updated `Values`.
 *
 * @tparam PoseType Pose type used by the interpolator.
 * @param interp_graph Factor graph containing interpolation metadata
 * (borders).
 * @param values Current estimated `Values` (outer states used as borders).
 * @param estim_states Ordered set of estimated `StateData` used by the main
 * solve.
 * @param interp_states Ordered set of `StateData` entries to interpolate.
 * @param q_psd_diag Diagonal PSD vector for the WNOA motion prior.
 * @return std::pair<Values, InterpCovarianceMap> Updated values and
 * per-interpolated-state covariance map keyed by variable `Key`.
 */
template <class PoseType>
std::pair<Values, InterpCovarianceMap> updateInterpValuesWithCovariance(
    const NonlinearFactorGraph& interp_graph, const Values& values,
    const std::set<StateData>& estim_states,
    const std::set<StateData>& interp_states, const Vector q_psd_diag) {
  auto covariance_map = std::make_shared<InterpCovarianceMap>();
  Values values_updated =
      updateInterpValues<PoseType>(interp_graph, values, estim_states,
                                   interp_states, q_psd_diag, covariance_map);
  return std::make_pair(values_updated, std::move(*covariance_map));
}

}  // namespace gtsam
