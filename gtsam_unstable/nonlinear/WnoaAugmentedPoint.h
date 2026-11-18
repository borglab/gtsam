/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file WnoaAugmentedPoint.h
 * @brief Dimension overloads of the WNOA graph helpers for AugmentedPoint.
 * @author Oluwatodimu Afolayan
 */
#pragma once

#include <gtsam/nonlinear/WnoaFactorGraph.h>
#include <gtsam_unstable/geometry/AugmentedPoint.h>

#include <set>

namespace gtsam {

/**
 * interpolateFactorGraph for AugmentedPoint<N> positions, selected by the
 * dimension N so that wrappers avoid nested template arguments.
 *
 * The state partition selects the problem. With the knots as estimated states
 * and the measurement-time states as interpolated states, factors on the
 * measurement-time states are wrapped in WnoaInterpFactor. With every distinct
 * state estimated and none interpolated, the factors stay on their own states,
 * and consecutive states are joined by WNOA motion factors (the full Gaussian
 * process formulation).
 */
template <int N>
NonlinearFactorGraph interpolateFactorGraph(
    const NonlinearFactorGraph& graph,
    const std::set<StateData>& estimated_states,
    const std::set<StateData>& interp_states, Vector q_psd_diag,
    bool fixed_noise = false) {
  return interpolateFactorGraph<AugmentedPoint<N>>(
      graph, estimated_states, interp_states, q_psd_diag, fixed_noise);
}

/**
 * updateInterpValues for AugmentedPoint<N> positions, selected by the
 * dimension N so that wrappers avoid nested template arguments. Inserts the
 * WNOA conditional mean of each interpolated state.
 */
template <int N>
Values updateInterpValues(const NonlinearFactorGraph& interp_graph,
                          const Values& values,
                          const std::set<StateData>& estim_states,
                          const std::set<StateData>& interp_states,
                          const Vector q_psd_diag) {
  return updateInterpValues<AugmentedPoint<N>>(
      interp_graph, values, estim_states, interp_states, q_psd_diag);
}

}  // namespace gtsam
