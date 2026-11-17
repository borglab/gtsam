/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StagQuadraticCost.h
 * @brief   Quadratic state and control cost for the future half of the STAG
 *          graph
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam_unstable/mpc_stag/cost/StagCost.h>

#include <cstddef>
#include <type_traits>

namespace gtsam {

/**
 * Quadratic state and control cost towards a fixed goal:
 * - running cost of step k: 1/2 ||Local(x_k, goal)||^2_Q + 1/2 ||u_k||^2_R,
 *   a PriorFactor on x_k at the goal and a PriorFactor on u_k at zero;
 * - terminal cost: 1/2 ||Local(x_H, goal)||^2_Qf, a PriorFactor on x_H at the
 *   goal.
 * Here ||e||^2_W = e^T W e. The cost is the same for every step and time.
 *
 * @tparam STATE    State type, a GTSAM manifold type.
 * @tparam CONTROL  Control type, a fixed-size Eigen vector such as Vector2
 *                  (Vector1 for a scalar control), so that zero is defined.
 */
template <class STATE, class CONTROL>
class StagQuadraticCost : public StagCost<STATE, CONTROL> {
  static_assert(traits<CONTROL>::dimension != Eigen::Dynamic &&
                    std::is_same_v<CONTROL,
                                   Eigen::Matrix<double,
                                                 traits<CONTROL>::dimension,
                                                 1>>,
                "StagQuadraticCost: CONTROL must be a fixed-size Eigen "
                "vector, e.g. Vector2.");

 public:
  /// State dimension.
  static constexpr int StateDim = traits<STATE>::dimension;

  /// Control dimension.
  static constexpr int ControlDim = traits<CONTROL>::dimension;

  /// Weight on the state error, StateDim x StateDim.
  using StateWeight = Eigen::Matrix<double, StateDim, StateDim>;

  /// Weight on the control, ControlDim x ControlDim.
  using ControlWeight = Eigen::Matrix<double, ControlDim, ControlDim>;

  /**
   * Construct from the goal state, the running weights Q (state) and R
   * (control), and the terminal weight Qf. Each weight must be symmetric
   * and either positive definite or diagonal with non-negative entries (a
   * zero entry ignores that component). The weights are not checked, and
   * other weights give a wrong cost.
   */
  StagQuadraticCost(const STATE& goal, const StateWeight& Q,
                    const ControlWeight& R, const StateWeight& Qf)
      : goal_(goal),
        stateNoise_(noiseModel::Gaussian::Information(Q)),
        controlNoise_(noiseModel::Gaussian::Information(R)),
        terminalNoise_(noiseModel::Gaussian::Information(Qf)) {}

  /// Priors on x_k at the goal (Q) and on u_k at zero (R).
  NonlinearFactorGraph runningCost(size_t /*k*/, Key stateKey, Key controlKey,
                                   double /*time*/) const override {
    NonlinearFactorGraph graph;
    graph.emplace_shared<PriorFactor<STATE>>(stateKey, goal_, stateNoise_);
    graph.emplace_shared<PriorFactor<CONTROL>>(controlKey, CONTROL::Zero(),
                                               controlNoise_);
    return graph;
  }

  /// Prior on x_H at the goal (Qf).
  NonlinearFactorGraph terminalCost(Key stateKey,
                                    double /*time*/) const override {
    NonlinearFactorGraph graph;
    graph.emplace_shared<PriorFactor<STATE>>(stateKey, goal_, terminalNoise_);
    return graph;
  }

 private:
  const STATE goal_;                      ///< Goal state.
  const SharedNoiseModel stateNoise_;     ///< Q, as a noise model.
  const SharedNoiseModel controlNoise_;   ///< R, as a noise model.
  const SharedNoiseModel terminalNoise_;  ///< Qf, as a noise model.
};

}  // namespace gtsam
