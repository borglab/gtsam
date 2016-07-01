/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussNewtonOptimizer.cpp
 * @brief
 * @author  Richard Roberts
 * @date   Feb 26, 2012
 */

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

typedef internal::NonlinearOptimizerState State;

/* ************************************************************************* */
GaussNewtonOptimizer::GaussNewtonOptimizer(const NonlinearFactorGraph& graph,
                                           const Values& initialValues,
                                           const GaussNewtonParams& params)
    : NonlinearOptimizer(
          graph, std::unique_ptr<State>(new State(initialValues, graph.error(initialValues)))),
      params_(ensureHasOrdering(params, graph)) {}

GaussNewtonOptimizer::GaussNewtonOptimizer(const NonlinearFactorGraph& graph,
                                           const Values& initialValues, const Ordering& ordering)
    : NonlinearOptimizer(
          graph, std::unique_ptr<State>(new State(initialValues, graph.error(initialValues)))) {
  params_.ordering = ordering;
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr GaussNewtonOptimizer::iterate() {
  gttic(GaussNewtonOptimizer_Iterate);

  // Linearize graph
  gttic(GaussNewtonOptimizer_Linearize);
  GaussianFactorGraph::shared_ptr linear = graph_.linearize(state_->values);
  gttoc(GaussNewtonOptimizer_Linearize);

  // Solve Factor Graph
  gttic(GaussNewtonOptimizer_Solve);
  const VectorValues delta = solve(*linear, params_);
  gttoc(GaussNewtonOptimizer_Solve);

  // Maybe show output
  if (params_.verbosity >= NonlinearOptimizerParams::DELTA)
    delta.print("delta");

  // Create new state with new values and new error
  Values newValues = state_->values.retract(delta);
  state_.reset(new State(std::move(newValues), graph_.error(newValues), state_->iterations + 1));

  return linear;
}

/* ************************************************************************* */
GaussNewtonParams GaussNewtonOptimizer::ensureHasOrdering(
    GaussNewtonParams params, const NonlinearFactorGraph& graph) const {
  if (!params.ordering)
    params.ordering = Ordering::Create(params.orderingType, graph);
  return params;
}

} /* namespace gtsam */
