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
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void GaussNewtonOptimizer::iterate() {

  const NonlinearOptimizerState& current = state_;

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = graph_.linearize(current.values);

  // Solve Factor Graph
  const VectorValues delta = solve(*linear, current.values, params_);

  // Maybe show output
  if(params_.verbosity >= NonlinearOptimizerParams::DELTA) delta.print("delta");

  // Create new state with new values and new error
  state_.values = current.values.retract(delta);
  state_.error = graph_.error(state_.values);
  ++ state_.iterations;
}

/* ************************************************************************* */
GaussNewtonParams GaussNewtonOptimizer::ensureHasOrdering(
    GaussNewtonParams params, const NonlinearFactorGraph& graph) const {
  if (!params.ordering)
    params.ordering = Ordering::Create(params.orderingType, graph);
  return params;
}

} /* namespace gtsam */
