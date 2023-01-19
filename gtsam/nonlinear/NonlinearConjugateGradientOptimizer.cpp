/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   NonlinearConjugateGradientOptimizer.cpp
 * @brief  Simple non-linear optimizer that solves using *non-preconditioned* CG
 * @author Yong-Dian Jian
 * @date   Jun 11, 2012
 */

#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <cmath>

namespace gtsam {

typedef internal::NonlinearOptimizerState State;

/**
 * @brief Return the gradient vector of a nonlinear factor graph
 * @param nfg the graph
 * @param values a linearization point
 * Can be moved to NonlinearFactorGraph.h if desired
 */
static VectorValues gradientInPlace(const NonlinearFactorGraph &nfg,
    const Values &values) {
  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = nfg.linearize(values);
  return linear->gradientAtZero();
}

NonlinearConjugateGradientOptimizer::NonlinearConjugateGradientOptimizer(
    const NonlinearFactorGraph& graph, const Values& initialValues, const Parameters& params)
    : Base(graph, std::unique_ptr<State>(new State(initialValues, graph.error(initialValues)))),
    params_(params) {}

double NonlinearConjugateGradientOptimizer::System::error(const State& state) const {
  return graph_.error(state);
}

NonlinearConjugateGradientOptimizer::System::Gradient NonlinearConjugateGradientOptimizer::System::gradient(
    const State &state) const {
  return gradientInPlace(graph_, state);
}

NonlinearConjugateGradientOptimizer::System::State NonlinearConjugateGradientOptimizer::System::advance(
    const State &current, const double alpha, const Gradient &g) const {
  Gradient step = g;
  step *= alpha;
  return current.retract(step);
}

GaussianFactorGraph::shared_ptr NonlinearConjugateGradientOptimizer::iterate() {
  Values newValues;
  int dummy;
  std::tie(newValues, dummy) = nonlinearConjugateGradient<System, Values>(
      System(graph_), state_->values, params_, true /* single iteration */);
  state_.reset(new State(newValues, graph_.error(newValues), state_->iterations + 1));

  // NOTE(frank): We don't linearize this system, so we must return null here.
  return nullptr;
}

const Values& NonlinearConjugateGradientOptimizer::optimize() {
  // Optimize until convergence
  System system(graph_);
  Values newValues;
  int iterations;
  std::tie(newValues, iterations) =
      nonlinearConjugateGradient(system, state_->values, params_, false);
  state_.reset(new State(std::move(newValues), graph_.error(newValues), iterations));
  return state_->values;
}

} /* namespace gtsam */

