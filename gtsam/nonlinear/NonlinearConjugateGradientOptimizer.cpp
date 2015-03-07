/**
 * @file   GradientDescentOptimizer.cpp
 * @brief  
 * @author ydjian
 * @date   Jun 11, 2012
 */

#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <cmath>

using namespace std;

namespace gtsam {

/* Return the gradient vector of a nonlinear factor given a linearization point and a variable ordering
 * Can be moved to NonlinearFactorGraph.h if desired */
VectorValues gradientInPlace(const NonlinearFactorGraph &nfg, const Values &values) {
  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = nfg.linearize(values);
  return linear->gradientAtZero();
}

double NonlinearConjugateGradientOptimizer::System::error(const State &state) const {
  return graph_.error(state);
}

NonlinearConjugateGradientOptimizer::System::Gradient NonlinearConjugateGradientOptimizer::System::gradient(const State &state) const {
  return gradientInPlace(graph_, state);
}
NonlinearConjugateGradientOptimizer::System::State NonlinearConjugateGradientOptimizer::System::advance(const State &current, const double alpha, const Gradient &g) const {
  Gradient step = g;
  step *= alpha;
  return current.retract(step);
}

void NonlinearConjugateGradientOptimizer::iterate() {
  int dummy ;
  boost::tie(state_.values, dummy) = nonlinearConjugateGradient<System, Values>(System(graph_), state_.values, params_, true /* single iterations */);
  ++state_.iterations;
  state_.error = graph_.error(state_.values);
}

const Values& NonlinearConjugateGradientOptimizer::optimize() {
  boost::tie(state_.values, state_.iterations) = nonlinearConjugateGradient<System, Values>(System(graph_), state_.values, params_, false /* up to convergent */);
  state_.error = graph_.error(state_.values);
  return state_.values;
}

} /* namespace gtsam */

