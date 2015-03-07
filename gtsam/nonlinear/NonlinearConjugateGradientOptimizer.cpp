/**
 * @file   GradientDescentOptimizer.cpp
 * @brief  
 * @author ydjian
 * @date   Jun 11, 2012
 */

#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <cmath>

using namespace std;

namespace gtsam {

/* Return the gradient vector of a nonlinear factor given a linearization point and a variable ordering
 * Can be moved to NonlinearFactorGraph.h if desired */
void gradientInPlace(const NonlinearFactorGraph &nfg, const Values &values, const Ordering &ordering, VectorValues &g) {
  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = nfg.linearize(values, ordering);
  FactorGraph<JacobianFactor> jfg;  jfg.reserve(linear->size());
  BOOST_FOREACH(const GaussianFactorGraph::sharedFactor& factor, *linear) {
    if(boost::shared_ptr<JacobianFactor> jf = boost::dynamic_pointer_cast<JacobianFactor>(factor))
      jfg.push_back((jf));
    else
      jfg.push_back(boost::make_shared<JacobianFactor>(*factor));
  }
  // compute the gradient direction
  gradientAtZero(jfg, g);
}

double NonlinearConjugateGradientOptimizer::System::error(const State &state) const {
  return graph_.error(state);
}

NonlinearConjugateGradientOptimizer::System::Gradient NonlinearConjugateGradientOptimizer::System::gradient(const State &state) const {
  Gradient result = state.zeroVectors(ordering_);
  gradientInPlace(graph_, state, ordering_, result);
  return result;
}
NonlinearConjugateGradientOptimizer::System::State NonlinearConjugateGradientOptimizer::System::advance(const State &current, const double alpha, const Gradient &g) const {
  Gradient step = g;
  step.vector() *= alpha;
  return current.retract(step, ordering_);
}

void NonlinearConjugateGradientOptimizer::iterate() {
  size_t dummy ;
  boost::tie(state_.values, dummy) = nonlinearConjugateGradient<System, Values>(System(graph_, *ordering_), state_.values, params_, true /* single iterations */);
  ++state_.iterations;
  state_.error = graph_.error(state_.values);
}

const Values& NonlinearConjugateGradientOptimizer::optimize() {
  boost::tie(state_.values, state_.iterations) = nonlinearConjugateGradient<System, Values>(System(graph_, *ordering_), state_.values, params_, false /* up to convergent */);
  state_.error = graph_.error(state_.values);
  return state_.values;
}

} /* namespace gtsam */
