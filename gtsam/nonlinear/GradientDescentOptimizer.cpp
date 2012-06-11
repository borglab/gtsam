/**
 * @file   GradientDescentOptimizer.cpp
 * @brief  
 * @author ydjian
 * @date   Jun 11, 2012
 */

#include <cmath>

#include <gtsam/nonlinear/GradientDescentOptimizer.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/JacobianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <gtsam_unstable/linear/iterative.h>

using namespace std;

namespace gtsam {

/**
 * Return the gradient vector of a nonlinear factor given a linearization point and a variable ordering
 * Can be moved to NonlinearFactorGraph.h if desired
 */
void gradientInPlace(const NonlinearFactorGraph &nfg, const Values &values, const Ordering &ordering, VectorValues &g) {

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = nfg.linearize(values, ordering);
  const FactorGraph<JacobianFactor>::shared_ptr jfg = linear->dynamicCastFactors<FactorGraph<JacobianFactor> >();

  // compute the gradient direction
  gtsam::gradientAtZero(*jfg, g);
}


/* ************************************************************************* */
void GradientDescentOptimizer::iterate() {


  // Pull out parameters we'll use
  const NonlinearOptimizerParams::Verbosity nloVerbosity = params_.verbosity;

  // compute the gradient vector
  gradientInPlace(graph_, state_.values, *ordering_, *gradient_);

  /* normalize it such that it becomes a unit vector */
  const double g = gradient_->vector().norm();
  gradient_->vector() /= g;

  // perform the golden section search algorithm to decide the the optimal step size
  // detail refer to http://en.wikipedia.org/wiki/Golden_section_search
  VectorValues step = VectorValues::SameStructure(*gradient_);
  const double phi = 0.5*(1.0+std::sqrt(5.0)), resphi = 2.0 - phi, tau = 1e-5;
  double minStep = -1.0, maxStep = 0,
         newStep = minStep + (maxStep-minStep) / (phi+1.0) ;

  step.vector() = newStep * gradient_->vector();
  Values newValues = state_.values.retract(step, *ordering_);
  double newError = graph_.error(newValues);

  if ( nloVerbosity ) {
    std::cout << "minStep = " << minStep << ", maxStep = " << maxStep << ", newStep = "  << newStep << ", newError = " << newError << std::endl;
  }

  while (true) {
    const bool flag = (maxStep - newStep > newStep - minStep) ? true : false ;
    const double testStep = flag ?
                            newStep + resphi * (maxStep - newStep) : newStep - resphi * (newStep - minStep);

    if ( (maxStep- minStep) < tau * (std::fabs(testStep) + std::fabs(newStep)) ) {
      newStep = 0.5*(minStep+maxStep);
      step.vector() = newStep * gradient_->vector();
      newValues = state_.values.retract(step, *ordering_);
      newError = graph_.error(newValues);

      if ( newError < state_.error ) {
        state_.values = state_.values.retract(step, *ordering_);
        state_.error = graph_.error(state_.values);
      }

      break;
    }

    step.vector() = testStep * gradient_->vector();
    const Values testValues = state_.values.retract(step, *ordering_);
    const double testError = graph_.error(testValues);

    // update the working range
    if ( testError < newError ) {
      if ( flag ) {
        minStep = newStep;
        newStep = testStep;
        newError = testError;
      }
      else {
        maxStep = newStep;
        newStep = testStep;
        newError = testError;
      }
    }
    else {
      if ( flag ) {
        maxStep = testStep;
      }
      else {
        minStep = testStep;
      }
    }

    if ( nloVerbosity ) {
      std::cout << "minStep = " << minStep << ", maxStep = " << maxStep << ", newStep = "  << newStep << ", newError = " << newError << std::endl;
    }
  }

  // Increment the iteration counter
  ++state_.iterations;
}

} /* namespace gtsam */
