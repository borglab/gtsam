/**
 * @file   GradientDescentOptimizer.cpp
 * @brief
 * @author ydjian
 * @date   Jun 11, 2012
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/* an implementation of gradient-descent method using the NLO interface */

class GradientDescentState : public NonlinearOptimizerState {

public:

  typedef NonlinearOptimizerState Base;

  GradientDescentState(const NonlinearFactorGraph& graph, const Values& values)
    : Base(graph, values) {}
};

class GradientDescentOptimizer : public NonlinearOptimizer {

public:

  typedef boost::shared_ptr<GradientDescentOptimizer> shared_ptr;
  typedef NonlinearOptimizer Base;
  typedef GradientDescentState States;
  typedef NonlinearOptimizerParams Parameters;

protected:

  Parameters params_;
  States state_;
  Ordering::shared_ptr ordering_;
  VectorValues::shared_ptr gradient_;

public:

  GradientDescentOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues, const Parameters& params = Parameters())
    : Base(graph), params_(params), state_(graph, initialValues),
      ordering_(initialValues.orderingArbitrary()),
      gradient_(new VectorValues(initialValues.zeroVectors(*ordering_))) {}

  virtual ~GradientDescentOptimizer() {}

  virtual void iterate();

protected:

  virtual const NonlinearOptimizerState& _state() const { return state_; }
  virtual const NonlinearOptimizerParams& _params() const { return params_; }
};


/**
 *  An implementation of the nonlinear cg method using the template below
 */

class ConjugateGradientOptimizer {

  class System {

  public:

    typedef Values State;
    typedef VectorValues Gradient;

  protected:

    NonlinearFactorGraph graph_;
    Ordering ordering_;

  public:

    System(const NonlinearFactorGraph &graph, const Ordering &ordering): graph_(graph), ordering_(ordering) {}
    double error(const State &state) const ;
    Gradient gradient(const State &state) const ;
    State advance(const State &current, const double alpha, const Gradient &g) const ;
  };

public:

  typedef NonlinearOptimizerParams Parameters;
  typedef boost::shared_ptr<ConjugateGradientOptimizer> shared_ptr;

protected:

  NonlinearFactorGraph graph_;
  Values initialEstimate_;
  Parameters params_;
  Ordering::shared_ptr ordering_;
  VectorValues::shared_ptr gradient_;
  bool cg_;

public:

  ConjugateGradientOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
                             const Parameters& params = Parameters(), const bool cg = true)
    : graph_(graph), initialEstimate_(initialValues), params_(params),
      ordering_(initialValues.orderingArbitrary()),
      gradient_(new VectorValues(initialValues.zeroVectors(*ordering_))),
      cg_(cg) {}

  virtual ~ConjugateGradientOptimizer() {}
  virtual Values optimize () ;
};

/**
 * Implement the golden-section line search algorithm
 */
template <class S, class V, class W>
double lineSearch(const S &system, const V currentValues, const W &gradient) {

  /* normalize it such that it becomes a unit vector */
  const double g = gradient.norm();

  // perform the golden section search algorithm to decide the the optimal step size
  // detail refer to http://en.wikipedia.org/wiki/Golden_section_search
  const double phi = 0.5*(1.0+std::sqrt(5.0)), resphi = 2.0 - phi, tau = 1e-5;
  double minStep = -1.0/g, maxStep = 0,
         newStep = minStep + (maxStep-minStep) / (phi+1.0) ;

  V newValues = system.advance(currentValues, newStep, gradient);
  double newError = system.error(newValues);

  while (true) {
    const bool flag = (maxStep - newStep > newStep - minStep) ? true : false ;
    const double testStep = flag ?
                            newStep + resphi * (maxStep - newStep) : newStep - resphi * (newStep - minStep);

    if ( (maxStep- minStep) < tau * (std::fabs(testStep) + std::fabs(newStep)) ) {
      return  0.5*(minStep+maxStep);
    }

    const V testValues = system.advance(currentValues, testStep, gradient);
    const double testError = system.error(testValues);

    // update the working range
    if ( testError >= newError ) {
      if ( flag ) maxStep = testStep;
      else minStep = testStep;
    }
    else {
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
  }
  return 0.0;
}

/**
 * Implement the nonlinear conjugate gradient method using the Polak-Ribieve formula suggested in
 * http://en.wikipedia.org/wiki/Nonlinear_conjugate_gradient_method.
 *
 * The S (system) class requires three member functions: error(state), gradient(state) and
 * advance(state, step-size, direction). The V class denotes the state or the solution.
 *
 * The last parameter is a switch between gradient-descent and conjugate gradient
 */

template <class S, class V>
V conjugateGradient(const S &system, const V &initial, const NonlinearOptimizerParams &params, const bool gradientDescent) {

  GTSAM_CONCEPT_MANIFOLD_TYPE(V);

  // check if we're already close enough
  double currentError = system.error(initial);
  if(currentError <= params.errorTol) {
    if (params.verbosity >= NonlinearOptimizerParams::ERROR)
      std::cout << "Exiting, as error = " << currentError << " < " << params.errorTol << std::endl;
    return initial;
  }

  V currentValues = initial;
  typename S::Gradient currentGradient = system.gradient(currentValues), prevGradient,
                       direction = currentGradient;

  /* do one step of gradient descent */
  V prevValues = currentValues; double prevError = currentError;
  double alpha = lineSearch(system, currentValues, direction);
  currentValues = system.advance(prevValues, alpha, direction);
  currentError = system.error(currentValues);
  Index iteration = 0;

  // Maybe show output
  if (params.verbosity >= NonlinearOptimizerParams::ERROR) std::cout << "Initial error: " << currentError << std::endl;

  // Iterative loop
  do {

    if ( gradientDescent == true) {
      direction = system.gradient(currentValues);
    }
    else {
      prevGradient = currentGradient;
      currentGradient = system.gradient(currentValues);
      const double beta = std::max(0.0, currentGradient.dot(currentGradient-prevGradient)/currentGradient.dot(currentGradient));
      direction = currentGradient + (beta*direction);
    }

    alpha = lineSearch(system, currentValues, direction);

    prevValues = currentValues; prevError = currentError;

    currentValues = system.advance(prevValues, alpha, direction);
    currentError = system.error(currentValues);

    // Maybe show output
    if(params.verbosity >= NonlinearOptimizerParams::ERROR) std::cout << "currentError: " << currentError << std::endl;
  } while( ++iteration < params.maxIterations &&
           !checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol, prevError, currentError, params.verbosity));

  // Printing if verbose
  if (params.verbosity >= NonlinearOptimizerParams::ERROR && iteration >= params.maxIterations)
    std::cout << "Terminating because reached maximum iterations" << std::endl;

  return currentValues;
}






}
