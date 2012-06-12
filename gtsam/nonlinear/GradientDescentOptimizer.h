/**
 * @file   GradientDescentOptimizer.cpp
 * @brief
 * @author ydjian
 * @date   Jun 11, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/* an implementation of gradient-descent method using the NLO interface */

class GradientDescentParams : public NonlinearOptimizerParams {

public:
  typedef NonlinearOptimizerParams Base;

  GradientDescentParams():Base() {}
  virtual void print(const std::string& str = "") const {
    Base::print(str);
  }

  virtual ~GradientDescentParams() {}
};

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
  typedef GradientDescentParams Parameters;

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


#include <gtsam/linear/IterativeSolver.h>

/* Yet another implementation of the gradient-descent method using the iterative.h interface */
class GradientDescentParams2 : public NonlinearOptimizerParams {
public:
  GradientDescentParams2(){}
};

class GradientDescentOptimizer2 {

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

  typedef GradientDescentParams2 Parameters;
  typedef boost::shared_ptr<GradientDescentOptimizer2> shared_ptr;

protected:

  NonlinearFactorGraph graph_;
  Values initialEstimate_;
  Parameters params_;
  Ordering::shared_ptr ordering_;
  VectorValues::shared_ptr gradient_;

public:

  GradientDescentOptimizer2(const NonlinearFactorGraph& graph, const Values& initialValues, const Parameters& params = Parameters())
    : graph_(graph), initialEstimate_(initialValues), params_(params),
      ordering_(initialValues.orderingArbitrary()),
      gradient_(new VectorValues(initialValues.zeroVectors(*ordering_))) {}

  virtual ~GradientDescentOptimizer2() {}

  virtual Values optimize () ;
};


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

template <class S, class V>
V gradientDescent(const S &system, const V &initial, const NonlinearOptimizerParams &params) {

  V currentValues = initial, prevValues;
  double currentError = system.error(currentValues), prevError;
  Index iteration = 0;

  // check if we're already close enough
  if(currentError <= params.errorTol) {
    if (params.verbosity >= NonlinearOptimizerParams::ERROR)
      std::cout << "Exiting, as error = " << currentError << " < " << params.errorTol << std::endl;
    return currentValues;
  }

  // Maybe show output
  if (params.verbosity >= NonlinearOptimizerParams::ERROR) std::cout << "Initial error: " << currentError << std::endl;

  // Iterative loop
  do {
    // Do next iteration
    const typename S::Gradient gradient = system.gradient(currentValues);
    const double alpha = lineSearch(system, currentValues, gradient);

    prevValues = currentValues;
    prevError = currentError;

    currentValues = system.advance(prevValues, alpha, gradient);
    currentError = system.error(currentValues);

    // Maybe show output
    if(params.verbosity >= NonlinearOptimizerParams::ERROR) std::cout << "currentError: " << currentError << std::endl;
  } while( ++iteration < params.maxIterations &&
           !checkConvergence(params.relativeErrorTol, params.absoluteErrorTol,
            params.errorTol, prevError, currentError, params.verbosity));

  // Printing if verbose
  if (params.verbosity >= NonlinearOptimizerParams::ERROR && iteration >= params.maxIterations)
    std::cout << "Terminating because reached maximum iterations" << std::endl;

  return currentValues;
}

}
