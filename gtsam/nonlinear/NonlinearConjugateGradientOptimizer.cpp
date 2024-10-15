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

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

#include <cmath>

namespace gtsam {

typedef internal::NonlinearOptimizerState State;

/**
 * @brief Return the gradient vector of a nonlinear factor graph
 * @param nfg the graph
 * @param values a linearization point
 * Can be moved to NonlinearFactorGraph.h if desired
 */
static VectorValues gradientInPlace(const NonlinearFactorGraph& nfg,
                                    const Values& values) {
  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = nfg.linearize(values);
  return linear->gradientAtZero();
}

NonlinearConjugateGradientOptimizer::NonlinearConjugateGradientOptimizer(
    const NonlinearFactorGraph& graph, const Values& initialValues,
    const Parameters& params)
    : Base(graph, std::unique_ptr<State>(
                      new State(initialValues, graph.error(initialValues)))),
      params_(params) {}

double NonlinearConjugateGradientOptimizer::error(const Values& state) const {
  return graph_.error(state);
}

VectorValues NonlinearConjugateGradientOptimizer::gradient(
    const Values& state) const {
  return gradientInPlace(graph_, state);
}

Values NonlinearConjugateGradientOptimizer::advance(
    const Values& current, const double alpha,
    const VectorValues& gradient) const {
  return current.retract(alpha * gradient);
}

GaussianFactorGraph::shared_ptr NonlinearConjugateGradientOptimizer::iterate() {
  const auto [newValues, dummy] = nonlinearConjugateGradient(
      state_->values, params_, true /* single iteration */);
  state_.reset(
      new State(newValues, graph_.error(newValues), state_->iterations + 1));

  // NOTE(frank): We don't linearize this system, so we must return null here.
  return nullptr;
}

const Values& NonlinearConjugateGradientOptimizer::optimize() {
  // Optimize until convergence
  const auto [newValues, iterations] =
      nonlinearConjugateGradient(state_->values, params_, false);
  state_.reset(
      new State(std::move(newValues), graph_.error(newValues), iterations));
  return state_->values;
}

double NonlinearConjugateGradientOptimizer::lineSearch(
    const Values& currentValues, const VectorValues& gradient) const {
  /* normalize it such that it becomes a unit vector */
  const double g = gradient.norm();

  // perform the golden section search algorithm to decide the the optimal
  // step size detail refer to
  // http://en.wikipedia.org/wiki/Golden_section_search
  const double phi = 0.5 * (1.0 + std::sqrt(5.0)), resphi = 2.0 - phi,
               tau = 1e-5;
  double minStep = -1.0 / g, maxStep = 0,
         newStep = minStep + (maxStep - minStep) / (phi + 1.0);

  Values newValues = advance(currentValues, newStep, gradient);
  double newError = error(newValues);

  while (true) {
    const bool flag = (maxStep - newStep > newStep - minStep) ? true : false;
    const double testStep = flag ? newStep + resphi * (maxStep - newStep)
                                 : newStep - resphi * (newStep - minStep);

    if ((maxStep - minStep) < tau * (std::abs(testStep) + std::abs(newStep))) {
      return 0.5 * (minStep + maxStep);
    }

    const Values testValues = advance(currentValues, testStep, gradient);
    const double testError = error(testValues);

    // update the working range
    if (testError >= newError) {
      if (flag)
        maxStep = testStep;
      else
        minStep = testStep;
    } else {
      if (flag) {
        minStep = newStep;
        newStep = testStep;
        newError = testError;
      } else {
        maxStep = newStep;
        newStep = testStep;
        newError = testError;
      }
    }
  }
  return 0.0;
}

std::tuple<Values, int>
NonlinearConjugateGradientOptimizer::nonlinearConjugateGradient(
    const Values& initial, const NonlinearOptimizerParams& params,
    const bool singleIteration, const bool gradientDescent) const {
  size_t iteration = 0;

  // check if we're already close enough
  double currentError = error(initial);
  if (currentError <= params.errorTol) {
    if (params.verbosity >= NonlinearOptimizerParams::ERROR) {
      std::cout << "Exiting, as error = " << currentError << " < "
                << params.errorTol << std::endl;
    }
    return {initial, iteration};
  }

  Values currentValues = initial;
  VectorValues currentGradient = gradient(currentValues), prevGradient,
               direction = currentGradient;

  /* do one step of gradient descent */
  Values prevValues = currentValues;
  double prevError = currentError;
  double alpha = lineSearch(currentValues, direction);
  currentValues = advance(prevValues, alpha, direction);
  currentError = error(currentValues);

  // Maybe show output
  if (params.verbosity >= NonlinearOptimizerParams::ERROR)
    std::cout << "Initial error: " << currentError << std::endl;

  // Iterative loop
  do {
    if (gradientDescent == true) {
      direction = gradient(currentValues);
    } else {
      prevGradient = currentGradient;
      currentGradient = gradient(currentValues);
      // Polak-Ribiere: beta = g'*(g_n-g_n-1)/g_n-1'*g_n-1
      const double beta =
          std::max(0.0, currentGradient.dot(currentGradient - prevGradient) /
                            prevGradient.dot(prevGradient));
      direction = currentGradient + (beta * direction);
    }

    alpha = lineSearch(currentValues, direction);

    prevValues = currentValues;
    prevError = currentError;

    currentValues = advance(prevValues, alpha, direction);
    currentError = error(currentValues);

    // User hook:
    if (params.iterationHook)
      params.iterationHook(iteration, prevError, currentError);

    // Maybe show output
    if (params.verbosity >= NonlinearOptimizerParams::ERROR)
      std::cout << "iteration: " << iteration
                << ", currentError: " << currentError << std::endl;
  } while (++iteration < params.maxIterations && !singleIteration &&
           !checkConvergence(params.relativeErrorTol, params.absoluteErrorTol,
                             params.errorTol, prevError, currentError,
                             params.verbosity));

  // Printing if verbose
  if (params.verbosity >= NonlinearOptimizerParams::ERROR &&
      iteration >= params.maxIterations)
    std::cout << "nonlinearConjugateGradient: Terminating because reached "
                 "maximum iterations"
              << std::endl;

  return {currentValues, iteration};
}

} /* namespace gtsam */
