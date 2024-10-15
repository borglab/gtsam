/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   NonlinearConjugateGradientOptimizer.h
 * @brief  Simple non-linear optimizer that solves using *non-preconditioned* CG
 * @author Yong-Dian Jian
 * @date   June 11, 2012
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/**  An implementation of the nonlinear CG method using the template below */
class GTSAM_EXPORT NonlinearConjugateGradientOptimizer
    : public NonlinearOptimizer {
 public:
  typedef NonlinearOptimizer Base;
  typedef NonlinearOptimizerParams Parameters;
  typedef std::shared_ptr<NonlinearConjugateGradientOptimizer> shared_ptr;

 protected:
  Parameters params_;

  const NonlinearOptimizerParams &_params() const override { return params_; }

 public:
  /// Constructor
  NonlinearConjugateGradientOptimizer(const NonlinearFactorGraph &graph,
                                      const Values &initialValues,
                                      const Parameters &params = Parameters());

  /// Destructor
  ~NonlinearConjugateGradientOptimizer() override {}

  double error(const Values &state) const;

  VectorValues gradient(const Values &state) const;

  Values advance(const Values &current, const double alpha,
                 const VectorValues &g) const;

  /**
   * Perform a single iteration, returning GaussianFactorGraph corresponding to
   * the linearized factor graph.
   */
  GaussianFactorGraph::shared_ptr iterate() override;

  /**
   * Optimize for the maximum-likelihood estimate, returning a the optimized
   * variable assignments.
   */
  const Values &optimize() override;

  /** Implement the golden-section line search algorithm */
  double lineSearch(const Values &currentValues, const VectorValues &gradient) const {
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

      if ((maxStep - minStep) <
          tau * (std::abs(testStep) + std::abs(newStep))) {
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

  /**
   * Implement the nonlinear conjugate gradient method using the Polak-Ribiere
   * formula suggested in
   * http://en.wikipedia.org/wiki/Nonlinear_conjugate_gradient_method.
   *
   * The last parameter is a switch between gradient-descent and conjugate
   * gradient
   */
  std::tuple<Values, int> nonlinearConjugateGradient(
      const Values &initial, const NonlinearOptimizerParams &params,
      const bool singleIteration, const bool gradientDescent = false) const {
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
};

}  // namespace gtsam
