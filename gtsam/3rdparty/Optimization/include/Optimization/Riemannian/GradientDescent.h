/** This header file provides a lightweight template function implementing
 * gradient descent on a Riemannian manifold.
 *
 * Copyright (C) 2017 - 2022 by David M. Rosen (dmrosen@mit.edu)
 */

#pragma once

#include <iostream>
#include <limits>
#include <math.h>
#include <optional>

#include "Optimization/Riemannian/Concepts.h"
#include "Optimization/Util/Stopwatch.h"

namespace Optimization {
namespace Riemannian {

/** An alias template for a user-definable function that can be
 * used to access various interesting bits of information about the internal
 * state of a gradient descent algorithm as it runs.  More
 * precisely, this function is called at the end of each iteration, and is
 * provided access to the following quantities:
 *
 * i: index of current iteration
 * t: total elapsed computation time at the *start* of the current iteration
 * x: iterate at the *start* of the current iteration
 * f: objective value at x
 * g: Riemannian gradient at x
 * h: the update step computed during this iteration
 * df: decrease in objective value obtained by applying the update step
 */
template <typename Variable, typename Tangent, typename Scalar = double,
          typename... Args>
using GradientDescentUserFunction =
    std::function<void(size_t i, double t, const Variable &x, Scalar f,
                       const Tangent &g, const Tangent &h, Scalar df,
                       Args &...args)>;

/** A lightweight struct containing a few additional algorithm-specific
 * configuration parameters for a gradient descent optimization method
 * (cf. Section 4.2 of "Optimization Algorithms on Matrix Manifolds") */
template <typename Scalar = double>
struct GradientDescentParams : public SmoothOptimizerParams<Scalar> {

  // Initial trial stepsize for the Armijo linesearch (must be > 0)
  Scalar alpha = 1.0;

  // Shrinkage factor for the backtracking linesearch (must be in (0,1)).
  Scalar beta = .5;

  // Sufficient fractional decrease for step acceptance (must be in (0,1)).
  Scalar sigma = .5;

  // Maximum number of linesearch iterations to attempt
  size_t max_ls_iterations = 100;
};

/** A set of status flags indicating the stopping criterion that triggered
 * algorithm termination */
enum class GradientDescentStatus {
  /** The algorithm obtained a solution satisfying the gradient tolerance */
  Gradient,

  /** The algorithm terminated because the relative decrease in function value
   * obtained after the last accepted update was less than the specified
   * tolerance */
  RelativeDecrease,

  /** The algorithm terminated because the norm of the last accepted update step
   * was less than the specified tolerance */
  Stepsize,

  /** The algorithm terminated because linesearch failed to determine a stepsize
   * along the gradient direction that generated a sufficient decrease in
   * function value */
  LineSearch,

  /** The algorithm exhausted the allotted number of major (outer) iterations */
  IterationLimit,

  /** The algorithm exhausted the allotted computation time */
  ElapsedTime,
};

/** A useful struct used to hold the output of a gradient descent optimization
 * method */
template <typename Variable, typename Scalar = double>
struct GradientDescentResult : public SmoothOptimizerResult<Variable, Scalar> {

  GradientDescentStatus status;

  // Number of linesearch iterations performed during each iteration of the
  // algorithm
  std::vector<size_t> linesearch_iterations;
};

/** This function implements gradient descent on a general Riemannian manifold
 * (cf. Algorithm 1 in Sec. 4.2 of "Optimization Algorithms on Matrix
 * Manifolds", by P.-A. Absil, R. Mahoney, and R. Sepulchre".
 *
 * Here:
 *
 * - f is the objective function to be minimized.
 *
 * - gradF is a function that accepts as input a point X in M, and returns
 *   grad f(X), the Riemannian gradient of f at X (cf. Sec. 3.6 of "Optimization
 *   Algorithms on Matrix Manifolds").
 *
 * - metric is the Riemannian metric for M (a smooth assignment of an inner
 *   product to each of M's tangent spaces, cf. Sec. 3.6 of "Optimization
 *   Methods on Matrix Manifolds").
 *
 * - retract is a retraction operator: this is a function taking as input a
 *   point X in M and a tangent vector V in the tangent space T_X(M) of M at X,
 *   and returns a new point Y in M that is (intuitively) gotten by 'moving
 *   along V from X' (cf. Sec. 4.1 of "Optimization Methods on Riemannian
 *   Manifolds" for a precise definition).
 *
 * - x0 (in M) is the initialization point for the Riemannian gradient descent
 *   algorithm.
 */
template <typename Variable, typename Tangent, typename Scalar = double,
          typename... Args>
GradientDescentResult<Variable, Scalar> GradientDescent(
    const Objective<Variable, Scalar, Args...> &f,
    const VectorField<Variable, Tangent, Args...> &grad_f,
    const RiemannianMetric<Variable, Tangent, Scalar, Args...> &metric,
    const Retraction<Variable, Tangent, Args...> &retract, const Variable &x0,
    Args &...args,
    const GradientDescentParams<Scalar> &params =
        GradientDescentParams<Scalar>(),
    const std::optional<GradientDescentUserFunction<
        Variable, Tangent, Scalar, Args...>> &user_function = std::nullopt) {

  /// Argument checking

  // Termination criteria

  if (params.max_computation_time < 0)
    throw std::invalid_argument(
        "Maximum computation time must be a nonnegative real value");

  if (params.gradient_tolerance < 0)
    throw std::invalid_argument(
        "Gradient tolerance must be a nonnegative real value");

  if (params.alpha <= 0)
    throw std::invalid_argument("Initial stepsize for backtracking line-search "
                                "must be a positive real value");

  if (params.beta <= 0 || params.beta >= 1)
    throw std::invalid_argument(
        "Multiplicative shrinkage factor for stepsize in backtracking "
        "line-search must be a value in the range (0, 1)");

  if (params.sigma <= 0 || params.sigma >= 1)
    throw std::invalid_argument(
        "Sufficient fractional decrease parameter for step acceptance in "
        "backtracking line search must be a value in the range (0, 1)");

  /// Declare and initialize some useful variables

  // Square root of machine precision for Scalars
  Scalar sqrt_eps = sqrt(std::numeric_limits<Scalar>::epsilon());

  // Current and proposed next iterates
  Variable x, x_proposed;

  // Gradient at the current iterate
  Tangent grad_f_x;

  // Norm of the Riemannian gradient at the current iterate
  Scalar grad_f_x_norm;

  // Proposed update vector
  Tangent h;

  // Norm of update step
  Scalar h_norm;

  // Number of linesearches performed during the current iteration
  size_t ls_iters = 0;

  // Function values at the current iterate proposed iterates, respectively
  Scalar f_x, f_x_proposed;

  // Decrease in function value between subsequent iterations
  Scalar df;

  // Relative decrease in function value between the current and proposed
  // iterates
  Scalar relative_decrease;

  // Some useful constants for display purposes

  // Field with for displaying outer iterations
  size_t iter_field_width = floor(log10(params.max_iterations)) + 1;

  // Field width for displaying linesearch iterations
  size_t linesearch_iter_field_width =
      floor(log10(params.max_ls_iterations)) + 1;

  // Output struct
  GradientDescentResult<Variable, Scalar> result;
  result.status = GradientDescentStatus::IterationLimit;

  /// INITIALIZATION

  // Set initial iterate and function value
  x = x0;
  f_x = f(x, args...);

  // Compute gradient
  grad_f_x = grad_f(x, args...);
  grad_f_x_norm = sqrt(metric(x, grad_f_x, grad_f_x, args...));

  if (params.verbose) {
    // Set display options for real-valued quantities
    std::cout << std::scientific;
    std::cout.precision(params.precision);
  }
  if (params.verbose) {
    std::cout << "Gradient descent optimization: " << std::endl << std::endl;
  }

  // Start clock
  auto start_time = Stopwatch::tick();

  for (size_t iteration = 0; iteration < params.max_iterations; iteration++) {
    double elapsed_time = Stopwatch::tock(start_time);

    if (elapsed_time > params.max_computation_time) {
      result.status = GradientDescentStatus::ElapsedTime;
      break;
    }

    // Record output
    result.time.push_back(elapsed_time);
    result.objective_values.push_back(f_x);
    result.gradient_norms.push_back(grad_f_x_norm);

    if (params.log_iterates)
      result.iterates.push_back(x);

    if (params.verbose) {
      std::cout << "Iter: ";
      std::cout.width(iter_field_width);
      std::cout << iteration << ", time: " << elapsed_time << ", f: ";
      std::cout.width(params.precision + 7);
      std::cout << f_x << ", |g|: " << grad_f_x_norm;
    }

    // Test gradient-based stopping criterion
    if (grad_f_x_norm < params.gradient_tolerance) {
      result.status = GradientDescentStatus::Gradient;
      break;
    }

    /// Armijo linesearch

    // Armijo stepsize
    // Set t_A here so that t_A = alpha when executing the first iteration of
    // the following do-while
    Scalar t_A = params.alpha / params.beta;
    ls_iters = 0;

    bool accept = false;
    do {
      ls_iters++;
      // Shrink stepsize
      t_A *= params.beta;

      // Compute updated search step and proposed iterate
      h = -t_A * grad_f_x;
      x_proposed = retract(x, h, args...);

      // Compute function value at the proposed iterate and its improvement over
      // current iterate
      f_x_proposed = f(x_proposed, args...);
      df = f_x - f_x_proposed;

      accept = (df > params.sigma * t_A * grad_f_x_norm * grad_f_x_norm);

    } while ((!accept) && (ls_iters < params.max_ls_iterations));

    if (params.verbose) {
      std::cout << ", ls iters: ";
      std::cout.width(linesearch_iter_field_width);
      std::cout << ls_iters;
    }

    // Was linesearch successful?
    if (!accept) {
      result.status = GradientDescentStatus::LineSearch;
      break;
    }

    /// Iterate accepted!

    // Compute norm of update step
    h_norm = t_A * grad_f_x_norm;

    relative_decrease = df / (fabs(f_x) + sqrt_eps);

    // Record output
    result.linesearch_iterations.push_back(ls_iters);
    result.update_step_norms.push_back(h_norm);

    // Call the user-supplied function to provide access to internal algorithm
    // state
    if (user_function)
      (*user_function)(iteration, elapsed_time, x, f_x, grad_f_x, h, df,
                       args...);

    // Display output, if requested
    if (params.verbose) {
      std::cout << ", |h|: " << h_norm << ", df: " << df;
    }

    /// Update cached variables
    x = x_proposed;
    f_x = f_x_proposed;

    // Update gradient
    grad_f_x = grad_f(x, args...);
    grad_f_x_norm = sqrt(metric(x, grad_f_x, grad_f_x, args...));

    // Test additional stopping criteria
    if (relative_decrease < params.relative_decrease_tolerance) {
      result.status = GradientDescentStatus::RelativeDecrease;
      break;
    }

    if (h_norm < params.stepsize_tolerance) {
      result.status = GradientDescentStatus::Stepsize;
      break;
    }

    if (params.verbose)
      std::cout << std::endl;

  } // gradient descent iteration

  result.elapsed_time = Stopwatch::tock(start_time);

  // Record output
  result.x = x;
  result.f = f_x;
  result.gradfx_norm = grad_f_x_norm;

  if (params.verbose) {
    std::cout << std::endl
              << std::endl
              << "Optimization finished!" << std::endl;

    // Print the reason for termination
    switch (result.status) {
    case GradientDescentStatus::Gradient:
      std::cout << "Found first-order critical point! (Gradient norm: "
                << grad_f_x_norm << ")" << std::endl;
      break;
    case GradientDescentStatus::RelativeDecrease:
      std::cout
          << "Algorithm terminated due to insufficient relative decrease: "
          << relative_decrease << " < " << params.relative_decrease_tolerance
          << std::endl;
      break;
    case GradientDescentStatus::Stepsize:
      std::cout
          << "Algorithm terminated due to excessively small step size: |h| = "
          << h_norm << " < " << params.stepsize_tolerance << std::endl;
      break;
    case GradientDescentStatus::LineSearch:
      std::cout << "Algorithm terminated due to linesearch's inability to find "
                   "a stepsize with sufficient decrease"
                << std::endl;
      break;
    case GradientDescentStatus::IterationLimit:
      std::cout << "Algorithm exceeded maximum number of outer iterations"
                << std::endl;
      break;
    case GradientDescentStatus::ElapsedTime:
      std::cout << "Algorithm exceeded maximum allowed computation time: "
                << result.elapsed_time << " > " << params.max_computation_time
                << std::endl;
      break;
    }

    std::cout << "Final objective value: " << result.f << std::endl;
    std::cout << "Total elapsed computation time: " << result.elapsed_time
              << " seconds" << std::endl
              << std::endl;
  }

  return result;
}

/** The next function provides a convenient specialization/simplification of
 * the Riemannian gradient descent interface for the (common) use case of
 * optimization over Euclidean spaces.  This function makes the following
 * assumptions:
 *
 * - The metric is the standard Euclidean metric:  g(X, V1, V2) := <V1, V2> for
 *   all X in M
 *
 * - The retraction is the standard Euclidean retraction:  R_X(V) := X + V for
 *   all X in M.
 *
 * - We exploit the global parallelism and self-duality of Euclidean spaces to
 *   represent both *points* X in Euclidean space and *tangent vectors* V using
 *   the *same* data type (Vector).
 */

template <typename Vector, typename Scalar = double, typename... Args>
using EuclideanGradientDescentUserFunction =
    GradientDescentUserFunction<Vector, Vector, Scalar, Args...>;

template <typename Vector, typename Scalar = double, typename... Args>
GradientDescentResult<Vector, Scalar> EuclideanGradientDescent(
    const Objective<Vector, Scalar, Args...> &f,
    const EuclideanVectorField<Vector, Args...> grad_f, const Vector &x0,
    Args &...args,
    const GradientDescentParams<Scalar> &params =
        GradientDescentParams<Scalar>(),
    const std::optional<EuclideanGradientDescentUserFunction<
        Vector, Scalar, Args...>> &user_function = std::nullopt) {

  /// Run gradient descent using these Euclidean operators
  return GradientDescent<Vector, Vector, Scalar, Args...>(
      f, grad_f, EuclideanMetric<Vector, Scalar, Args...>,
      EuclideanRetraction<Vector, Args...>, x0, args..., params, user_function);
}
} // namespace Riemannian
} // namespace Optimization
