/** This header file provides a lightweight template function implementing
 * a truncated-Newton trust-region algorithm for large-scale superlinear
 * optimization on Riemannian manifolds.
 *
 * For more information on trust-region algorithms, please see the excellent
 * references:
 *
 * "Trust-Region Methods" by Conn, Gould, and Toint
 *
 * "Trust-Region Methods on Riemannian Manifolds" by Absil, Baker and Gallivan
 *
 * "Optimization Algorithms on Matrix Manifolds" by Absil, Mahoney, and
 *  Sepulchre
 *
 * Copyright (C) 2017 - 2022 by David M. Rosen (dmrosen@mit.edu)
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>

#include "Optimization/LinearAlgebra/IterativeSolvers.h"
#include "Optimization/Riemannian/Concepts.h"
#include "Optimization/Util/Stopwatch.h"

namespace Optimization {

namespace Riemannian {

/** An alias template for a user-definable function that can be
 * used to access various interesting bits of information about the internal
 * state of a truncated-Newton optimization algorithm as it runs.  Here:
 *
 * - i: index of current iteration
 * - t: total elapsed computation time at the *start* of the current iteration
 * - x: iterate at the *start* of the current iteration
 * - f: objective value at x
 * - g: Riemannian gradient at x
 * - HessOp:  Hessian operator at x
 * - Delta: trust-region radius at the *start* of the current iteration
 * - num_STPCG_iters: number of iterations of the Steihaug-Toint preconditioned
 *   conjugate-gradient method performed when computing the trust-update step h
 * - h:  the trust-region update step computed during this iteration
 * - df: decrease in objective value obtained by applying the trust-region
 *   update
 * - rho: value of the gain ratio for the proposed update step h
 * - accepted:  Boolean value indicating whether the proposed trust-region
 *   update step h was accepted
 *
 * This function is called at the end of each outer (major) iteration, after all
 * of the above-referenced quantities have been computed, but *before* the
 * update step h computed during this iteration is applied.
 *
 * This function may also return the Boolean value 'true' in order to terminate
 * the truncated-Newton trust-region algorithm; this provides a convenient means
 * of implementing a custom (user-definable) stopping criterion.
 *
 */
template <typename Variable, typename Tangent, typename Scalar = double,
          typename... Args>
using TNTUserFunction =
    std::function<bool(size_t i, double t, const Variable &x, Scalar f,
                       const Tangent &g,
                       const LinearOperator<Variable, Tangent, Args...> &HessOp,
                       Scalar Delta, size_t num_STPCG_iters, const Tangent &h,
                       Scalar df, Scalar rho, bool accepted, Args &...args)>;

/** A lightweight struct containing a few additional algorithm-specific
 * configuration parameters for a truncated-Newton trust-region method
 * (cf. Algorithm 6.1.1 of "Trust-Region Methods") */
template <typename Scalar = double>
struct TNTParams : public SmoothOptimizerParams<Scalar> {
  /// Trust-region control parameters

  /** Initial size of trust-region radius */
  Scalar Delta0 = 1;

  /** Lower-bound on the gain ratio for accepting a proposed step (should
   * satisfy 0 < eta1 <= eta2) */
  Scalar eta1 = .05;

  /** Lower-bound on the gain ratio for a 'very successful' iteration (should
   * satisfy eta1 <= eta2 < 1). */
  Scalar eta2 = .9;

  /** Multiplicative factor for decreasing the trust-region radius on an
   * unsuccessful iteration; this parameter should satisfy 0 < alpha1 < 1. */
  Scalar alpha1 = .25;

  /** Multiplicative factor for increasing the trust-region radius on a very
   * successful iteration.  This parameter should satisfy alpha2 > 1. */
  Scalar alpha2 = 2.5;

  /// Truncated preconditioned conjugate-gradient control parameters
  // See Section 7.5.1 of "Trust-Region Methods" for details

  /** Maximum number of conjugate-gradient iterations to apply to solve each
   * instance of the trust-region subproblem */
  size_t max_TPCG_iterations = 1000;

  /** Stopping tolerance for the norm of the predicted gradient (acccording to
   * the local quadratic model): terminate the inner iterations if ||g_pred||
   * kappa_fgr * || g_init ||.  This value should lie in the range (0, 1) */
  Scalar kappa_fgr = .1;

  /** Stopping tolerance for the norm of the predicted gradient (according to
   * the local quadratic model):  terminate the inner iterations if ||g_pred|| <
   * ||g_init||^{1+theta}.  This value should be positive, and controls the
   * asymptotic convergence rate of the TNT algorithm: specifically, for theta >
   * 0, TNT convergences q-superlinearly with order (1 + theta) (cf. Theorem 2.3
   * of "Truncated-Newton Algorithms for Large-Scale Unconstrained
   * Optimization", by R.S. Dembo and T. Steihaug.) */
  Scalar theta = .5;

  /// Additional termination criteria

  /** Stopping tolerance for the norm of the preconditioned gradient
   * || M^{-1} g ||.  Note that this stopping criterion has the advantage of
   * (approximate) scale invariance */
  Scalar preconditioned_gradient_tolerance = 1e-6;

  /** Stopping condition based on the size of the trust-region radius; terminate
   * if the radius is reduced below this quantity */
  Scalar Delta_tolerance = 1e-6;
};

/** A set of status flags indicating the stopping criterion that triggered
 * algorithm termination */
enum class TNTStatus {

  /** The algorithm obtained a solution satisfying the gradient tolerance */
  Gradient,

  /** The algorithm obtained a solution satisfying the preconditioned gradient
   * tolerance */
  PreconditionedGradient,

  /** The algorithm terminated because the relative decrease in function value
   * obtained after the last accepted update was less than the specified
   * tolerance */
  RelativeDecrease,

  /** The algorithm terminated because the norm of the last accepted update
   * step was less than the specified tolerance */
  Stepsize,

  /** The algorithm terminated because the trust-region radius decreased below
   * the specified threshold */
  TrustRegion,

  /** The algorithm exhausted the allotted number of major (outer) iterations */
  IterationLimit,

  /** The algorithm exhausted the allotted computation time */
  ElapsedTime,

  /** The algorithm terminated due to the user-supplied stopping criterion */
  UserFunction
};

/** A useful struct used to hold the output of a truncated-Newton trust-region
 * optimization method */
template <typename Variable, typename Scalar = double>
struct TNTResult : public SmoothOptimizerResult<Variable, Scalar> {

  /** The norm of the preconditioned gradient at the returned estimate */
  Scalar preconditioned_grad_f_x_norm;

  /** The stopping condition that triggered algorithm termination */
  TNTStatus status;

  /** The sequence of norms of the preconditioned gradients generated by the
   * algorithm */
  std::vector<Scalar> preconditioned_gradient_norms;

  /** The number of (inner) iterations performed by the Steihaug-Toint
   * preconditioned conjugate-gradient method during each major (outer)
   * iteration */
  std::vector<size_t> inner_iterations;

  /** The M-norm of the update step computed during each iteration */
  std::vector<Scalar> update_step_M_norms;

  /** The gain ratios of the update steps computed during each iteration */
  std::vector<Scalar> gain_ratios;

  /** The trust-region radius at the START of each iteration */
  std::vector<Scalar> trust_region_radius;
};

/** This function implements a Riemannian truncated-Newton trust-region method,
 * using the Steihaug-Toint preconditioned truncated conjugate-gradient method
 * as the inner (approximate) trust-region subproblem solver. This
 * implementation follows the one given as Algorithm 6.1.1 of "Trust-Region
 * Methods", generalized to the setting of a generic Riemannian manifold (see
 * also the paper "Trust-Region Methods on Riemannian Manifolds" by P.-A. Absil,
 * C.G. Baker, and K.A. Gallivan, and/or "Optimization Algorithms on Matrix
 * Manifolds" by P.-A. Absil, R. Mahoney, and R. Sepulchre).
 *
 * Here:
 *
 * - f is the objective function to be minimized.
 *
 * - QM is a function that constructs a quadratic model for the objective f on
 *   the tangent space of the manifold M at X. Concretely, this is a
 *   std::function having the signature:
 *
 *   void QM(const Variable &X, Tangent &gradient, LinearOperator<Variable,
 *        Tangent, Args...> &Hessian, Args &... args)>;
 *
 *   this function sets the argument 'gradient' to the Riemannian gradient
 *   grad f(X) of f at X, and the argument 'Hessian' to the Riemanninan Hessian
 *   Hess f(X) of f at X (recall that this is a linear operator
 *   Hess f(X) : T_X(M) -> T_X(M) on the tangent space T_X(M) of M at X that
 *   maps each tangent vector V to the covariant derivative of the gradient
 *   vector field grad f(X) along V at X; cf. Secs. 3.6 and 5.5 of
 *   "Optimization Methods on Matrix Manifolds").
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
 * - precon is an optional preconditioning operator for preconditioning the
 *   conjugate-gradient method used to compute the inexact Newton update; this
 *   should be chosen so that precon * HessF has a more favorable spectrum than
 *   HessF alone.
 *
 * - x0 (in M) is the initialization point for the Riemannian truncated-Newton
 *   trust-region algorithm.
 */
template <typename Variable, typename Tangent, typename Scalar = double,
          typename... Args>
TNTResult<Variable, Scalar>
TNT(const Objective<Variable, Scalar, Args...> &f,
    const QuadraticModel<Variable, Tangent, Args...> &QM,
    const RiemannianMetric<Variable, Tangent, Scalar, Args...> &metric,
    const Retraction<Variable, Tangent, Args...> &retract, const Variable &x0,
    Args &...args,
    const std::optional<LinearOperator<Variable, Tangent, Args...>> &precon =
        std::nullopt,
    const TNTParams<Scalar> &params = TNTParams<Scalar>(),
    const std::optional<TNTUserFunction<Variable, Tangent, Scalar, Args...>>
        &user_function = std::nullopt) {

  /// Argument checking

  // Termination criteria

  if (params.max_computation_time < 0)
    throw std::invalid_argument(
        "Maximum computation time must be a nonnegative real value");

  if (params.gradient_tolerance < 0)
    throw std::invalid_argument(
        "Gradient tolerance must be a nonnegative real value");

  if (params.preconditioned_gradient_tolerance < 0)
    throw std::invalid_argument(
        "Preconditioned gradient tolerance must be a nonnegative real value");

  if (params.relative_decrease_tolerance < 0)
    throw std::invalid_argument(
        "Relative decrease tolerance must be a nonnegative real value");

  if (params.stepsize_tolerance < 0)
    throw std::invalid_argument(
        "Stepsize tolerance must be a nonnegative real value");

  if (params.Delta_tolerance < 0)
    throw std::invalid_argument(
        "Trust-region radius tolerance must be a nonnegative real value");

  // Trust-region control parameters

  if (params.Delta0 <= 0)
    throw std::invalid_argument(
        "Initial trust-region radius must be a positive real value");

  if (params.eta1 <= 0 || params.eta1 >= 1)
    throw std::invalid_argument(
        "Threshold on gain ratio for a successful iteration (eta1) must "
        "satisfy 0 < eta1 < 1");

  if (params.eta1 > params.eta2 || params.eta2 >= 1)
    throw std::invalid_argument(
        "Threshold on gain ratio for a very successful iteration (eta2) must "
        "satisfy eta1 <= eta2 < 1");

  if (params.alpha1 <= 0 || params.alpha1 >= 1)
    throw std::invalid_argument(
        "Multiplicative factor for decreasing trust-region radius (alpha1) "
        "must satisfy 0 < alpha1 < 1");

  if (params.alpha2 <= 1)
    throw std::invalid_argument(
        "Multiplicative factor for increasing trust-region radius (alpha1) "
        "must satisfy alpha2 > 1");

  // Linear-algebraic control parameters
  if (params.kappa_fgr <= 0 || params.kappa_fgr >= 1)
    throw std::invalid_argument("Target relative decrease in predicted "
                                "residual for inexact update step computation "
                                "(kappa_fgr) must satisfy 0 < kappa_fgr < 1");

  if (params.theta < 0)
    throw std::invalid_argument("Target superlinear convergence rate parameter "
                                "(theta) must be a nonnegative real number");

  /// Declare and initialize some useful variables

  // Square root of machine precision for type Scalar
  Scalar sqrt_eps = sqrt(std::numeric_limits<Scalar>::epsilon());

  // Output struct
  TNTResult<Variable, Scalar> result;
  result.status =
      TNTStatus::IterationLimit; // "Default" stopping condition (i.e. will
                                 // trigger if no other is)

  // Current iterate and proposed next iterates;
  Variable x, x_proposed;

  // Function value at the current iterate and proposed iterate
  Scalar fx, fx_proposed;

  // Gradient and preconditioned gradient at the current iterate
  Scalar gradfx_norm, preconditioned_gradfx_norm;

  // Trust-region radius
  Scalar Delta;

  // Update step norm in the Euclidean and M-norms
  Scalar h_norm, h_M_norm;

  // Relative decrease between subsequent accepted function iterations
  Scalar relative_decrease;

  // Gradient and Hessian operator at the current iterate
  Tangent grad;
  LinearOperator<Variable, Tangent, Args...> Hess;

  // Some useful constants for display purposes

  // Field with for displaying outer iterations
  size_t iter_field_width = floor(log10(params.max_iterations)) + 1;

  // Field width for displaying inner iterations
  size_t inner_iter_field_width = floor(log10(params.max_TPCG_iterations)) + 1;

  // Preallocate space for output vectors

  result.time.reserve(params.max_iterations + 1);
  result.objective_values.reserve(params.max_iterations + 1);
  result.gradient_norms.reserve(params.max_iterations + 1);
  result.preconditioned_gradient_norms.reserve(params.max_iterations + 1);
  result.trust_region_radius.reserve(params.max_iterations + 1);

  if (params.log_iterates)
    result.iterates.reserve(params.max_iterations + 1);

  /// INITIALIZATION

  // Set initial iterate ...
  x = x0;
  // Function value ...
  fx = f(x, args...);

  // And local quadratic model
  QM(x, grad, Hess, args...);

  gradfx_norm = sqrt(metric(x, grad, grad, args...));
  if (precon) {
    // Precondition gradient
    Tangent preconditioned_gradient = (*precon)(x, grad, args...);
    preconditioned_gradfx_norm = sqrt(
        metric(x, preconditioned_gradient, preconditioned_gradient, args...));
  } else {
    // We use the identity preconditioner, so the norms of the gradient and
    // preconditioned gradient are identical
    preconditioned_gradfx_norm = gradfx_norm;
  }

  /// Set up function handles for inner STPCG linear system solver

  // We perform *unconstrained* optimization, so we have no Lagrange multipliers
  typedef std::nullptr_t MultiplierType;

  // Linear operator
  Optimization::LinearAlgebra::SymmetricLinearOperator<Tangent, Args...> H =
      [&x, &Hess](const Tangent &v, Args &...args) -> Tangent {
    return Hess(x, v, args...);
  };

  // Inner product
  Optimization::LinearAlgebra::InnerProduct<Tangent, Scalar, Args...>
      inner_product = [&x, &metric](const Tangent &v1, const Tangent &v2,
                                    Args &...args) -> Scalar {
    return metric(x, v1, v2, args...);
  };

  // Optional preconditioner, if the user supplied one
  Optimization::LinearAlgebra::STPCGPreconditioner<Tangent, MultiplierType,
                                                   Args...>
      P = [&x, &precon](const Tangent &v,
                        Args &...args) -> std::pair<Tangent, MultiplierType> {
    return std::pair<Tangent, MultiplierType>((*precon)(x, v, args...),
                                              MultiplierType());
  };

  std::optional<Optimization::LinearAlgebra::STPCGPreconditioner<
      Tangent, MultiplierType, Args...>>
      Pop(precon
              ? P
              : std::optional<Optimization::LinearAlgebra::STPCGPreconditioner<
                    Tangent, MultiplierType, Args...>>(std::nullopt));

  // Initialize trust-region radius
  Delta = params.Delta0;

  if (params.verbose) {
    // Set display options for real-valued quantities
    std::cout << std::scientific;
    std::cout.precision(params.precision);
    std::cout << "Truncated-Newton trust-region optimization: " << std::endl
              << std::endl;
  }

  // Start clock
  auto start_time = Stopwatch::tick();

  /// ITERATIONS

  // Basic trust-region iteration -- see Algorithm 6.1.1 of "Trust-Region
  // Methods"
  for (size_t iteration = 0; iteration < params.max_iterations; ++iteration) {
    double elapsed_time = Stopwatch::tock(start_time);

    if (elapsed_time > params.max_computation_time) {
      result.status = TNTStatus::ElapsedTime;
      break;
    }

    // Record output
    result.time.push_back(elapsed_time);
    result.objective_values.push_back(fx);
    result.gradient_norms.push_back(gradfx_norm);
    result.preconditioned_gradient_norms.push_back(preconditioned_gradfx_norm);
    result.trust_region_radius.push_back(Delta);

    if (params.log_iterates)
      result.iterates.push_back(x);

    if (params.verbose) {
      std::cout << "Iter: ";
      std::cout.width(iter_field_width);
      std::cout << iteration << ", time: " << elapsed_time << ", f: ";
      std::cout.width(params.precision + 7);
      std::cout << fx << ", |g|: " << gradfx_norm
                << ", |M^{-1}g|: " << preconditioned_gradfx_norm;
    }

    // Test gradient-based stopping criterion
    if (gradfx_norm < params.gradient_tolerance) {
      result.status = TNTStatus::Gradient;
      break;
    }
    if (preconditioned_gradfx_norm < params.preconditioned_gradient_tolerance) {
      result.status = TNTStatus::PreconditionedGradient;
      break;
    }

    /// STEP 2:  Solve the local trust-region subproblem at the current
    /// iterate using the computed quadratic model using the Steihaug-Toint
    /// truncated preconditioned conjugate-gradient method

    // Norm of the update in the norm determined by the preconditioner
    size_t inner_iterations;
    Tangent h = Optimization::LinearAlgebra::STPCG<Tangent, MultiplierType,
                                                   Scalar, Args...>(
        grad, H, inner_product, args..., h_M_norm, inner_iterations, Delta,
        params.max_TPCG_iterations, params.kappa_fgr, params.theta, Pop);
    h_norm = sqrt(metric(x, h, h, args...));

    if (params.verbose) {
      std::cout << ", Delta: " << Delta << ", inner iters: ";
      std::cout.width(inner_iter_field_width);
      std::cout << inner_iterations << ", |h|: " << h_norm
                << ", |h|_M: " << h_M_norm;
    }

    /// STEP 3:  Update and evaluate trial point

    // Get trial point
    x_proposed = retract(x, h, args...);

    // Evalute objective at trial point
    fx_proposed = f(x_proposed, args...);

    // Predicted model decrease
    Scalar dm = -metric(x, grad, h, args...) -
                .5 * metric(x, h, Hess(x, h, args...), args...);

    // Actual function decrease
    Scalar df = fx - fx_proposed;

    // Relative function decrease
    relative_decrease = df / (sqrt_eps + fabs(fx));

    // Evaluate gain ratio
    Scalar rho = df / dm;

    if (params.verbose) {
      std::cout << ", df: ";
      std::cout.width(params.precision + 7);
      std::cout << df << ", rho: ";
      std::cout.width(params.precision + 7);
      std::cout << rho << ". ";
    }

    /// Determine acceptance of trial point
    bool step_accepted = (!std::isnan(rho) && rho > params.eta1);

    if (params.verbose)
      std::cout << (step_accepted ? "Step accepted" : "Step REJECTED!");

    // Record output
    result.inner_iterations.push_back(inner_iterations);
    result.update_step_norms.push_back(h_norm);
    result.update_step_M_norms.push_back(h_M_norm);
    result.gain_ratios.push_back(rho);

    // Call the user-supplied function to provide access to internal algorithm
    // state, and check for user-requested termination
    if (user_function) {
      if ((*user_function)(iteration, elapsed_time, x, fx, grad, Hess, Delta,
                           inner_iterations, h, df, rho, step_accepted,
                           args...)) {
        result.status = TNTStatus::UserFunction;
        break;
      }
    }

    /// Update cached values if the iterate is accepted
    if (step_accepted) {
      // Accept iterate and cache values ...
      x = std::move(x_proposed);
      fx = fx_proposed;

      // Test relative decrease-based stopping criterion
      if (relative_decrease < params.relative_decrease_tolerance) {
        result.status = TNTStatus::RelativeDecrease;
        break;
      }

      // Test stepsize-based stopping criterion ...
      if (h_norm < params.stepsize_tolerance) {
        result.status = TNTStatus::Stepsize;
        break;
      }

      // ... and recompute the local quadratic model at the current iterate
      QM(x, grad, Hess, args...);

      gradfx_norm = sqrt(metric(x, grad, grad, args...));
      if (precon) {
        // Precondition gradient
        Tangent preconditioned_gradient = (*precon)(x, grad, args...);
        preconditioned_gradfx_norm = sqrt(metric(
            x, preconditioned_gradient, preconditioned_gradient, args...));
      } else {
        // We use the identity preconditioner, so the norms of the gradient
        // and preconditioned gradient are identical
        preconditioned_gradfx_norm = gradfx_norm;
      }

    } // if (step_accepted)

    /// STEP 4: Update trust-region radius
    if ((!std::isnan(rho)) && (rho >= params.eta2)) {
      // This iteration was very successful, so we want to increase the
      // trust-region radius
      Delta = std::max<Scalar>(params.alpha2 * h_M_norm, Delta);
    } else if (std::isnan(rho) || (rho < params.eta1)) {
      // This was not a successful iteration, so we should shrink the
      // trust-region radius
      Delta = params.alpha1 * h_M_norm;

      if (Delta < params.Delta_tolerance) {
        result.status = TNTStatus::TrustRegion;
        break;
      }
    } // trust-region update

    if (params.verbose)
      std::cout << std::endl;
  } // end trust-region iterations
  result.elapsed_time = Stopwatch::tock(start_time);

  // Record output
  result.x = x;
  result.f = fx;
  result.gradfx_norm = gradfx_norm;
  result.preconditioned_grad_f_x_norm = preconditioned_gradfx_norm;

  // Also add these to the sequence of recorded state traces
  result.time.push_back(result.elapsed_time);
  result.objective_values.push_back(fx);
  result.gradient_norms.push_back(gradfx_norm);
  result.preconditioned_gradient_norms.push_back(preconditioned_gradfx_norm);
  result.trust_region_radius.push_back(Delta);

  if (params.log_iterates)
    result.iterates.push_back(x);

  if (params.verbose) {
    std::cout << std::endl
              << std::endl
              << "Optimization finished!" << std::endl;

    // Print the reason for termination
    switch (result.status) {
    case TNTStatus::Gradient:
      std::cout << "Found first-order critical point! (Gradient norm: "
                << gradfx_norm << ")" << std::endl;
      break;
    case TNTStatus::PreconditionedGradient:
      std::cout << "Found first-order critical point! (Preconditioned "
                   "gradient norm: "
                << preconditioned_gradfx_norm << ")" << std::endl;
      break;
    case TNTStatus::RelativeDecrease:
      std::cout
          << "Algorithm terminated due to insufficient relative decrease: "
          << relative_decrease << " < " << params.relative_decrease_tolerance
          << std::endl;
      break;
    case TNTStatus::Stepsize:
      std::cout
          << "Algorithm terminated due to excessively small step size: |h| = "
          << h_norm << " < " << params.stepsize_tolerance << std::endl;
      break;
    case TNTStatus::TrustRegion:
      std::cout << "Algorithm terminated due to excessively small trust region "
                   "radius: "
                << Delta << " < " << params.Delta_tolerance << std::endl;
      break;
    case TNTStatus::IterationLimit:
      std::cout << "Algorithm exceeded maximum number of outer iterations"
                << std::endl;
      break;
    case TNTStatus::ElapsedTime:
      std::cout << "Algorithm exceeded maximum allowed computation time: ("
                << result.elapsed_time << " > " << params.max_computation_time
                << " seconds)" << std::endl;
      break;
    case TNTStatus::UserFunction:
      std::cout
          << "Algorithm terminated due to user-supplied stopping criterion"
          << std::endl;
      break;
    }

    std::cout << "Final objective value: " << result.f << std::endl;
    std::cout << "Norm of Riemannian gradient: " << result.gradfx_norm
              << std::endl;
    std::cout << "Norm of preconditioned Riemannian gradient: "
              << result.preconditioned_grad_f_x_norm << std::endl;
    std::cout << "Total elapsed computation time: " << result.elapsed_time
              << " seconds" << std::endl
              << std::endl;

    // Reset std::cout output stream to default display parameters
    std::cout << std::defaultfloat;
    std::cout.precision(6);
  }

  return result;
}

/** Syntactic sugar for the above Riemannian TNT function: this interface
 * enables the user to supply separate functions that return the Riemannian
 * gradient and Hessian operator at X, rather than a single QuadraticModel
 * function.  Here:
 *
 * - gradF is a function that accepts as input a point X in M, and returns
 *   grad f(X), the Riemannian gradient of f at X.
 *
 * - HC is a function that accepts as input a point X in M, and returns
 *   Hess f(X), the linear operator Hess f(X) : T_X(M) -> T_X(M) on the tangent
 *   space T_X(M) of M at X that assigns to each tangent vector V the covariant
 *   derivative of the gradient vector field grad f(X) along V.
 */
template <typename Variable, typename Tangent, typename Scalar = double,
          typename... Args>
TNTResult<Variable, Scalar>
TNT(const Objective<Variable, Scalar, Args...> &f,
    const VectorField<Variable, Tangent, Args...> &grad_f,
    const LinearOperatorConstructor<Variable, Tangent, Args...>
        &HessianConstructor,
    const RiemannianMetric<Variable, Tangent, Scalar, Args...> &metric,
    const Retraction<Variable, Tangent, Args...> &retract, const Variable &x0,
    Args &...args,
    const std::optional<LinearOperator<Variable, Tangent, Args...>> &precon =
        std::nullopt,
    const TNTParams<Scalar> &params = TNTParams<Scalar>(),
    const std::optional<TNTUserFunction<Variable, Tangent, Scalar, Args...>>
        &user_function = std::nullopt) {

  // Construct a QuadraticModel function from the passed VectorField and
  // HessianConstructor functions
  QuadraticModel<Variable, Tangent, Args...> QM =
      [&grad_f, &HessianConstructor](
          const Variable &X, Tangent &grad,
          LinearOperator<Variable, Tangent, Args...> &Hess, Args &...args) {
        // Set gradient
        grad = grad_f(X, args...);

        // Set Hessian operator
        Hess = HessianConstructor(X, args...);
      };

  // Now call TNT using this QuadraticModel function
  return TNT<Variable, Tangent, Scalar, Args...>(
      f, QM, metric, retract, x0, args..., precon, params, user_function);
}

/** These next functions provide a convenient specialization/simplification of
 * the Riemannian TNT interface for the (common) use case of optimization over
 * Euclidean spaces.  These functions make the following assumptions:
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
using EuclideanTNTUserFunction =
    TNTUserFunction<Vector, Vector, Scalar, Args...>;

template <typename Vector, typename Scalar = double, typename... Args>
TNTResult<Vector, Scalar> EuclideanTNT(
    const Objective<Vector, Scalar, Args...> &f,
    const EuclideanQuadraticModel<Vector, Args...> &QM, const Vector &x0,
    Args &...args,
    const std::optional<EuclideanLinearOperator<Vector, Args...>> &precon =
        std::nullopt,
    const TNTParams<Scalar> &params = TNTParams<Scalar>(),
    const std::optional<EuclideanTNTUserFunction<Vector, Scalar, Args...>>
        &user_function = std::nullopt) {

  /// Run TNT algorithm using these Euclidean operators
  return TNT<Vector, Vector, Scalar, Args...>(
      f, QM, EuclideanMetric<Vector, Scalar, Args...>,
      EuclideanRetraction<Vector, Args...>, x0, args..., precon, params,
      user_function);
}

/** Syntactic sugar: enables the user to supply separate functions that return
 * the Euclidean gradient and Hessian operator at X, rather than a single
 * QuadraticModel function */
template <typename Vector, typename Scalar = double, typename... Args>
TNTResult<Vector, Scalar> EuclideanTNT(
    const Objective<Vector, Scalar, Args...> &f,
    const EuclideanVectorField<Vector, Args...> &nabla_f,
    const EuclideanLinearOperatorConstructor<Vector, Args...>
        &HessianConstructor,
    const Vector &x0, Args &...args,
    const std::optional<EuclideanLinearOperator<Vector, Args...>> &precon =
        std::nullopt,
    const TNTParams<Scalar> &params = TNTParams<Scalar>(),
    const std::optional<EuclideanTNTUserFunction<Vector, Scalar, Args...>>
        &user_function = std::nullopt) {

  EuclideanQuadraticModel<Vector, Args...> QM =
      [&nabla_f, &HessianConstructor](
          const Vector &X, Vector &grad,
          EuclideanLinearOperator<Vector, Args...> &Hess, Args &...args) {
        // Set gradient
        grad = nabla_f(X, args...);

        // Set Hessian operator
        Hess = HessianConstructor(X, args...);
      };

  // Now call EuclideanTNT using this QuadraticModel function
  return EuclideanTNT<Vector, Scalar, Args...>(f, QM, x0, args..., precon,
                                               params, user_function);
}

} // namespace Riemannian
} // namespace Optimization
