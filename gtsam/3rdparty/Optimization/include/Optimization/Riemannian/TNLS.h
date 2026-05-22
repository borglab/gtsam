/** This header file provides a lightweight template function implementing
 * a Riemannian truncated-Newton trust-region algorithm for solving large-scale
 * nonlinear least-squares problems of the form:
 *
 * min_x L(x)
 *
 * where L(x) := | F(x) | and F: X -> Y is a vector-valued mapping from a
 * smooth Riemannian manifold X into a Euclidean space Y.  While it is possible
 * to solve such problems using generic Riemannian optimization algorithms (e.g.
 * the Riemannian truncated-Newton trust-region method implemented in TNT.h),
 * TNLS can take advantage of the specific form of the nonlinear least-squares
 * problem by employing specialized linear-algebraic methods to solve the
 * sequence of model trust-region subproblems:
 *
 * min_h | gra h + F(xk)|
 *
 * s.t.  |x| <= Delta
 *
 * in a more numerically robust and efficient manner.
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
 * Copyright (C) 2017 - 2021 by David M. Rosen (dmrosen@mit.edu)
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

/** An alias template for a pair (M, MT, Minv) : T_x(M) -> T_x(M)
 * of linear operators on the tangent space T_x(M) of the manifold M at x.
 * Specifically:
 *
 * - M(x) is an invertible right-preconditioner for the Jacobian gradFx of F at
 *   x; that is, the product gradFx * M(x) should have a more favorable spectrum
 *   than gradFx alone
 * - MT(x) is the transpose of M(x)
 */
template <typename VariableX, typename TangentX, typename... Args>
using TNLSPreconditioner =
    std::pair<LinearOperator<VariableX, TangentX, Args...>,
              LinearOperator<VariableX, TangentX, Args...>>;

/** An alias template for a user-definable function that can be
 * used to access various interesting bits of information about the internal
 * state of the algorithm as it runs.  Here:
 *
 * - i: index of current iteration
 * - t: total elapsed computation time at the *start* of the current
 *   iteration
 * - x: iterate at the *start* of the current iteration
 * - Fx:  Current value of the vector function F at the current iterate
 * - gradFx, gradFxT:  The Jacobian operator of F at the current iterate
 * - Delta: trust-region radius at the *start* of the current iteration
 * - num_LSQR_iters: number of iterations of LSQR method performed when
 *   computing the trust-update step h
 * - h:  the trust-region update step computed during this iteration
 * - dL: decrease in objective value L(x) := |F(x)| obtained by applying the
 *   trust-region update
 * - rho: value of the gain ratio for the proposed update step h
 * - accepted:  Boolean value indicating whether the proposed trust-region
 *   update step h was accepted
 *
 * This function is called at the end of each outer (major) iteration, after
 * all of the above-referenced quantities have been computed, but *before*
 * the update step h computed during this iteration is applied.
 *
 * This function may also return the Boolean value 'true' in order to
 * terminate the truncated-Newton trust-region algorithm; this provides a
 * convenient means of implementing a custom (user-definable) stopping
 * criterion.
 *
 */
template <typename VariableX, typename TangentX, typename VectorY,
          typename Scalar = double, typename... Args>
using TNLSUserFunction = std::function<bool(
    size_t i, double t, const VariableX &x, VectorY Fx,
    const Jacobian<VariableX, TangentX, VectorY, Args...> &gradFx,
    const JacobianAdjoint<VariableX, TangentX, VectorY, Args...> &gradFxT,
    Scalar Delta, size_t num_LSQR_iters, const TangentX &h, Scalar dL,
    Scalar rho, bool accepted, Args &...args)>;

/** A lightweight struct containing a few additional algorithm-specific
 * configuration parameters for a truncated-Newton trust-region method
 * (cf. Algorithm 6.1.1 of "Trust-Region Methods") */
template <typename Scalar = double>
struct TNLSParams : public SmoothOptimizerParams<Scalar> {
  /// Trust-region control parameters

  /** Initial size of trust-region radius */
  Scalar Delta0 = 1;

  /** Lower-bound on the gain ratio for accepting a proposed step (should
satisfy 0 < eta1 <= eta2) */
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

  /// LSQR control parameters

  /** Maximum number of LSQR iterations to apply to solve each
   * instance of the trust-region subproblem */
  size_t max_LSQR_iterations = 1000;

  /** Stopping tolerance for the norm of the predicted gradient (acccording to
   * the local quadratic model): terminate the inner iteration if the linear
   * least-squares residual r satisfies |r| <= kappa_fgr * | F(x) |.  This
   * value should lie in the range (0, 1) */
  Scalar kappa_fgr = .1;

  /** Stopping tolerance for the norm of the predicted gradient (according to
   * the local quadratic model):  terminate the inner iterations if if the
   * linear least-squares residual r satisfies |r| <=| F(x) |^{1+theta}.  This
   * value should be positive, and controls the asymptotic convergence rate of
   * the TNLS algorithm: specifically, for theta > 0, TNLS convergences
   * q-superlinearly with order (1 + theta) (cf. Theorem 2.3 of
   * "Truncated-Newton Algorithms for Large-Scale Unconstrained Optimization",
   * by R.S. Dembo and T. Steihaug.) */
  Scalar theta = .5;

  /** Tikhonov regularization parameter */
  Scalar lambda = 0;

  /** Stopping tolerances based upon the estimates for the norm of the
   * coefficient matrix its conditioning */
  Scalar Atol = 1e-6;
  Scalar Acond_limit = 1e8;

  /// Additional termination criteria

  /** Stopping tolerance based upon the norm |F(x)|; this is useful for
   * estimating solutions of nonlinear equations of the form F(x) = 0.  */
  Scalar root_tolerance = 1e-6;

  /** Stopping condition based on the size of the trust-region radius; terminate
   * if the radius is reduced below this quantity */
  Scalar Delta_tolerance = 1e-6;
};

/** A set of status flags indicating the stopping criterion that triggered
 * algorithm termination */
enum class TNLSStatus {

  /** The algorithm terminated because it found a point x with sufficiently
   * small norm |F(x)| to be considered a solution of F(x) = 0 */
  Root,

  /** The algorithm obtained a solution satisfying the gradient tolerance */
  Gradient,

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

  /** The algorithm exhausted the allotted number of major (outer) iterations
   */
  IterationLimit,

  /** The algorithm exhausted the allotted computation time */
  ElapsedTime,

  /** The algorithm terminated due to the user-supplied stopping criterion */
  UserFunction
};

/** A useful struct used to hold the output of a truncated-Newton trust-region
 * optimization method */
template <typename Variable, typename Scalar = double>
struct TNLSResult : public SmoothOptimizerResult<Variable, Scalar> {

  /** The stopping condition that triggered algorithm termination */
  TNLSStatus status;

  /** The number of (inner) iterations performed by the Steihaug-Toint
   * preconditioned conjugate-gradient method during each major (outer)
   * iteration */
  std::vector<size_t> inner_iterations;

  /** The gain ratio of the update step computed during each iteration */
  std::vector<Scalar> rho;

  /** The trust-region radius at the START of each iteration */
  std::vector<Scalar> trust_region_radius;
};

/** This function implements a Riemannian truncated-Newton trust-region
 * algorithm for solving large-scale nonlinear least-squares problems of the
 * form:
 *
 * min_x L(x)
 *
 * where L(x) := |F(x)| and F: X -> Y is a vector-valued mapping from a smooth
 * Riemannian manifold X into a Euclidean space Y, using LSQR as the iterative
 * linear-algebraic method for (approximately) solving the model trust-region
 * subproblems:
 *
 * min_h |gradF(x) * h + F(x)]^2
 *
 * s.t.  |h| <= Delta
 *
 * Here:
 *
 * - F: X -> Y is a vector-valued mapping from a Riemannian manifold X into a
 *   Euclidean space Y.
 *
 * - J is a function that accepts as input a point x in X, and returns a
 *   *pair* (gradF, gradFt) consisting of the Jacobian of F and its transpose
 *   operator, evaluated at x.
 *
 * - metric_X is the Riemannian metric for X (a smooth assignment of an inner
 *   product to each of M's tangent spaces, cf. Sec. 3.6 of "Optimization
 *   Methods on Matrix Manifolds").
 *
 * - inner_product_Y is the inner product on the (Euclidean) space Y
 *
 * - retract_X is a retraction operator for X: this is a function taking as
 *   input a point X in M and a tangent vector V in the tangent space T_X(M) of
 *   M at X, and returns a new point Y in M that is (intuitively) gotten by
 *   'moving along V from X' (cf. Sec. 4.1 of "Optimization Methods on
 *   Riemannian Manifolds" for a precise definition).
 *
 * - x0 (in X) is the initialization point for the Riemannian truncated-Newton
 *   trust-region algorithm.
 */
template <typename VariableX, typename TangentX, typename VectorY,
          typename Scalar = double, typename... Args>
TNLSResult<VariableX, Scalar>
TNLS(const Mapping<VariableX, VectorY, Args...> &F,
     const JacobianPairFunction<VariableX, TangentX, VectorY> &J,
     const RiemannianMetric<VariableX, TangentX, Scalar, Args...> &metric_X,
     const LinearAlgebra::InnerProduct<VectorY, Scalar, Args...>
         &inner_product_Y,
     const Retraction<VariableX, TangentX, Args...> &retract_X,
     const VariableX &x0, Args &...args,
     const std::optional<TNLSPreconditioner<VariableX, TangentX, Args...>>
         &precon = std::nullopt,
     const TNLSParams<Scalar> &params = TNLSParams<Scalar>(),
     const std::optional<
         TNLSUserFunction<VariableX, TangentX, VectorY, Scalar, Args...>>
         &user_function = std::nullopt) {

  /// Argument checking

  // Termination criteria

  if (params.max_computation_time < 0)
    throw std::invalid_argument(
        "Maximum computation time must be a nonnegative real value");

  if (params.root_tolerance < 0)
    throw std::invalid_argument(
        "Root tolerance must be a nonnegative real value");

  if (params.gradient_tolerance < 0)
    throw std::invalid_argument(
        "Gradient tolerance must be a nonnegative real value");

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

  if (params.Atol < 0)
    throw std::invalid_argument("Relative norm stopping tolerance Atol must be "
                                "a nonnegative real number");

  if (params.Acond_limit <= 0)
    throw std::invalid_argument(
        "Stopping criterion Acond_limit must be a positive real number");

  /// Declare and initialize some useful variables

  // Square root of machine precision for type Scalar
  Scalar sqrt_eps = sqrt(std::numeric_limits<Scalar>::epsilon());

  // Preconditioners (if any were supplied)
  LinearOperator<VariableX, TangentX, Args...> M, MT, Minv;

  // Output struct
  TNLSResult<VariableX, Scalar> result;

  // "Default" stopping condition (i.e. will
  // trigger if no other is)
  result.status = TNLSStatus::IterationLimit;

  // Current iterate and proposed next iterates;
  VariableX x, x_proposed;

  // Value of the (vector-valued) function F value at the current iterate and
  // proposed iterates
  VectorY Fx, Fx_proposed;

  // Squared norm of the (vector-valued) function F at the current and proposed
  // iterates
  Scalar Fx_norm, Fx_squared_norm, Fx_proposed_norm, Fx_proposed_squared_norm;

  // Trust-region radius
  Scalar Delta;

  // Newton step computed at x
  TangentX h;

  // Update step norm in the Euclidean and M-norms
  Scalar h_norm, h_M_norm;

  // Relative decrease between subsequent accepted function iterations
  Scalar relative_decrease;

  // Gradient of the squared-error loss L(x) = |F(X)| at the current
  // iterate x: gradL(x) := gradF(x)' * F(x) / |F(X)|
  TangentX gradLx;

  // Norm of gradfx
  Scalar gradLx_norm;

  // Jacobian operator and its adjoint, evaluated at the current iterate x
  Jacobian<VariableX, TangentX, VectorY, Args...> gradFx;
  JacobianAdjoint<VariableX, TangentX, VectorY, Args...> gradFxT;

  // Some useful constants for display purposes

  // Field with for displaying outer iterations
  size_t iter_field_width = floor(log10(params.max_iterations)) + 1;

  // Field width for displaying inner iterations
  size_t inner_iter_field_width = floor(log10(params.max_LSQR_iterations)) + 1;

  /// INITIALIZATION

  // Set initial iterate ...
  x = x0;

  // Function value ...
  Fx = F(x, args...);
  Fx_squared_norm = inner_product_Y(Fx, Fx, args...);
  Fx_norm = sqrt(Fx_squared_norm);

  // Jacobian and its adjoint ...
  std::tie(gradFx, gradFxT) = J(x, args...);

  // and gradient
  gradLx = gradFxT(x, Fx) / Fx_norm;
  gradLx_norm = sqrt(metric_X(x, gradLx, gradLx, args...));

  /// Set up function handles for inner LSQR linear system solver

  // Linear operator A (Jacobian gradF)
  Optimization::LinearAlgebra::LinearOperator<TangentX, VectorY, Args...> A;

  if (precon) {
    // We are right-preconditioning by M: A(x) = gradFx * M * v
    A = [&x, &gradFx, &precon](const TangentX &v, Args &...args) -> VectorY {
      return gradFx(x, precon->first(x, v, args...), args...);
    };
  } else {
    // No preconditioning: A = gradF * v
    A = [&x, &gradFx](const TangentX &v, Args &...args) -> VectorY {
      return gradFx(x, v, args...);
    };
  }

  Optimization::LinearAlgebra::LinearOperator<VectorY, TangentX, Args...> At;
  if (precon) {
    // We are right-preconditioning by M: At = Mt * gradFx' * w
    At = [&x, &gradFxT, &precon](const TangentX &w, Args &...args) -> VectorY {
      return precon->second(x, gradFxT(x, w, args...), args...);
    };
  } else {
    // No preconditioning: At = gradFx' * w
    At = [&x, &gradFxT](const VectorY &w, Args &...args) -> TangentX {
      return gradFxT(x, w, args...);
    };
  }

  // Inner product on T_x(X)
  Optimization::LinearAlgebra::InnerProduct<TangentX, Scalar, Args...>
      inner_product_X = [&x, &metric_X](const TangentX &v1, const TangentX &v2,
                                        Args &...args) -> Scalar {
    return metric_X(x, v1, v2, args...);
  };

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
      result.status = TNLSStatus::ElapsedTime;
      break;
    }

    // Record output
    result.time.push_back(elapsed_time);
    result.objective_values.push_back(Fx_norm);
    result.gradient_norms.push_back(gradLx_norm);
    result.trust_region_radius.push_back(Delta);

    if (params.log_iterates)
      result.iterates.push_back(x);

    if (params.verbose) {
      std::cout << "Iter: ";
      std::cout.width(iter_field_width);
      std::cout << iteration << ", time: " << elapsed_time << ", |F(x)|: ";
      std::cout.width(params.precision + 7);
      std::cout << Fx_norm << ", |grad|: " << gradLx_norm;
    }

    // Test absolute residual-based stopping criterion
    if (Fx_norm < params.root_tolerance) {
      result.status = TNLSStatus::Root;
      break;
    }

    // Test gradient-based stopping criterion
    if (gradLx_norm < params.gradient_tolerance) {
      result.status = TNLSStatus::Gradient;
      break;
    }

    /// STEP 2:  Solve the local trust-region subproblem at the current
    /// iterate LSQR

    // Set target relative reduction in residual of F(x) -- see the paper
    // "Inexact Newton Methods" by R.S. Dembo, S.C. Eisenstat, and T. Steihaug
    Scalar etak = std::min(std::pow(Fx_norm, params.theta), params.kappa_fgr);

    size_t inner_iterations;

    h = Optimization::LinearAlgebra::LSQR<TangentX, VectorY, Scalar, Args...>(
        A, At, -Fx, inner_product_X, inner_product_Y, args..., h_M_norm,
        inner_iterations, params.max_LSQR_iterations, params.lambda, etak,
        params.Atol, params.Acond_limit, Delta);

    if (precon) {
      // If we are using right-preconditioning, then the update returned by LSQR
      // is expressed in the preconditioned coordinate system defined by M^-1.
      // Therefore, we must recover h using M
      h = precon->first(x, h, args...);
    }

    // Compute norm of returned solution
    h_norm = sqrt(metric_X(x, h, h, args...));

    if (params.verbose) {
      std::cout << ", Delta: " << Delta << ", inner iters: ";
      std::cout.width(inner_iter_field_width);
      std::cout << inner_iterations << ", |h|: " << h_norm
                << ", |h|_M: " << h_M_norm;
    }

    /// STEP 3:  Update and evaluate trial point

    // Get trial point
    x_proposed = retract_X(x, h, args...);

    // Evalute objective at trial point
    Fx_proposed = F(x_proposed, args...);
    Fx_proposed_squared_norm =
        inner_product_Y(Fx_proposed, Fx_proposed, args...);
    Fx_proposed_norm = sqrt(Fx_proposed_squared_norm);

    // Compute predicted model decrease

    // Predicted (linearized) residual after applying update h
    VectorY r = gradFx(x, h, args...) + Fx;

    // Linearized squared residual norm
    Scalar r2 = inner_product_Y(r, r, args...);

    // Decreased in linearized quadratic model after applying update h
    Scalar dq = Fx_squared_norm - r2;

    // Actual decrease in residual norm
    Scalar dL = Fx_norm - Fx_proposed_norm;

    // Actual decrease in *squared* residual norm
    Scalar df2 = Fx_squared_norm - Fx_proposed_squared_norm;

    // Relative function decrease
    relative_decrease = dL / (sqrt_eps + Fx_norm);

    // Evaluate gain ratio
    Scalar rho = df2 / dq;

    if (params.verbose) {
      std::cout << ", dL: ";
      std::cout.width(params.precision + 7);
      std::cout << dL << ", rho: ";
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
    result.rho.push_back(rho);

    // Call the user-supplied function to provide access to internal algorithm
    // state, and check for user-requested termination
    if (user_function) {
      if ((*user_function)(iteration, elapsed_time, x, Fx, gradFx, gradFxT,
                           Delta, inner_iterations, h, dL, rho, step_accepted,
                           args...)) {
        result.status = TNLSStatus::UserFunction;
        break;
      }
    }

    /// Update cached values if the iterate is accepted
    if (step_accepted) {
      // Accept iterate and cache values ...
      x = std::move(x_proposed);
      Fx = std::move(Fx_proposed);
      Fx_squared_norm = Fx_proposed_squared_norm;
      Fx_norm = Fx_proposed_norm;

      // Test relative decrease-based stopping criterion
      if (relative_decrease < params.relative_decrease_tolerance) {
        result.status = TNLSStatus::RelativeDecrease;
        break;
      }

      // Test stepsize-based stopping criterion ...
      if (h_norm < params.stepsize_tolerance) {
        result.status = TNLSStatus::Stepsize;
        break;
      }

      // ... and recompute the Jacobian and its adjoint at the updated iterate
      std::tie(gradFx, gradFxT) = J(x, args...);

      gradLx = gradFxT(x, Fx, args...) / Fx_norm;
      gradLx_norm = sqrt(metric_X(x, gradLx, gradLx, args...));

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
        result.status = TNLSStatus::TrustRegion;
        break;
      }
    } // trust-region update

    if (params.verbose)
      std::cout << std::endl;
  } // end trust-region iterations
  result.elapsed_time = Stopwatch::tock(start_time);

  // Record output
  result.x = x;
  result.f = Fx_norm;
  result.gradfx_norm = gradLx_norm;

  if (params.verbose) {
    std::cout << std::endl
              << std::endl
              << "Optimization finished!" << std::endl;

    // Print the reason for termination
    switch (result.status) {
    case TNLSStatus::Root:
      std::cout << "Found root of F(x) = 0! (Residual norm: " << Fx_norm << ")"
                << std::endl;
      break;
    case TNLSStatus::Gradient:
      std::cout << "Found first-order critical point! (Gradient norm: "
                << gradLx_norm << ")" << std::endl;
      break;
    case TNLSStatus::RelativeDecrease:
      std::cout
          << "Algorithm terminated due to insufficient relative decrease: "
          << relative_decrease << " < " << params.relative_decrease_tolerance
          << std::endl;
      break;
    case TNLSStatus::Stepsize:
      std::cout
          << "Algorithm terminated due to excessively small step size: |h| = "
          << h_norm << " < " << params.stepsize_tolerance << std::endl;
      break;
    case TNLSStatus::TrustRegion:
      std::cout << "Algorithm terminated due to excessively small trust region "
                   "radius: "
                << Delta << " < " << params.Delta_tolerance << std::endl;
      break;
    case TNLSStatus::IterationLimit:
      std::cout << "Algorithm exceeded maximum number of outer iterations"
                << std::endl;
      break;
    case TNLSStatus::ElapsedTime:
      std::cout << "Algorithm exceeded maximum allowed computation time: ("
                << result.elapsed_time << " > " << params.max_computation_time
                << " seconds)" << std::endl;
      break;
    case TNLSStatus::UserFunction:
      std::cout
          << "Algorithm terminated due to user-supplied stopping criterion"
          << std::endl;
      break;
    }

    std::cout << "Final objective value: " << result.f << std::endl;
    std::cout << "Norm of Riemannian gradient: " << result.gradfx_norm
              << std::endl;
    std::cout << "Total elapsed computation time: " << result.elapsed_time
              << " seconds" << std::endl
              << std::endl;

    // Reset std::cout output stream to default display parameters
    std::cout << std::defaultfloat;
    std::cout.precision(6);
  }

  return result;
}

/** These next functions provide a convenient specialization/simplification of
 * the Riemannian truncated-Newton least-squares interface for the (common) use
 * case of optimization over Euclidean spaces.  These functions make the
 * following assumptions:
 *
 * - The metric is the standard Euclidean metric:  g(x, v1, v2) := <v1, v2>
 *   for all x in X
 *
 * - The retraction is the standard Euclidean retraction:  R_x(v) := x + v for
 *   all x in X.
 *
 * - We exploit the global parallelism and self-duality of Euclidean spaces to
 *   represent both *points* x in Euclidean space and *tangent vectors* v
 *   using the *same* data type (Vector).
 */

template <typename Vector, typename Scalar = double, typename... Args>
TNLSResult<Vector, Scalar> EuclideanTNLS(
    const Mapping<Vector, Vector, Args...> &F,
    const JacobianPairFunction<Vector, Vector, Vector> &J, const Vector &x0,
    Args &...args,
    const std::optional<TNLSPreconditioner<Vector, Vector, Args...>> &precon =
        std::nullopt,
    const TNLSParams<Scalar> &params = TNLSParams<Scalar>(),
    const std::optional<
        TNLSUserFunction<Vector, Vector, Vector, Scalar, Args...>>
        &user_function = std::nullopt) {

  /// Run TNLS algorithm using these Euclidean operators
  return TNLS<Vector, Vector, Vector, Scalar, Args...>(
      F, J, EuclideanMetric<Vector, Scalar, Args...>,
      EuclideanInnerProduct<Vector, Scalar, Args...>,
      EuclideanRetraction<Vector, Args...>, x0, args..., precon, params,
      user_function);
}

} // namespace Riemannian
} // namespace Optimization
