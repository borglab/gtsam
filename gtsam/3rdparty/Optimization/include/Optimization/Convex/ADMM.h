/** This header file provides several lightweight alias templates and template
 * classes implementing the (optionally Nestorov-accelerated) Alternating
 * Direction Method of Multipliers (ADMM) algorithm for solving convex
 * minimization problems of the form:
 *
 * min f(x) + g(y)
 *
 * s.t. Ax + By = c
 *
 * via operator splitting.  This implementation is based upon the one described
 * in Section 3.1 of "Distributed Optimization and Statistical Learning via the
 * Alternating Direction Method of Multipliers", by S. Boyd, N. Parikh, E. Chu,
 * B. Peleato, and J. Eckstein, and Algorithm 8 of the paper "Fast Alternating
 * Direction Optimization Methods", by T. Goldstein, B. O'Donoghue, S. Setzer,
 * and R. Baraniuk.
 *
 * Copyright (C) 2018 by David M. Rosen (dmrosen@mit.edu)
 */

#pragma once

#include "Optimization/Convex/Concepts.h"
#include "Optimization/Util/Stopwatch.h" // Useful timing functions

#include <algorithm>
#include <cmath>
#include <experimental/optional>
#include <functional>
#include <iostream>
#include <limits>
#include <utility>

namespace Optimization {
namespace Convex {

/** Alias templates for functions that return the minimizers of the augmented
 * Lagrangian:
 *
 * L_rho(x, y, lambda) := f(x) + g(y) + (rho / 2) * | Ax + By - c + (1/rho) *
 *                                                               lambda |_2^2
 *
 * considered as a function of its first and second arguments, respectively.
 */

template <typename VariableX, typename VariableY, typename VariableR,
          typename Scalar = double, typename... Args>
using AugLagMinX = std::function<VariableX(
    const VariableY &y, const VariableR &lambda, Scalar rho, Args &... args)>;

template <typename VariableX, typename VariableY, typename VariableR,
          typename Scalar = double, typename... Args>
using AugLagMinY = std::function<VariableY(
    const VariableX &x, const VariableR &lambda, Scalar rho, Args &... args)>;

/** An alias template for a user-definable function that can be
 * used to access various interesting bits of information about the internal
 * state of the ADMM algorithm as it runs.  More precisely, this function is
 * called at the end of each iteration, and is provided access to
 * the following quantities:
 *
 * i: index of current iteration
 * t: total elapsed computation time at the *end* of the current iteration
 * x: value of the iterate x at the *end* of the current iteration
 * y: value of the iterate y at the *end* of the current iteration
 * lambda: value of the dual variable lambda at the *end* of the current
 * iteration
 * rho: value of the penalty parameter at the *end* of the current iteration
 * r: primal residual at the *end* of the current iteration
 * s: dual residual at the *end* of the current iteration
 */
template <typename VariableX, typename VariableY, typename VariableR,
          typename Scalar = double, typename... Args>
using ADMMUserFunction =
    std::function<void(size_t i, double t, const VariableX &x,
                       const VariableY &y, const VariableR &lambda, Scalar rho,
                       const VariableR &r, const VariableX &s, Args &... args)>;

/** A simple enumeration type describing the strategy used to adapt the penalty
 * parameter rho in the augmented Lagrangian */

enum class ADMMPenaltyAdaptation {
  /** Vanilla ADMM: No parameter adaptation */
  None,

  /** Use the primal- and dual-residual balancing approach described in the
     paper "Alternating Direction Method with Self-Adaptive Penalty Parameters",
     by B. He, H. Yang, and S. Wang */
  Residual_Balance,

  /** Use the spectral penalty (Barzilai-Borwein-based) selection method
     described in the paper "Adaptive ADMM with Spectral Penalty Parameter
     Selection", by Z. Xu, M.A.T. Figueiredo, and T. Goldstein.*/
  // Spectral
};

enum class ADMMMode {
  /** Vanilla ADMM */
  Simple,

  /** Nestorov-accelerated */
  Accelerated,
};

template <typename Scalar = double> struct ADMMParams : public OptimizerParams {

  /// PENALTY PARAMETER SETTINGS

  /** (Initial) value of penalty parameter rho */
  Scalar rho = 1.0;

  /** Adaptation strategy for penalty parameter (currently only supported in
   * 'simple' mode) */
  ADMMPenaltyAdaptation penalty_adaptation_mode = ADMMPenaltyAdaptation::None;

  /** If penalty_adaptation_mode != None, this parameter controls how
   * frequently (in terms of number of iterations) the penalty parameter is
   * updated */
  size_t penalty_adaptation_period = 2;

  /** This value sets an upper limit (in terms of number of iterations) on the
   * window within which the augmented Lagrangian penalty parameter will be
   * adjusted -- this is to ensure that the penalty parameter will eventually be
   * constant, so that the ADMM algorithm is guaranteed to converge */
  size_t penalty_adaptation_window = std::numeric_limits<size_t>::max();

  /** If the 'Residual_Balance' adaptation strategy is used, this value sets the
   * threshold for the maximum admissible ratio between the primal and dual
   * residuals before increasing or decreasing the penalty parameter (cf.
   * equation (3.13) of "Distributed Optimization and Statistical Learning via
   * the Alternating Direction Method of Multipliers).  This value should be
   * positive, and greater than 1. */
  double residual_balance_mu = 10;

  /** If the 'Residual_Balance' adaptation strategy is used, this is the
   * multiplicative factor used to increase or decrease the penalty parameter
   * when cf. equation (3.13) of "Distributed Optimization and Statistical
   * Learning via the Alternating Direction Method of Multipliers).  This value
   * should be positive, and greater than 1. */
  double residual_balance_tau = 2;

  /// NESTEROV ACCELERATION

  /** ADMM mode: Simple (vanilla) or Nestorov-accelerated */
  ADMMMode mode = ADMMMode::Simple;

  /** Threshold on the fractional decrease in the merit function ck defined in
  eq. (30) of "Fast Alternating Direction Optimization Methods" that must be
  obtained in order to accept an accelerated iteration.  This value should be in
  the range (0, 1), and is best taken close to 1 (to provide a permissive
  acceptance criterion for acceleration) */
  Scalar eta = .999;

  /** Termination criteria:
   *
   * We employ a termination criterion based upon the primal and dual residuals:
   *
   * r_k := Ax + By - c
   * s_k := rho * A^t * B * (y_k - y_{k-1})
   *
   * as suggested in Section 3.3.1 of "Distributed Optimization and Statistical
   * Learning via the Alternating Direction Method of Multipliers"; namely, we
   * define combined (absolute + relative) primal and dual residual stopping
   * tolerances eps_pri_k and eps_dual_k at iteration k according to:
   *
   * eps_pri_k := eps_abs_pri + eps_rel * max { |Ax_k|_2, |By_k|_2, |c|_2 }
   *
   * eps_dual_k := eps_abs_dual+ eps_rel * |A^t lambda_k|_2
   *
   * where eps_abs_pri, eps_abs_dual, and eps_rel are user-supplied tolerances,
   * and terminate if:
   *
   * |r_k|_2 <= eps_pri_k AND |s_k|_2 <= eps_dual_k
   */

  /** Absolute primal stopping tolerance */
  Scalar eps_abs_pri = 1e-2;

  /** Absolute dual stopping tolerance */
  Scalar eps_abs_dual = 1e-2;

  /** Relative stopping tolerance */
  Scalar eps_rel = 1e-3;
};

/** Enum class that describes the termination status of the algorithm */
enum class ADMMStatus {
  /** ADMM algorithm terminated because the residual stopping criteria were
     satisfied */
  RESIDUAL_TOLERANCE,

  /** ADMM algorithm terminated because it exhausted the alloted number of
     iterations before achieving the desired residual tolerances */
  ITERATION_LIMIT,

  /** ADMM algorithm terminated because it exceeded the alloted computation time
     before achieving the desired residual tolerances */
  ELAPSED_TIME
};

/** Enum class that describes the type of each iteration of the ADMM algorithm
 */
enum class ADMMIterationType {
  /** The iteration produced a (nontrivially) Nestorov-accelerated step */
  Accelerated,

  /** The iteration produced a standard (unaccelerated) ADMM step */
  Standard,

  /** The iteration was a restart iteration (only occurs when using accelerated
     ADMM) */
  Restart
};

/** A useful struct used to hold the output of the ADMM algorithm */
template <typename VariableX, typename VariableY, typename VariableR,
          typename Scalar = double>
struct ADMMResult
    : OptimizerResult<std::tuple<VariableX, VariableY, VariableR>, Scalar> {

  /** The stopping condition that triggered algorithm termination */
  ADMMStatus status;

  /** Primal residual at the end of each iteration */
  std::vector<double> primal_residuals;

  /** Dual residual at the end of each iteration */
  std::vector<double> dual_residuals;

  /** Value of (the square root of) the monotonically-nonincreasing
   * convergence parameter
   *
   * m_k := sqrt( rho * |B(y- y_{k-1})|^2 + rho * |r_k|^2 )
   *
   * defined in the paper "On Non-Ergodic Convergence Rate of Douglas-Rachford
   * Alternating Direction Method of Multipliers", by B.S. He and X. Yuan (cf.
   * eq. (3.6) and Theorem 5.1) at the end of each iteration */
  std::vector<Scalar> m_k;

  /** The sequence of augmented Lagrangian penalty parameters employed by
   * the algorithm at each iteration.*/
  std::vector<Scalar> penalty_parameters;

  /** A vector containing the classification of each iteration performed by the
   * ADMM algorithm */
  std::vector<ADMMIterationType> iteration_types;
};

/** Helper function:  This function implements the residual-balancing updating
 * strategy for the augmented Lagrangian penalty parameter (cf. equation (3.13)
 * of "Distributed Optimization and Statistical Learning via the Alternating
 * Direction Method of Multipliers) */
template <typename Scalar = double>
Scalar residual_balance_penalty_parameter_update(Scalar primal_residual,
                                                 Scalar dual_residual,
                                                 Scalar mu, Scalar tau,
                                                 Scalar rho) {
  if (primal_residual > mu * dual_residual)
    return tau * rho;
  else if (dual_residual > mu * primal_residual)
    return rho / tau;
  else
    return rho;
}

template <typename VariableX, typename VariableY, typename VariableR,
          typename Scalar = double, typename... Args>
ADMMResult<VariableX, VariableY, VariableR, Scalar>
ADMM(const AugLagMinX<VariableX, VariableY, VariableR, Scalar, Args...> &minLx,
     const AugLagMinY<VariableX, VariableY, VariableR, Scalar, Args...> &minLy,
     const LinearOperator<VariableX, VariableR, Args...> &A,
     const LinearOperator<VariableY, VariableR, Args...> &B,
     const LinearOperator<VariableR, VariableX, Args...> &At,
     const InnerProduct<VariableX, Scalar, Args...> inner_product_x,
     const InnerProduct<VariableR, Scalar, Args...> inner_product_r,
     const VariableR &c, const VariableX &x0, const VariableY &y0, Args... args,
     const ADMMParams<Scalar> &params = ADMMParams<Scalar>(),
     const std::experimental::optional<
         ADMMUserFunction<VariableX, VariableY, VariableR, Scalar, Args...>>
         &user_function = std::experimental::nullopt) {

  /// Declare some useful variables

  /// Iterates required by main (simple) ADMM loop

  // Current iterates
  VariableX x;
  VariableY y;
  VariableR lambda;

  // Current value of augmented Lagrangian penalty parameter
  Scalar rho;

  // Cache variables for intermediate products Ax, By
  VariableR Ax, By;

  // Cache variable for product of B with previous iterate y_prev; used in the
  // computation of the monotonically non-increasing convergence measure m_k
  VariableR By_prev;

  // Primal residual vector
  VariableR r;

  // Dual residual vector
  VariableX s;

  // Previous iterate of y (needed for the dual residual computation)
  VariableY y_prev;

  Scalar c_norm = sqrt(inner_product_r(c, c, args...));

  ADMMIterationType iteration_type;

  // Primal and dual residual values
  Scalar primal_residual, dual_residual;

  // Monotonically non-increasing convergence measure defined in the paper "On
  // Non-Ergodic Convergence Rate of Douglas-Rachford Alternating Direction
  // Method of Multipliers", by B.S. He and X. Yuan (cf. eq. (3.6) and
  // Theorem 5.1).
  //
  //  m_k := sqrt( rho * |B(y_k - y_{k-1})|^2 + rho * |r_k|^2 )
  Scalar m_k, m_kminus1;

  /// Additional variables needed for accelerated ADMM (only used if
  /// ADMMMode == Accelerated)

  VariableY y_hat;       // Forward-predicted y-variable value
  VariableR lambda_hat;  // Forward-predicted dual variable value
  VariableR lambda_prev; // Previous value of dual variable

  // Acceleration forward-prediction weighting sequence
  Scalar alpha_k, alpha_kplus1;

  /// Output struct
  ADMMResult<VariableX, VariableY, VariableR> result;
  result.status = ADMMStatus::ITERATION_LIMIT;

  /// INITIALIZATION
  rho = params.rho;
  x = x0;
  y = y0;
  Ax = A(x, args...);
  By = B(y, args...);
  lambda = rho * (Ax + By - c);
  y_prev = y0;
  By_prev = B(y_prev);

  iteration_type =
      (params.mode == ADMMMode::Accelerated ? ADMMIterationType::Restart
                                            : ADMMIterationType::Standard);

  // Additional initializations for accelerated ADMM
  if (params.mode == ADMMMode::Accelerated) {
    y_hat = y;
    lambda_hat = lambda;
    lambda_prev = lambda;
    m_kminus1 = std::numeric_limits<double>::max();

    alpha_k = 1.0;
  }

  if (params.verbose) {
    std::cout << std::scientific;
    std::cout.precision(params.precision);
  }
  // Field width for displaying outer iterations
  size_t iter_field_width = floor(log10(params.max_iterations)) + 1;

  if (params.verbose)
    std::cout << "ADMM optimization: " << std::endl << std::endl;

  /// ITERATE!
  auto start_time = Stopwatch::tick();
  for (size_t iter = 0; iter < params.max_iterations; ++iter) {

    /// ADMM ITERATION

    /// UPDATE X
    // Update x by minimizing augmented Lagrangian with respect to x (eq. 3.2)
    x = minLx((params.mode == ADMMMode::Accelerated ? y_hat : y),
              (params.mode == ADMMMode::Accelerated ? lambda_hat : lambda), rho,
              args...);

    /// UPDATE Y
    // Update y by minimizing augmented Lagrangian with respect to y (eq. 3.3)
    y = minLy(x, (params.mode == ADMMMode::Accelerated ? lambda_hat : lambda),
              rho, args...);

    /// COMPUTE PRIMAL RESIDUAL VECTOR
    // Compute primal residual vector (cf. Sec. 3.3 of "Distributed Optimization
    // and Statistical Learning via the Alternating Direction Method of
    // Multipliers")
    Ax = A(x, args...);
    By = B(y, args...);

    r = Ax + By - c;
    primal_residual = sqrt(inner_product_r(r, r, args...));

    /// UPDATE LAMBDA
    // Update dual variable lambda (eq. 3.4), w/ r = Ax + By -c
    lambda =
        (params.mode == ADMMMode::Accelerated ? lambda_hat : lambda) + rho * r;

    /// COMPUTE MONOTONIC CONVERGENCE MEASURE mk:
    VariableR By_diff =
        By -
        (params.mode == ADMMMode::Accelerated ? B(y_hat, args...) : By_prev);

    m_k = sqrt(rho * inner_product_r(r, r, args...) +
               rho * inner_product_r(By_diff, By_diff, args...));

    /// ACCELERATED ADMM UPDATE
    // Compute the Nestorov-accelerated forward-prediction step described in
    // Algorithm 8 of "Fast Alternating Direction Optimization Methods", by T.
    // Goldstein, B. O'Donoghue, S. Setzer, and R. Baraniuk
    if (params.mode == ADMMMode::Accelerated) {

      /// Test acceptance of the current iteration
      if (m_k < params.eta * m_kminus1) {
        alpha_kplus1 = (1 + sqrt(1 + 4 * alpha_k * alpha_k)) / 2;

        // Forward-predict variable y
        y_hat = y + ((alpha_k - 1) / alpha_kplus1) * (y - y_prev);

        // Forward-predict variable lambda
        lambda_hat =
            lambda + ((alpha_k - 1) / alpha_kplus1) * (lambda - lambda_prev);

        // If the *previous* iteration was a restart iteration ...
        if (iteration_type == ADMMIterationType::Restart) {
          // ... then alpha_k == 1.0, so this iteration is a Standard ADMM step
          iteration_type = ADMMIterationType::Standard;
        } else {
          // If the *previous* iteration was NOT a restart iteration, then it
          // was either a standard or accelerated ADMM step, in which case
          // *this* step will have (nontrivial) Nestorov acceleration
          iteration_type = ADMMIterationType::Accelerated;
        } // if (iteration type)

      } else {
        // Reject step and restart

        alpha_kplus1 = 1.0;
        y_hat = y_prev;
        lambda_hat = lambda;
        m_k = m_kminus1;

        iteration_type = ADMMIterationType::Restart;
      } // test acceptance of step

    } // Accelerated ADMM update

    /// COMPUTE DUAL RESIDUAL VECTOR

    // Compute dual residual (cf. Sec. 3.3 of "Distributed Optimization and
    // Statistical Learning via the Alternating Direction Method of
    // Multipliers").  Note that here we use the modified dual residual shown
    // in eq. (29) of "Fast Alternating Direction Optimization Methods" for
    // the case in which params.mode == Accelerated).

    if (iteration_type != ADMMIterationType::Restart) {
      s = rho * At(By - (iteration_type == ADMMIterationType::Accelerated
                             ? B(y_hat, args...)
                             : By_prev),

                   args...);
      dual_residual = sqrt(inner_product_x(s, s, args...));
    }

    /// End of ADMM step computations

    // Record the elapsed time at the END of this iteration
    double elapsed_time = Stopwatch::tock(start_time);

    /// Display output for this iteration
    // Display information about this iteration, if requested
    if (params.verbose) {
      std::cout << "Iter: ";
      std::cout.width(iter_field_width);
      std::cout << iter << ", time: " << elapsed_time;
      std::cout << ", primal residual: ";
      std::cout.width(params.precision + 7);
      std::cout << primal_residual << ", dual residual:";
      std::cout.width(params.precision + 7);
      std::cout << dual_residual << ", m: ";
      std::cout.width(params.precision + 7);
      std::cout << m_k << ", penalty:";
      std::cout.width(params.precision + 7);
      std::cout << rho;

      // Print out iteration type
      switch (iteration_type) {
      case ADMMIterationType::Accelerated:
        std::cout << " (accelerated)";
        break;
      case ADMMIterationType::Standard:
        std::cout << " (standard)";
        break;
      case ADMMIterationType::Restart:
        std::cout << " (restart)";
        break;
      }
      std::cout << std::endl;
    } // if (params.verbose)

    /// RECORD ITERATION RESULTS
    result.time.push_back(elapsed_time);
    result.primal_residuals.push_back(primal_residual);
    result.dual_residuals.push_back(dual_residual);
    result.m_k.push_back(m_k);
    result.penalty_parameters.push_back(rho);
    result.iteration_types.push_back(iteration_type);

    if (params.log_iterates)
      result.iterates.emplace_back(
          x, (params.mode == ADMMMode::Accelerated ? y_hat : y),
          (params.mode == ADMMMode::Accelerated ? lambda_hat : lambda));

    /// TEST STOPPING CRITERIA
    // Test elapsed-time-based stopping criterion
    if (elapsed_time > params.max_computation_time) {
      result.status = ADMMStatus::ELAPSED_TIME;
      break;
    }

    /// Compute primal stopping tolerance (Sec. 3.3.1)
    Scalar Ax_norm = sqrt(inner_product_r(Ax, Ax, args...));
    Scalar By_norm = sqrt(inner_product_r(By, By, args...));
    Scalar eps_primal =
        params.eps_abs_pri +
        params.eps_rel * std::max<Scalar>({Ax_norm, By_norm, c_norm});

    /// Compute dual stopping tolerance (Sec. 3.3.1)
    const VariableX At_lambda = At(lambda, args...);
    Scalar At_lambda_norm =
        sqrt(inner_product_x(At_lambda, At_lambda, args...));
    Scalar eps_dual = params.eps_abs_dual + params.eps_rel * At_lambda_norm;

    /// Test residual-based stopping criterion
    if ((primal_residual < eps_primal) && (dual_residual < eps_dual)) {
      result.status = ADMMStatus::RESIDUAL_TOLERANCE;
      break;
    }

    /// PENALTY PARAMETER UPDATE
    if ((params.penalty_adaptation_mode ==
         ADMMPenaltyAdaptation::Residual_Balance) &&
        ((iter % params.penalty_adaptation_period) == 0) &&
        (iter < params.penalty_adaptation_window)) {

      // Residual balancing (eq. 3.13)
      rho = residual_balance_penalty_parameter_update(
          primal_residual, dual_residual, params.residual_balance_mu,
          params.residual_balance_tau, rho);

      if (params.mode == ADMMMode::Accelerated) {
        // Changing the penalty parameter completely invalidates the previous
        // convergence measure, so treat this as a restart iteration
        alpha_kplus1 = 1.0;
        y_hat = y_prev;
        lambda_hat = lambda;

        iteration_type = ADMMIterationType::Restart;
      }

    } // penalty parameter update

    /// CACHE PARAMETERS AND PREPARE FOR NEXT ITERATION
    y_prev = y;
    By_prev = By;

    if (params.mode == ADMMMode::Accelerated) {
      lambda_prev = lambda;
      alpha_k = alpha_kplus1;
      m_kminus1 = (iteration_type == ADMMIterationType::Restart
                       ? std::numeric_limits<Scalar>::max()
                       : m_k);
    }

    /// Call user-supplied function to provide access to internal algorithm
    /// state, if requested
    if (user_function)
      (*user_function)(
          iter, Stopwatch::tock(start_time), x,
          (params.mode == ADMMMode::Accelerated ? y_hat : y),
          (params.mode == ADMMMode::Accelerated ? lambda_hat : lambda), rho, r,
          s, args...);

  } // ADMM ITERATIONS

  /// RECORD FINAL OUTPUT
  result.x = {x, (params.mode == ADMMMode::Accelerated ? y_hat : y),
              (params.mode == ADMMMode::Accelerated ? lambda_hat : lambda)};
  result.elapsed_time = Stopwatch::tock(start_time);

  /// Print final output, if requested
  if (params.verbose) {
    std::cout << std::endl << "Optimization finished!" << std::endl;

    // Print the reason for termination
    switch (result.status) {
    case ADMMStatus::RESIDUAL_TOLERANCE:
      std::cout << "Found minimizer!" << std::endl;
      break;
    case ADMMStatus::ITERATION_LIMIT:
      std::cout << "Algorithm exceeded maximum number of outer iterations"
                << std::endl;
      break;
    case ADMMStatus::ELAPSED_TIME:
      std::cout << "Algorithm exceeded maximum allowed computation time: "
                << result.elapsed_time << " > " << params.max_computation_time
                << std::endl;
      break;
    }

    std::cout << "Final primal residual: " << primal_residual
              << ", final dual residual: " << dual_residual
              << ", total elapsed computation time: " << result.elapsed_time
              << " seconds" << std::endl;
  } // if(verbose)

  return result;
} // namespace Convex

/** This next function provides a convenient specialization of the ADMM
 * interface for the (common) use case in which a single data type is used to
 * represent each variable */

template <typename Variable, typename Scalar = double, typename... Args>
ADMMResult<Variable, Variable, Variable, Scalar>
ADMM(const AugLagMinX<Variable, Variable, Variable, Scalar, Args...> &minLx,
     const AugLagMinY<Variable, Variable, Variable, Scalar, Args...> &minLy,
     const LinearOperator<Variable, Variable, Args...> &A,
     const LinearOperator<Variable, Variable, Args...> &B,
     const LinearOperator<Variable, Variable, Args...> &At,
     const InnerProduct<Variable, Scalar, Args...> inner_product,
     const Variable &c, const Variable &x0, const Variable &y0, Args... args,
     const ADMMParams<Scalar> &params = ADMMParams<Scalar>(),
     const std::experimental::optional<
         ADMMUserFunction<Variable, Variable, Variable, Scalar, Args...>>
         &user_function = std::experimental::nullopt) {
  return ADMM<Variable, Variable, Variable, Scalar, Args...>(
      minLx, minLy, A, B, At, inner_product, inner_product, c, x0, y0, args...,
      params, user_function);
}

} // namespace Convex
} // namespace Optimization
