/** This header file provides several Krylov-subspace-based iterative
 * linear-algebra methods commonly used in the implementation of optimization
 * algorithms.
 *
 * Copyright (C) 2017 - 2022 by David M. Rosen (dmrosen@mit.edu)
 */

#pragma once

#include "Optimization/LinearAlgebra/Concepts.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>

namespace Optimization {

namespace LinearAlgebra {

/** An alias template for a user-definable function that can be
 * used to access various interesting bits of information about the internal
 * state of the Steihaug-Toint truncated preconditioned projected
 * conjugate-gradient algorithm as it runs. Here:
 *
 * - k: Index of current iteration
 * - g: Gradient vector for quadratic model
 * - H: Hessian operator for quadratic model
 * - P: Optional preconditioning operator
 * - At: Optional transpose of constraint operator
 * - sk: The value of the estimated solution at the *start* of the current
 *   iteration
 * - rk: Value of the residual rk := g + H*sk at the *start of the current
 *   iteration
 * - vk: Projected preconditioned residual at the *start* of the current
 *   iteration
 * - pk: The update direction computed during this iteration
 * - alpha_k: The steplength along pk computed during this iteration
 *
 * This function is called at the end of each iteration, after all of the
 * above-referenced quantities have been computed, but *before* the update step
 * alpha_k * pk computed during this iteration is to update the solution sk.
 *
 * This function may also return the Boolean value 'true' in order to
 * terminate the STPCG algorithm; this provides a convenient means of
 * implementing a custom (user-definable) stopping criterion.
 */
template <typename Vector, typename Multiplier, typename Scalar = double,
          typename... Args>
using STPCGUserFunction = std::function<bool(
    size_t k, const Vector &g,
    const SymmetricLinearOperator<Vector, Args...> &H,
    const std::optional<
        LinearOperator<Vector, std::pair<Vector, Multiplier>, Args...>> &P,
    const std::optional<LinearOperator<Multiplier, Vector, Args...>> &At,
    const Vector &sk, const Vector &rk, const Vector &vk, const Vector &pk,
    Scalar alpha_k, Args &... args)>;

/** An alias template for preconditioners used in conjunction with the
 * Steihaug-Toint truncated preconditioned projected conjugate-gradient
 * algorithm for solving trust-region problems of the form:
 *
 * min_s <g, s> + (1/2)<s, H*s>
 * s.t.  A*s = 0
 *       |s|_P <= Delta
 *
 * A preconditioner P of this type must be a linear map that accepts
 * as input a vector s of the same type as the decision variable of the
 * minimization problem, and returns a *pair* of values:
 *
 * P(s) = (v, lambda)
 *
 * that satisfy the linear system:
 *
 * [M A'][v] = [s]
 * [A 0 ][l] = [0]
 *
 * where M is a positive definite approximation of H (used to precondition the
 * projected CG algorithm).
 */
template <typename Vector, typename Multiplier, typename... Args>
using STPCGPreconditioner =
    LinearOperator<Vector, std::pair<Vector, Multiplier>, Args...>;

/** This function implements the Steihaug-Toint truncated preconditioned
 * projected conjugate-gradient algorithm for approximately solving a
 * trust-region problem of the form:
 *
 *   min_s <g, s> + (1/2)<s, H*s>
 *   s.t.  A*s = 0
 *         |s|_P <= Delta
 *
 * This specific implementation is based upon Algorithms 5.4.2 and 7.5.1 in
 * the reference "Trust-Region Methods" by Conn, Gould, and Toint, and
 * Algorithms 6.1 and 6.3 of the paper "On the Solution of Equality
 * Constrained Quadratic Programming Problems Arising in Optimization" by
 * N.I.M. Gould, M.E. Hribar, and J. Nocedal.
 *
 * Here:
 *
 * - g is the gradient of the model objective
 * - H is a symmetric linear operator defining the Hessian of the model
 *   objective
 * - inner_product is the inner product operator for the space containing
 *   the trust-region subproblem's decision variable s.
 * - P is an optional constraint preconditioning operator that accepts as
 *   input a vector s and returns the pair
 *
 *   P(s) = (x, lambda)
 *
 *   where x and lambda are the (unique) solution of the following linear
 *   system:
 *
 *   [M A'][x] = [s]
 *   [A 0 ][l]   [0]
 *
 *   and M is a positive definite preconditioner for H.  Note that in the
 *   case no linear constraints are present, the projected preconditioning
 *   operator P reduces to P(s) = M^{-1} s, where M is a positive-definite
 *   approximation of H.  If this argument is omitted, a default (no-op)
 *   preconditioner is used (corresponding to M = I).
 *
 *   || s ||_M <= Delta
 *
 * - At is an optional linear operator that computes the transpose AT : L ->
 *   S of the linear operator A : S -> L whose null space determines the
 *   feasible set of the trust-region subproblem.  Omission of this argument
 *   indicates that the trust-region subproblem has no linear constraints.
 *
 * - update_step_M_norm is a return value that gives the norm || h ||_M of
 *   the update step in the M-norm determined by the preconditioner
 *
 * - num_iterations is a return value that gives the number of iterations
 *   executed by the algorithm
 *
 * - Delta is the radius of the trust-region in the M-norm determined by the
 *   positive-definite matrix M defining the constraint preconditioner P:
 *
 *   |s|_M = sqrt(s'*M*s)
 *
 * - max_iterations is the maximum admissible number of iterations for the
 *   STPCG algorithm
 *
 * - kappa_fgr and theta are parameters that control the stopping criteria
 *   of the algorithm; termination occurs whenever the residual r_k, as
 *   measured in the P-norm determined by the constraint preconditioner P:
 *
 *     || r ||_P := sqrt(r'*P_x(r))
 *
 *   satisfies:
 *
 *     || r_k ||_P <= ||r_0||_P min [kappa, || r_0 ||_P^theta ]
 *
 *   (cf. Algorithm 7.5.1 of "Trust-Region Methods").  kappa and theta should
 *   both be in the range (0, 1)
 *
 * - user_function is an optional user-defined function that can be used to
 *   inspect various variables of interest as the algorithm runs, and
 *   (optionally) to define a custom stopping criterion for the algorithm.
 *
 * - epsilon is a numerical tolerance for determining whether a vector lies
 *   in the kernel of  the operator H, defined as |H*x|/|x| < epsilon.
 */
template <typename Vector, typename Multiplier, typename Scalar = double,
          typename... Args>
Vector STPCG(
    const Vector &g, const SymmetricLinearOperator<Vector, Args...> &H,
    const InnerProduct<Vector, Scalar, Args...> &inner_product, Args &... args,
    Scalar &update_step_M_norm, size_t &num_iterations, Scalar Delta,
    size_t max_iterations = 1000, Scalar kappa_fgr = .1, Scalar theta = .5,
    const std::optional<STPCGPreconditioner<Vector, Multiplier, Args...>> &P =
        std::nullopt,
    const std::optional<LinearOperator<Multiplier, Vector, Args...>> &At =
        std::nullopt,
    const std::optional<STPCGUserFunction<Vector, Multiplier, Scalar, Args...>>
        &user_function = std::nullopt,
    Scalar epsilon = 1e-8) {

  /// Argument checking

  if (Delta <= 0)
    throw std::invalid_argument(
        "Trust-region radius (Delta) must be a positive real value");

  if (max_iterations < 0)
    throw std::invalid_argument("Maximum number of iterations (max_iterations) "
                                "must be a nonnegative integer");

  if ((kappa_fgr < 0) || (kappa_fgr >= 1))
    throw std::invalid_argument(
        "Target fractional reduction of the gradient norm "
        "(kappa_fgr) must be a real value in the range [0,1)");

  if ((theta < 0) || (theta > 1))
    throw std::invalid_argument(
        "Target superlinear convergence rate (theta) must "
        "be a real value in the range [0,1]");

  if ((epsilon <= 0) || (epsilon >= 1))
    throw std::invalid_argument(
        "Relative norm tolerance for declaring a vector to "
        "lie in the kernel of H (epsilon) should be a "
        "small positive number in the range (0,1)");

  /// INITIALIZATION

  // The current estimate for the truncated-Newton update step s_k;
  // initialized to zero
  Vector s_k = 0 * g; // s_k and g must have the same dimensions

  // The residual at iteration k: r_k := H*s_k + g.  Note
  Vector r_k = g;

  // Preconditioned residual vector
  Vector v_k;

  // Lagrange multipliers for constraint preconditioning operator
  Multiplier lambda_k;

  // Update direction for s_k
  Vector p_k;

  // Working space to hold Hessian-vector products H*p_k
  Vector Hp_k;

  // Preconditioner return value
  if (!P) {
    // If no preconditioner was supplied then v_k === r_k
    v_k = r_k;
  } else {
    // Precondition the current residual vector r_k
    std::tie(v_k, lambda_k) = (*P)(r_k, args...);

    if (At) {
      // Following Section 6 of the paper "On the Solution of Equality-
      // Constrained Quadratic Programming Problems Arising in Optimization",
      // we extract the Lagrange multiplier estimates λ corresponding to the
      // solution of:
      //
      //   min |r - A'λ|_{M^{-1}}

      // and use these to subtract off the corresponding component A'λ (which,
      // by the Fundamental Theorem of Algebra, lies in the orthogonal
      // complement of ker(A)).  As described in the paper, the purpose of this
      // is to ensure that, when applying the preconditioning operator to
      // compute P(r) = (v, λ), the Lagrange multiplier estimate λ is small in
      // norm relative to v, so that the preconditioned search direction v can
      // be computed to high relative accuracy.
      r_k -= (*At)(lambda_k, args...);
    }
  } // preconditioning

  // Set initial search direction
  p_k = -v_k;

  // Value of inner product < s_k, M * p_k >; initially zero since s0 = 0
  Scalar sk_M_pk = 0;

  // Squared M-norm of current Newton step estimate s_k:  || s_k ||_M^2;
  // initially zero since s0 = 0
  Scalar sk_M_2 = 0;

  // Squared M-norm of current update step direction p_k: || p_k ||_M^2
  Scalar pk_M_2 = inner_product(r_k, v_k, args...);

  /// Useful cached variables

  // Squared radius of trust-region
  Scalar Delta_2 = Delta * Delta;

  // Magnitude of initial (preconditioned) residual in the norm specified by the
  // (optional) preconditioner P: r0_norm = sqrt(r0'*P*r0)
  Scalar r0_norm = sqrt(inner_product(r_k, v_k, args...));

  // Target norm of preconditioned residual after applying update step s_k
  Scalar target_rk_norm =
      r0_norm * std::min(kappa_fgr, std::pow(r0_norm, theta));

  Scalar alpha_k; // Steplength for full Newton step along search direction p_k
  Scalar beta_k;
  Scalar kappa_k; // Value of inner product < p_k, H*p_k >

  for (num_iterations = 0; num_iterations < max_iterations; ++num_iterations) {
    /// CHECK TERMINATION CRITERIA

    /// Termination criteria based upon predicted residual after applying the
    /// current update step s_k
    if (std::sqrt(inner_product(r_k, v_k, args...)) <= target_rk_norm)
      break;

    // Compute Hessian-vector product H*p_k for this iteration
    Hp_k = H(p_k, args...);

    /// Next, check termination criteria based upon check for negative curvature
    /// or overly-long steplengths

    // Compute kappa
    kappa_k = inner_product(p_k, Hp_k, args...);

    // Stopping condition based upon whether pk'Hpk = 0, up to numerical
    // tolerance(cf.Sec.7 of "On the Solution of Equality-Constrained
    // Quadratic Programming Problems Arising in Optimization ")
    if (sqrt(inner_product(Hp_k, Hp_k, args...)) /
            sqrt(inner_product(p_k, p_k, args...)) <
        epsilon) {

      // If control reaches this point in the function, the following two
      // conditions hold:
      //
      //  - The (preconditioned) residual v_k has M-norm greater than the
      //    target norm
      //
      //  - The current search direction lies in the kernel of the Hessian
      //    operator
      //
      //  In this case, we should follow the subspace spanned by p_k to the
      //  boundary of the trust-region radius
      if (inner_product(p_k, r_k, args...) < 0) {

        //  Multiply the current search direction by - 1 to ensure that it's a
        // direction of descent
        p_k *= -1;
        sk_M_pk *= -1;
      }

      // Compute sigma_k such that || s_k + sigma_k * p_k ||_M = Delta, i.e. so
      // that the next update terminates on the trust-region boundary
      Scalar sigma_k =
          (-sk_M_pk + sqrt(sk_M_pk * sk_M_pk + pk_M_2 * (Delta_2 - sk_M_2))) /
          pk_M_2;

      update_step_M_norm = Delta;

      s_k += sigma_k * p_k;
      return s_k;
    }

    // Compute (full) steplength alpha_k
    alpha_k = inner_product(r_k, v_k, args...) / kappa_k;

    // Compute norm of proposed (full) step
    Scalar skplus1_M_2 =
        sk_M_2 + 2 * alpha_k * sk_M_pk + alpha_k * alpha_k * pk_M_2;

    if ((kappa_k <= 0) || (skplus1_M_2 > Delta_2)) {
      /** Either p_k is a direction of negative curvature, or the full
       * (unrestricted) step along this direction leaves the trust-region.
       * In either case, we would like to rescale the stepsize alpha_k to
       * ensure that the proposed step terminates _on_ the trust-region
       * boundary, and then return the resulting step as our final answer
       */

      Scalar sigma_k =
          (-sk_M_pk + sqrt(sk_M_pk * sk_M_pk + pk_M_2 * (Delta_2 - sk_M_2))) /
          pk_M_2;

      update_step_M_norm = Delta;
      s_k += sigma_k * p_k;
      return s_k;
    }

    /// Call user-supplied function, if one was passed
    if (user_function && (*user_function)(num_iterations, g, H, P, At, s_k, r_k,
                                          v_k, p_k, alpha_k, args...)) {
      // User-defined stopping criterion just fired
      break;
    }

    /// UPDATE!  Compute values for next iteration

    // Update solution s_k -> s_kplus1
    s_k = s_k + alpha_k * p_k;

    // Update estimate for residual after applying s_k:  r_k -> r_kplus1
    r_k += alpha_k * Hp_k;

    // Compute preconditioned residual v_kplus1
    // Preconditioner return value
    if (!P) {
      // If no preconditioner was supplied then v_k === r_k
      v_k = r_k;
    } else {
      // Precondition the current residual vector r_k
      std::tie(v_k, lambda_k) = (*P)(r_k, args...);

      if (At) {
        // Following Section 6 of the paper "On the Solution of Equality-
        // Constrained Quadratic Programming Problems Arising in Optimization",
        // we extract the Lagrange multiplier estimates λ corresponding to the
        // solution of:
        //
        //   min |r - A'λ|_{M^{-1}}

        // and use these to subtract off the corresponding component A'λ (which,
        // by the Fundamental Theorem of Algebra, lies in the orthogonal
        // complement of ker(A)).  As described in the paper, the purpose of
        // this is to ensure that, when applying the preconditioning operator to
        // compute P(r) = (v, λ), the Lagrange multiplier estimate λ is small in
        // norm relative to v, so that the preconditioned search direction v can
        // be computed to high relative accuracy.
        r_k -= (*At)(lambda_k, args...);
      }
    } // preconditioning

    // Compute updated preconditioned inner product
    Scalar rk_vk = inner_product(r_k, v_k, args...);

    // Compute beta_k for *current* iterate (using *updated* values
    // r_kplus1 and v_kplus1 and *current* iterate value alpha_k, kappa_k)
    beta_k = rk_vk / (alpha_k * kappa_k);

    // Update inner products and norms
    sk_M_2 = skplus1_M_2;
    sk_M_pk = beta_k * (sk_M_pk + alpha_k * pk_M_2);
    pk_M_2 = rk_vk + beta_k * beta_k * pk_M_2;

    // Update search direction p_k
    p_k = -v_k + beta_k * p_k;

  } // for

  update_step_M_norm = sqrt(sk_M_2);
  return s_k;
}

/** An alias template for a user-definable function that can be
 * used to access various interesting bits of information about the internal
 * state of the LSQR algorithm as it runs.
 *
 * - k: Index of current iteration
 * - A:  Coefficient operator A
 * - At:  Transpose of coefficient operator A
 * - b:  Constant vector b
 * - xk: Estimated solution
 * - xk_norm:  Solution norm
 * - rbar_norm:  Norm of residual rbar(x) := Abar*x - bbar
 * - Abar_rbar_norm:  Norm of Abar'rbar
 * - Abar_norm_est:  Estimate for the norm of Abar
 * - Abar_cond_est:  Estimate for the condition number of Abar
 *
 * Note that this function is called at the *end* of each iteration (i.e.,
 * *after* each of the above quantities have been updated)
 *
 * This function may also return the Boolean value 'true' in order to
 * terminate the LSQR algorithm; this provides a convenient means of
 * implementing a custom (user-definable) stopping criterion.
 */
template <typename VectorX, typename VectorY, typename Scalar = double,
          typename... Args>
using LSQRUserFunction = std::function<bool(
    size_t k, const LinearOperator<VectorX, VectorY, Args...> &A,
    const LinearOperator<VectorY, VectorX, Args...> &At, const VectorY &b,
    const VectorX &xk, Scalar xk_norm, Scalar rbar_norm, Scalar Abar_rbar_norm,
    Scalar Abar_norm_est, Scalar Abar_cond_est, Args &... args)>;

/** This function implements (a slightly modified version of) the LSQR algorithm
 * for approximately solving linear least-squares problems of the form:
 *
 *     min_x  |Ax - b|^2 + lambda * |x|^2
 *
 *     s.t.   |x| <= Delta
 *
 * Note that this problem is equivalent to:
 *
 *     min_x  |Abar*x - bbar|^2
 *
 *     s.t.   |x| <= Delta
 *
 * with
 *
 *     Abar := [     A      ]
 *             [sqrt(lambda)]
 *
 *     bbar := [b]
 *             [0].
 *
 * This specific implementation is based upon the MATLAB implementation provided
 * by the Stanford Systems Optimization Laboratory, although modified to employ
 * alternative termination criteria (including a trust-region constraint). See
 * the references:
 *
 * - "LSQR: An algorithm for sparse linear equations and sparse least squares",
 *   by C.C. Paige and M.A. Saunders
 *
 * - "Algorithm 583.  LSQR: Sparse linear equations and least squares problems",
 *   by C.C. Paige and M.A. Saunders
 *
 * for details of the algorithm.
 *
 * Here:
 *
 * - A: X -> Y and At: Y -> X are linear operators corresponding to the
 *   coefficient matrix A and its transpose At
 *
 * - b is a constant vector
 *
 * - inner_product_x and inner_product_y are inner products defined on the
 *   domain and codomain of A
 *
 * - xnorm: upon termination, this is set to |x|, the norm of
 *   the returned solution
 *
 * - num_iterations: upon termination, this is set to the number of iterations
 *   executed by the algorithm
 *
 * - max_iterations: An optional argument specifying the maximum number of
 *   iterations to perform [default: 1000].
 *
 * - lambda: an optional Tikhonov regularization parameter for the least-squares
 *   problem [default: 0]
 *
 * - btol and Atol are relative error tolerances defining the stopping criteria
 *   for the algorithm: the method will terminate if *either* of the following
 *   criteria are satisfied:
 *
 *   S1:  |rbar| <= btol*|b| + Atol * |Abar| * |x|
 *
 *   S2:  |Abar'rbar| <= Atol * |Abar| * |rbar|
 *
 *   Condition S1 controls the relative reduction in the least-squares residual,
 *   and so is appropriate for use with *consistent* linear systems.  Condition
 *   S2 controls the relative magnitude of the least-squares objective's
 *   *gradient* g(x) := 2*Abar'rbar, and so is appropriate for use with
 *   *inconsistent* linear systems.
 *
 *   Default values are btol = 1e-6 and atol = 1e-6, corresponding to a
 *   relatively precise least-squares solution.
 *
 *   See Section 6 of the paper LSQR: An algorithm for sparse linear equations
 *   and sparse least squares" for more detail on the use of these criteria.
 *
 * - Abar_cond_limit: Similarly, this parameter imposes a termination tolerance
 *   on the estimated condition number of the coefficient matrix Abar: the
 *   algorithm will terminate if Abar_cond_est >= Abar_cond_limit.  This is
 *   intended to regularize ill-conditioned linear systems.  Again, see Section
 *   6 of the paper LSQR: An algorithm for sparse linear equations and sparse
 *   least squares" for more detail on the use of this criterion.
 *
 *   Default value: 1e8
 *
 * - Delta: An optional trust-region radius for the least-squares problem
 *   [default value: infinity].  This provides direct control over the norm of
 *   the returned solution, which is useful in the context of trust-region
 *   optimization algorithms
 *
 * - user_function is an optional user-defined function that can be used to
 *   inspect various variables of interest as the algorithm runs, and
 *   (optionally) to define a custom stopping criterion for the algorithm.
 */
template <typename VectorX, typename VectorY, typename Scalar = double,
          typename... Args>
VectorX
LSQR(const LinearOperator<VectorX, VectorY, Args...> &A,
     const LinearOperator<VectorY, VectorX, Args...> &At, const VectorY &b,
     const InnerProduct<VectorX, Scalar, Args...> &inner_product_x,
     const InnerProduct<VectorY, Scalar, Args...> &inner_product_y,
     Args &... args, Scalar &xnorm, size_t &num_iterations,
     size_t max_iterations = 1000, Scalar lambda = 0, Scalar btol = 1e-6,
     Scalar Atol = 1e-6, Scalar Abar_cond_limit = 1e8,
     Scalar Delta = sqrt(std::numeric_limits<Scalar>::max()),
     const std::optional<LSQRUserFunction<VectorX, VectorY, Scalar, Args...>>
         &user_function = std::nullopt) {

  /// Argument checking

  if (lambda < 0)
    throw std::invalid_argument("Tikhonov regularization parameter (lambda) "
                                "must be a nonnegative real value");

  if (btol < 0)
    throw std::invalid_argument(
        "Stopping tolerance btol must be a nonnegative real number");

  if (Atol < 0)
    throw std::invalid_argument(
        "Stopping tolerance Atol must be a nonnegative real number");

  if (Abar_cond_limit <= 0)
    throw std::invalid_argument(
        "Stopping tolerance Abar_cond_limit must be a positive real number");

  if (Delta <= 0)
    throw std::invalid_argument(
        "Trust-region radius (Delta) must be a positive real value");

  /// INITIALIZATION

  // Allocate output vector x
  VectorX x;

  // Norm of x
  xnorm = 0;

  // Total number of iterations
  num_iterations = 0;

  /// Norm and condition number estimates

  Scalar xxnorm = 0;

  // Estimate for the norm of Abar
  Scalar Abar_norm_est = 0;

  // Estimate for the condition number kappa(Abar) := |Abar|*|Abar^+|
  Scalar Abar_cond_est = 0;

  // Squared Frobenius norm of the matrix D := [d1, ..., dk] of search (update)
  // directions for x computed during each iteration of LSQR (cf. eq. (4.9) of
  // "LSQR - An Algorithm for Sparse Linear Equations and Sparse Least Squares")
  Scalar D_Fnorm2 = 0;

  // Norm of right-hand side vector b
  Scalar bnorm;

  /// Residual norms for least-squares system

  // Residual rbar(x) := Abar *x - bbar of the augmented least-squares system
  Scalar rbar_norm;

  // Norm of Abar'rbar.  Note that that grad L(x) = 2Abar'rbar(x), i.e.,
  // Abar'rbar is 1/2 of the gradient of the least-squares objective, so this
  // quantity serves as a useful measure of how close we are to a least-squares
  // solution
  Scalar Abar_rbar_norm = 0;

  // Square root of damping parameter
  Scalar sqrt_lambda = sqrt(lambda);

  /// Working vectors and scalars for bidiagonalization procedure

  // See eq. (3.1) of "LSQR - An Algorithm for Sparse Linear Equations and
  // Sparse Least Squares"
  Scalar alpha = 0;
  Scalar beta = 0;
  VectorX v, w;
  VectorY u;

  // Compute initial *direction vectors* for u and v
  u = b;
  v = At(u, args...);

  // Initialize x
  x = 0 * v;

  // Compute normalization constants for u and v
  alpha = sqrt(inner_product_x(v, v, args...));
  beta = sqrt(inner_product_y(u, u, args...));

  if (beta > 0) {
    // Normalize u
    u /= beta;
  }

  if (alpha > 0) {
    // Normalize v;
    v /= alpha;

    // Note that the current value of the scalar beta is a factor of alpha
    // larger than it should be, since we computed v using u, rather than the
    // *unit vector* uhat.
    alpha /= beta;

    // Cache w
    w = v;
  }

  // Check termination criteria
  Abar_rbar_norm = alpha * beta;
  if (Abar_rbar_norm == 0) {
    // This is already a least-squares solution, so return immediately
    return x;
  }

  // Record initial residual norms
  bnorm = beta;
  rbar_norm = beta;

  /// Scalars for plane rotation / bidiagonalization procedure

  // Elements of the currently-relevant blocks of the QR factorization we're
  // incrementally constructing; see equations (4.6) and (4.12) of the paper
  // ""LSQR - An Algorithm for Sparse Linear Equations and Sparse Least
  // Squares"
  Scalar rhobar = alpha;
  Scalar phibar = beta;

  Scalar cs2 = -1;
  Scalar sn2 = 0;
  Scalar z = 0;
  Scalar res2 = 0;

  /// MAIN LOOP

  for (num_iterations = 0; num_iterations < max_iterations; ++num_iterations) {

    /// Perform next step of bidiagonalization procedure to obtain the next
    /// values of beta, u, alpha, and v.  These should satisfy the relations:
    ///
    ///  beta * u = A * v - alpha * u
    ///  alpha * v = A' * u - beta * v
    ///
    /// See eqs.

    // Compute next u, beta
    u = A(v, args...) - alpha * u;
    beta = sqrt(inner_product_y(u, u, args...));

    if (beta > 0) {
      // Normalize u
      u /= beta;

      // Update estimate of norm of A
      Abar_norm_est = sqrt(Abar_norm_est * Abar_norm_est + alpha * alpha +
                           beta * beta + lambda);

      // Compute next v, alpha
      v = At(u, args...) - beta * v;
      alpha = sqrt(inner_product_x(v, v, args...));

      if (alpha > 0)
        v /= alpha;
    }

    /// Plane rotation to eliminate damping parameter.  This alters the diagonal
    /// element (rhobar) of the lower-bidiagonal matrix

    Scalar rhobar1 = sqrt(rhobar * rhobar + lambda);
    Scalar cs1 = rhobar / rhobar1;
    Scalar sn1 = sqrt_lambda / rhobar1;
    Scalar psi = sn1 * phibar;

    // Update phibar
    phibar *= cs1;

    /// Plane rotation to eliminate the subdiagonal element (beta) of the
    /// lower-bidiagonal matrix, producing an upper-bidiagonal matrix

    Scalar rho = sqrt(rhobar1 * rhobar1 + beta * beta);
    Scalar cs = rhobar1 / rho;
    Scalar sn = beta / rho;
    Scalar theta = sn * alpha;
    rhobar = -cs * alpha;
    Scalar phi = cs * phibar;
    phibar *= sn;
    Scalar tau = sn * phi;

    /// Use a plane rotation on the right to eliminate the super-diagonal
    /// element (theta) of the upper-bidiagonal matrix.  Then use the result to
    /// estimate the norm of x

    Scalar delta = sn2 * rho;
    Scalar gammabar = -cs2 * rho;
    Scalar rhs = phi - delta * z;
    Scalar zbar = rhs / gammabar;
    Scalar gamma = sqrt(gammabar * gammabar + theta * theta);
    cs2 = gammabar / gamma;
    sn2 = theta / gamma;
    z = rhs / gamma;

    /// UPDATE X AND W

    // Compute norms of wk and dk := (1/rho) * wk
    Scalar wk2 = inner_product_x(w, w, args...);
    Scalar dk2 = wk2 / (rho * rho);

    // Update update norm of x *AFTER* update is applied
    xnorm = sqrt(xxnorm + zbar * zbar);
    xxnorm += z * z;

    Scalar t2 = -theta / rho; // Update stepsize for w
    Scalar t1;                // Update stepsize for x

    // Check whether applying the *full* update would cause x to leave the
    // trust-region
    if (xnorm <= Delta) {
      // Use the full update step
      t1 = phi / rho;
    } else {
      // Applying the full update would cause x to leave the trust-region --
      // compute a shorter stepsize t1 such that x + t1 * w terminates on the
      // trust-region boundary instead

      Scalar xtx = inner_product_x(x, x, args...);
      Scalar wtx = inner_product_x(w, x, args...);

      // Compute steplength
      t1 = (-wtx + sqrt(wtx * wtx + wk2 * (Delta * Delta - xtx))) / wk2;

      // Update xnorm to reflect the fact that this step will terminate on the
      // boundary
      xnorm = Delta;
    }

    /// Update vectors!

    x += t1 * w;
    w = v + t2 * w;

    // Update squared Frobenius norm of D, using |dk|^2 = |wk|^2 / rho^2
    D_Fnorm2 += dk2;

    /// Update norm & conditioning estimates

    // Estimate condition number of Abar -- see eq. (5.10) of the paper "LSQR:
    // An algorithm for sparse linear equations and sparse least squares"
    Abar_cond_est = Abar_norm_est * sqrt(D_Fnorm2);

    // Compute norm of rbar -- see the end of Section 2 of "Algorithm 583. LSQR:
    // Sparse linear equations and least squares problems"
    Scalar res1 = phibar * phibar;
    res2 += psi * psi;
    rbar_norm = sqrt(res1 + res2);

    // Compute norm of Abar'rbar -- see the end of Section 2 of "Algorithm
    // 583. LSQR: Sparse linear equations and least squares problems"
    Abar_rbar_norm = alpha * fabs(tau);

    /// TEST STOPPING CONDITIONS
    // See Sec. 6 of the paper "LSQR: An algorithm for sparse linear equations
    // and sparse least squares"

    // Test 1: Check residual norm
    if (rbar_norm <= btol * bnorm + Atol * Abar_norm_est * xnorm)
      break;

    // Test 2: Check relative gradient norm
    if (Abar_rbar_norm <= Atol * Abar_norm_est * rbar_norm)
      break;

    // Test 3:  Check estimated condition number
    if (Abar_cond_est >= Abar_cond_limit)
      break;

    // Test 4:  Check trust-region radius
    if (xnorm >= Delta) {
      // The current estimate has reached the trust-region boundary, so
      // terminate
      break;
    }

    // Call user-supplied function, if one was passed
    /// Call user-supplied function, if one was passed
    if (user_function &&
        (*user_function)(num_iterations, A, At, b, x, xnorm, rbar_norm,
                         Abar_rbar_norm, Abar_norm_est, Abar_cond_est,
                         args...)) {
      // User-defined stopping criterion just fired
      break;
    }
  }

  return x;
}

/// Syntactic sugar -- presents a simplified interface for the case in which the
/// domain and codomain of the linear operator A have the same type
template <typename Vector, typename Scalar = double, typename... Args>
Vector
LSQR(const LinearOperator<Vector, Vector, Args...> &A,
     const LinearOperator<Vector, Vector, Args...> &At, const Vector &b,
     const InnerProduct<Vector, Scalar, Args...> &inner_product, Args &... args,
     Scalar &xnorm, size_t &num_iterations, size_t max_iterations = 1000,
     Scalar lambda = 0, Scalar btol = 1e-6, Scalar Atol = 1e-6,
     Scalar Abar_cond_limit = 1e8,
     Scalar Delta = sqrt(std::numeric_limits<Scalar>::max()),
     const std::optional<LSQRUserFunction<Vector, Vector, Scalar, Args...>>
         &user_function = std::nullopt) {

  return LSQR<Vector, Vector, Scalar, Args...>(
      A, At, b, inner_product, inner_product, args..., xnorm, num_iterations,
      max_iterations, lambda, btol, Atol, Abar_cond_limit, Delta,
      user_function);
}

} // namespace LinearAlgebra
} // namespace Optimization
