/** This header file provides an implementation of the locally optimal block
 * preconditioned conjugate gradient (LOBPCG) method for computing a few
 * algebraically-smallest eigenpairs (lambda, x) of the symmetric generalized
 * eigenproblem:
 *
 * Ax = lambda * Bx
 *
 * Here A and B are assumed to be symmetric linear operators, with B
 * positive-definite.
 *
 * Our implementation follows the paper:
 *
 * "A Robust and Efficient Implementation of LOBPCG", by J.A. Duersch, M. Shao,
 * C. Yang, and M. Gu
 *
 * Note that in *THIS SPECIFIC FILE*, it is assumed that the template parameters
 * 'Matrix' and 'Vector' correspond to dense Matrix and Vector types in the
 * Eigen library.
 *
 * Copyright (C) 2022 by David M. Rosen (d.rosen@northeastern.edu)
 */

#pragma once

#include <limits>
#include <optional>
#include <random>
#include <tuple>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <iostream>

#include "Optimization/LinearAlgebra/Concepts.h"

namespace Optimization {

namespace LinearAlgebra {

/** This function implements the basic Rayleigh-Ritz procedure: Given two
 * symmetric n x n matrices A and B, with B positive-definite, this function
 * computes and returns a pair of n x n matrices [Theta, C] satisfying
 *
 * C'AC = Theta
 * C'BC = I_n
 *
 * In a basic LOBPCG implementation, A would be replaced by S'AS and B would be
 * replaced by S'BS, where S is a basis matrix for the search space.
 */
template <typename Vector, typename Matrix>
std::pair<Vector, Matrix> RayleighRitz(const Matrix &A, const Matrix &B) {
  // Compute diagonal scaling matrix to equilibrate B
  Vector D = B.diagonal().cwiseSqrt().cwiseInverse();

  Eigen::GeneralizedSelfAdjointEigenSolver<Matrix> eig(
      D.asDiagonal() * A * D.asDiagonal(), D.asDiagonal() * B * D.asDiagonal());

  return std::make_pair(eig.eigenvalues(), D.asDiagonal() * eig.eigenvectors());
}

/** An alias template for a user-definable function that can be used to
 * access various interesting bits of information about the internal state
 * of the LOBPCG algorithm as it runs.  Here:
 * - i: Index of current iteration
 * - A: Symmetric linear operator for generalized eigenvalue problem
 * - B: Optional symmetric positive-definite linear operator for generalized
 *   eigenvalue problem
 * - T: Optional symmetric positive-definite preconditioning operator
 * - Theta: the current vector of eigenvalue estimates (Ritz values)
 * - nev:  The number of desired eigenpairs
 * - X: the current set of eigenvector estimates
 * - r: the current vector of residual norms
 * - nc: the number of converged eigenpairs
 *
 * This function is called once per iteration, after the Rayleigh-Ritz
 * procedure is used to update the current Ritz pairs and their
 * corresponding residuals
 *
 * This function may also return the Boolean value 'true' in order to
 * terminate the LOBPCG algorithm; this provides a convenient means of
 * implementing a custom (user-definable) stopping criterion.
 */
template <typename Vector, typename Matrix, typename Scalar = double,
          typename... Args>
using LOBPCGUserFunction = std::function<bool(
    size_t i, const SymmetricLinearOperator<Matrix, Args...> &A,
    const std::optional<SymmetricLinearOperator<Matrix, Args...>> &B,
    const std::optional<SymmetricLinearOperator<Matrix, Args...>> &T,
    size_t nev, const Vector &Theta, const Matrix &X, const Vector &residuals,
    size_t nc, Args &...args)>;

/** This function estimates the smallest eigenpairs (lambda, x) of the
 * generalized symmetric eigenvalue problem:
 *
 * Ax = lambda * Bx
 *
 * using Knyazev's locally optimal block preconditioned conjugate gradient
 * (LOBPCG) method.  Here:
 *
 * - A is a symmetric linear operator
 * - B is an optional positive-definite symmetric operator.  The absence of
 *   this parameter indicates that B == I (i.e. that we are solving a standard
 *   symmetric eigenproblem)
 * - T is an optional positive-definite symmetric preconditioner that
 *   approximates A^-1 in the sense that the condition number kappa(TA) should
 *   be (much) smaller than kappa(A) itself.  The absence of this parameter
 *   indicates that T == I (i.e. that no preconditioning is applied)
 * - X0 is an m x nx matrix containing an initial set of linearly-independent
 *   eigenvector estimates.
 * - nev is the number of desired eigenpairs; note that this number should be
 *   less than the block size nx.
 * - max_iters is the maximum number of iterations to perform
 * - num_iters is a return value that gives the number of iterations
 *   executed by the algorithm
 * - nc is a return value that indicates the number of eigenpairs
 *   that have converged to the requested precision
 * - tau is the stopping tolerance for the convergence test described in
 *   Section 4.3 of the paper "A Robust and Efficient Implementation of
 *   LOBPCG"; specifically, an eigenpair (lambda, x) is considered converged
 *   when:
 *
 *   ||Ax - lambda * Bx|| / (||A||_2 + lambda * ||B_2|| * ||x||) <= tau
 *
 * - user_function is an optional user-defined function that can be used to
 *   inspect various variables of interest as the algorithm runs, and
 *   (optionally) to define a custom stopping criterion for the algorithm.
 */
template <typename Vector, typename Matrix, typename Scalar = double,
          typename... Args>
std::pair<Vector, Matrix>
LOBPCG(const SymmetricLinearOperator<Matrix, Args...> &A,
       const std::optional<SymmetricLinearOperator<Matrix, Args...>> &B,
       const std::optional<SymmetricLinearOperator<Matrix, Args...>> &T,
       const Matrix &X0, size_t nev, size_t max_iters, size_t &num_iters,
       size_t &nc, Args &...args, Scalar tau = 1e-6,
       const std::optional<LOBPCGUserFunction<Vector, Matrix, Scalar, Args...>>
           &user_function = std::nullopt) {

  size_t m = X0.rows();  // Dimension of problem
  size_t nx = X0.cols(); // Block size

  // Column dimension of current search space basis matrix S
  size_t ns;

  /// Input sanitation

  if (nev > nx)
    throw std::invalid_argument(
        "Block size nx must be greater than or equal to "
        "the number nev of desired eigenpairs");

  if (nx > m)
    throw std::invalid_argument("Block size nx must be less than or equal to "
                                "the dimension m of the problem");

  /// Preallocate memory

  // Get some useful references to elements of the result struct
  Matrix X = X0; // Matrix of eigenvector estimates

  // Change of basis matrix C used to B-orthonormalize subspace basis
  // S and diagonalize S'AS.  This matrix satisfies:
  // - C'(S'BS)C = I
  // - C'(S'AS)C = Theta
  // where Theta is the corresponding set of Ritz values
  Matrix C;
  Vector Theta;

  // Cache variables for various matrix products
  Matrix AX, BX;

  // Matrix of residuals A*X - B*Theta
  Matrix R;

  // Matrix of preconditioned search directions
  Matrix W;

  // Matrix of implicit differences P
  Matrix P;

  // Search space basis matrix S = [X, W, P]
  Matrix S(m, 3 * nx); // We preallocate the maximum needed size

  // Cache variables for products AS and BS
  // NB:  We initialize these to their maximum possible size
  Matrix AS;
  Matrix BS;

  // Gram matrices S'AS, B'AS
  Matrix StAS;
  Matrix StBS;

  // Vector of residual norms: ri = ||Ri||, the norm of the ith column of R
  Vector r;

  /// Estimate 2-norms of A and B using a random matrix with standard Gaussian
  /// entries, as described in Section 4.3 of the paper "A Robust and Efficient
  /// Implementation of LOBPCG".  These quantities are used in the
  /// scale-invariant backward-stable termination criterion.

  // Sample a Gaussian
  std::default_random_engine gen;
  std::normal_distribution<Scalar> normal(0, 1.0);
  Matrix Omega(m, nx);

  for (size_t i = 0; i < m; ++i)
    for (size_t j = 0; j < nx; ++j)
      Omega(i, j) = normal(gen);

  Scalar A2normest = A(Omega).norm() / Omega.norm();
  Scalar B2normest = B ? (*B)(Omega).norm() / Omega.norm() : 1.0;

  /// INITIALIZATION

  AX = A(X);
  BX = B ? (*B)(X) : X;

  // B-orthonormalize columns of X using standard Rayleigh Ritz procedure
  std::tie(Theta, C) =
      RayleighRitz<Vector, Matrix>(X.transpose() * AX, X.transpose() * BX);

  // In-place update AX and BX
  AX = AX * C;
  BX = BX * C;

  // Compute residuals
  R = AX - BX * Theta.asDiagonal();

  // Initialize number of converged eigenpairs
  nc = 0;

  /// MAIN LOOP

  for (num_iters = 1; num_iters < max_iters; ++num_iters) {

    /// LOOP INVARIANTS: At the start of each iteration, the following hold:
    ///
    /// - X, AX, BX, R, P, and Theta are initialized to their values for the
    ///   current iteration

    /// COMPUTE ORTHONORMALIZED BASIS OF PRECONDITIONED SEARCH DIRECTIONS W

    // Compute preconditioned residuals (search directions) W
    W = T ? (*T)(R) : R;

    /// Set search space basis matrix S

    // Note that here we employ *SOFT LOCKING*:  We include in the search space
    // S = [X, W, P] only those columns of P and W corresponding to eigenpairs
    // that have NOT converged
    S.leftCols(nx) = X;
    S.middleCols(nx, nx - nc) = W.rightCols(nx - nc);

    if (num_iters > 1) {
      // Construct P matrix
      S.middleCols(2 * nx - nc, nx - nc) = P.rightCols(nx - nc);
      ns = 3 * nx - 2 * nc;
    } else {
      // Initial iteration: P == 0
      ns = 2 * nx - nc;
    }

    // Update AS, BS
    AS = A(S.leftCols(ns));
    BS = B ? (*B)(S.leftCols(ns)) : S.leftCols(ns);

    /// Update Gram matrices
    StAS = S.leftCols(ns).transpose() * AS;
    StBS = S.leftCols(ns).transpose() * BS;

    /// Rayleigh-Ritz procedure
    std::tie(Theta, C) = RayleighRitz<Vector, Matrix>(StAS, StBS);

    // Update eigenvector estimates
    X = S.leftCols(ns) * C.leftCols(nx);

    // Update AX and BX
    AX = A(X);
    BX = B ? (*B)(X) : X;

    // Update residuals R
    R = AX - BX * Theta.head(nx).asDiagonal();

    // Update matrix P
    P = S.middleCols(nx, ns - nx) * C.bottomLeftCorner(ns - nx, nx);

    /// TEST STOPPING CRITERIA

    // Calculate residual norms
    r = R.colwise().norm();

    // Here we apply the convergence test described in Section 4.3 of
    // the paper "A Robust and Efficient Implementation of LOBPCG"

    Vector tolerances =
        tau *
        (A2normest +
         B2normest * Theta.head(nx).cwiseAbs().transpose().array()) *
        X.colwise().norm().array();

    // Calculate vector of Boolean indicators for the convergence of
    // each eigenpair
    const auto converged =
        (r.head(nev).array() <= tolerances.head(nev).array());

    // Find the largest number k such that converged[i] = true for
    // all i <= k; this gives the number of converged eigenvectors.
    // Note that this may be less than the *TOTAL* number of
    // eigenpairs for which converged = true; this difference
    // accounts for the fact that eigenpairs must be soft-locked IN
    // ORDER: that is, we may apply soft locking only to the first
    // *CONTINGUOUS BLOCK* of converged eigenpairs
    for (nc = 0; nc < nev; ++nc)
      if (!converged[nc])
        break;

    // Call user-supplied function (if one was provided), and test
    // for user-defined stopping criterion
    if (user_function && (*user_function)(num_iters, A, B, T, nev,
                                          Theta.head(nx), X, r, nc, args...))
      break;

    // Test whether the requested number of eigenpairs have converged
    if (nc == nev)
      break;

  } // for(num_iters ... )

  /// Finalize solution
  Theta.conservativeResize(nev);
  X.conservativeResize(Eigen::NoChange, nev);

  return std::make_pair(Theta, X);
}

/** This function estimates the smallest eigenpairs (lambda, x) of the
 * generalized symmetric eigenvalue problem:
 *
 * Ax = lambda * Bx
 *
 * using Knyazev's locally optimal block preconditioned conjugate gradient
 * (LOBPCG) method.  Here:
 *
 * - A is a symmetric linear operator
 * - B is an optional positive-definite symmetric operator.  The absence of
 * this parameter indicates that B == I (i.e. that we are solving a standard
 *   symmetric eigenproblem)
 * - T is an optional positive-definite symmetric preconditioner that
 *   approximates A^-1 in the sense that the condition number kappa(TA) should
 *   be (much) smaller than kappa(A) itself.  The absence of this parameter
 *   indicates that T == I (i.e. that no preconditioning is applied)
 * - m is the dimension of the eigenvalue problem (order of A, B, T)
 * - nx is the block size
 * - nev is the number of desired eigenpairs; note that this number should be
 *   less than the block size m.
 * - max_iters is the maximum number of iterations to perform
 * - num_iters is a return value that gives the number of iterations
 *   executed by the algorithm
 * - num_converged is a return value that indicates the number of eigenpairs
 *   that have converged to the requested precision
 * - tau is the stopping tolerance for the convergence test described in
 *   Section 4.3 of the paper "A Robust and Efficient Implementation of
 *   LOBPCG"; specifically, an eigenpair (lambda, x) is considered converged
 *   when:
 *
 *   ||Ax - lambda * Bx|| / (||A||_2 + lambda * ||B_2|| * ||x||) <= tau
 *
 * - user_function is an optional user-defined function that can be used to
 *   inspect various variables of interest as the algorithm runs, and
 *   (optionally) to define a custom stopping criterion for the algorithm.
 */

template <typename Vector, typename Matrix, typename Scalar = double,
          typename... Args>
std::pair<Vector, Matrix>
LOBPCG(const SymmetricLinearOperator<Matrix, Args...> &A,
       const std::optional<SymmetricLinearOperator<Matrix, Args...>> &B,
       const std::optional<SymmetricLinearOperator<Matrix, Args...>> &T,
       size_t m, size_t nx, size_t nev, size_t max_iters, size_t &num_iters,
       size_t &nc, Args &...args, Scalar tau = 1e-6,
       const std::optional<LOBPCGUserFunction<Vector, Matrix, Scalar, Args...>>
           &user_function = std::nullopt) {
  Matrix X0 = Matrix::Random(m, nx);

  return LOBPCG<Vector, Matrix, Scalar, Args...>(
      A, B, T, X0, nev, max_iters, num_iters, nc, args..., tau, user_function);
}

} // namespace LinearAlgebra
} // namespace Optimization
