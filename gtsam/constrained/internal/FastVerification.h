/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FastVerification.h
 * @brief   LOBPCG + ILDL minimum-eigenpair routine used by
 *          RiemannianStaircaseOptimizer to check positive semidefiniteness
 *          of the dual certificate matrix S.
 *
 * Implementation uses David Rosen's Optimization and Preconditioners
 * libraries shipped in gtsam/3rdparty/.
 *
 * The body of this header is compiled only when
 * GTSAM_USE_LOBPCG_VERIFICATION is defined; otherwise the header is empty
 * and the LOBPCG verification path in RiemannianStaircaseOptimizer.cpp
 * is unreachable (it throws std::runtime_error).
 */

#pragma once

#ifdef GTSAM_USE_LOBPCG_VERIFICATION

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <Optimization/LinearAlgebra/LOBPCG.h>
#include <ILDL/ILDL.h>

#include <Eigen/CholmodSupport>
#include <Eigen/Sparse>

#include <cstddef>
#include <optional>
#include <tuple>

namespace gtsam {
namespace internal {

/**
 * @brief Test positive-semidefiniteness of `S + eta·I` via direct Cholesky,
 *        falling back to (preconditioned) LOBPCG to find a direction of
 *        sufficiently negative curvature if the PSD test fails.
 *
 * Returns true iff `S + eta·I` is PSD (no negative-curvature direction
 * found). When the function returns false, `x` is the offending eigenvector
 * and `theta = x^T S x` its Rayleigh quotient.
 */
inline bool fast_verification(const Eigen::SparseMatrix<double>& S, double eta,
                              std::size_t nx, double& theta, Vector& x,
                              std::size_t& num_iters, std::size_t max_iters,
                              double max_fill_factor, double drop_tol) {
  using SparseMat = Eigen::SparseMatrix<double>;

  num_iters = 0;
  theta = 0;
  const unsigned int n = static_cast<unsigned int>(S.rows());

  // STEP 1: PSD test on the regularized certificate M := S + eta·I via direct
  // sparse Cholesky.
  SparseMat Id(n, n);
  Id.setIdentity();
  const SparseMat M = S + eta * Id;

  Eigen::CholmodSupernodalLLT<SparseMat> MChol;
  MChol.cholmod().quick_return_if_not_posdef = 1;
  MChol.cholmod().print = 0;
  MChol.compute(M);
  if (MChol.info() == Eigen::Success) return true;

  // STEP 2: unpreconditioned LOBPCG (cheap path).
  Vector Theta;
  Matrix X;
  std::size_t num_converged = 0;

  Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix> Mop =
      [&M](const Matrix& V) -> Matrix { return M * V; };

  Optimization::LinearAlgebra::LOBPCGUserFunction<Vector, Matrix> stopfun =
      [&S, eta](
          std::size_t /*i*/,
          const Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>& /*M*/,
          const std::optional<
              Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>>& /*B*/,
          const std::optional<
              Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>>& /*T*/,
          std::size_t /*nev*/, const Vector& /*Theta*/, const Matrix& X,
          const Vector& /*r*/, std::size_t /*nc*/) {
        const double curv = X.col(0).dot(S * X.col(0));
        return curv < -eta / 2;
      };

  constexpr double unprecon_iter_frac = 0.15;
  std::tie(Theta, X) = Optimization::LinearAlgebra::LOBPCG<Vector, Matrix>(
      Mop,
      std::optional<Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>>(),
      std::optional<Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>>(),
      n, nx, 1,
      static_cast<std::size_t>(unprecon_iter_frac * max_iters), num_iters,
      num_converged, 0.0,
      std::optional<
          Optimization::LinearAlgebra::LOBPCGUserFunction<Vector, Matrix>>(
          stopfun));

  x = X.col(0);
  theta = x.dot(S * x);
  if (theta < -eta / 2) return false;

  // STEP 3: preconditioned LOBPCG for the near-zero negative case. Build an
  // incomplete symmetric indefinite factorization and use it as preconditioner.
  Preconditioners::ILDLOpts ildl_opts;
  ildl_opts.max_fill_factor = max_fill_factor;
  ildl_opts.drop_tol = drop_tol;
  Preconditioners::ILDL Mfact(M, ildl_opts);

  Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix> T =
      [&Mfact](const Matrix& V) -> Matrix {
        Matrix TV(V.rows(), V.cols());
        for (unsigned int i = 0; i < V.cols(); ++i) {
          TV.col(i) = Mfact.solve(V.col(i), true);
        }
        return TV;
      };

  std::size_t unprecon_iters = num_iters;
  std::tie(Theta, X) = Optimization::LinearAlgebra::LOBPCG<Vector, Matrix>(
      Mop,
      std::optional<Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>>(),
      std::optional<Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>>(T),
      n, nx, 1,
      static_cast<std::size_t>((1.0 - unprecon_iter_frac) * max_iters),
      num_iters, num_converged, 0.0,
      std::optional<
          Optimization::LinearAlgebra::LOBPCGUserFunction<Vector, Matrix>>(
          stopfun));

  x = X.col(0);
  theta = x.dot(S * x);
  num_iters += unprecon_iters;
  return false;
}

}  // namespace internal
}  // namespace gtsam

#endif  // GTSAM_USE_LOBPCG_VERIFICATION
