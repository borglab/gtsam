/** This unit test exercises the functionality of the iterative linear-algebraic
 * methods in the IterativeSolvers file */

#include "Optimization/LinearAlgebra/IterativeSolvers.h"

#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Sparse>
#include <Eigen/UmfPackSupport>

#include <gtest/gtest.h>

// Typedef for the numerical type will use in the following tests
typedef double Scalar;

// Typedefs for Eigen matrix types
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
typedef Eigen::SparseMatrix<Scalar, Eigen::ColMajor> SparseMatrix;
typedef Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic> DiagonalMatrix;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

/// Test tolerances
Scalar eps_abs = 1e-6;
Scalar eps_rel = 1e-6;

/// Dimensions of STPCG test problems
constexpr size_t STPCG_small_dim = 3;
constexpr size_t STPCG_large_dim = 1000;
constexpr size_t STPCG_num_constraints = 100;

/// Test fixture for testing the Steihaug-Toint preconditioned conjugate
/// gradient solver
class STPCGUnitTest : public testing::Test {
protected:
  /// Set up variables and functions used in the following tests

  // Inner product between Vectors
  Optimization::LinearAlgebra::InnerProduct<Vector> inner_product;

  // Input vector  g -- right-hand side of the equation Hv = -g we're trying to
  // solve in each test instance
  Vector small_g;
  Vector large_g;

  // Positive-definite Hessian matrices
  DiagonalMatrix small_P;
  DiagonalMatrix large_P;

  // Negative-definite Hessian matrices
  DiagonalMatrix small_N;

  // Preconditioning matrices
  DiagonalMatrix small_M;
  DiagonalMatrix large_M;

  // Hessian operators corresponding to P and N
  Optimization::LinearAlgebra::SymmetricLinearOperator<Vector> small_Pop;
  Optimization::LinearAlgebra::SymmetricLinearOperator<Vector> large_Pop;
  Optimization::LinearAlgebra::SymmetricLinearOperator<Vector> small_Nop;

  // Preconditioning operators
  Optimization::LinearAlgebra::STPCGPreconditioner<Vector, Vector> small_MinvOp;
  Optimization::LinearAlgebra::STPCGPreconditioner<Vector, Vector> large_MinvOp;

  /// method

  // Trust region radius
  Scalar Delta;

  // Truncation stopping criteria
  Scalar kappa;
  Scalar theta;
  size_t max_iterations;

  // Additional returns from STPCG
  Scalar update_step_norm;
  size_t num_iterations;

  virtual void SetUp() {

    // Inner product
    inner_product = [](const Vector &X, const Vector &Y) { return X.dot(Y); };

    // Right-hand side g
    small_g.resize(STPCG_small_dim);
    small_g << 21, -.4, 19;
    large_g.resize(STPCG_large_dim);
    large_g.setRandom();

    /// Termination criteria for the Steihaug-Toint truncated conjugate gradient
    /// solver

    /// Positive-definite Hessian matrices

    // Positive-definite matrix P
    // Manually-constructed small positive-definite matrix
    small_P.resize(STPCG_small_dim);
    small_P.diagonal() << 1000, 100, 1;
    small_Pop = [&](const Vector &X) { return small_P * X; };

    // Sample a large PD Hessian (note that here we are using the fact that
    // Eigen's Random operator returns values in the range [-1, 1]
    large_P = (2000 * Vector::Ones(STPCG_large_dim) +
               1000 * Vector::Random(STPCG_large_dim))
                  .asDiagonal();

    large_Pop = [&](const Vector &X) { return large_P * X; };

    /// Negative-definite Hessian matrices

    small_N = -1 * small_P;
    small_Nop = [&](const Vector &X) { return small_N * X; };

    /// Preconditioning operators

    // Manually-constructed small positive-definite preconditioner
    small_M = DiagonalMatrix(STPCG_small_dim);
    small_M.diagonal() << 100, 10, 1;
    small_MinvOp = [&](const Vector &v) -> std::pair<Vector, Vector> {
      return std::make_pair(small_M.inverse() * v, Vector());
    };

    // Randomly-sampled large positive-definite preconditioner
    large_M = (2000 * Vector::Ones(STPCG_large_dim) +
               1000 * Vector::Random(STPCG_large_dim))
                  .asDiagonal();

    large_MinvOp = [&](const Vector &v) -> std::pair<Vector, Vector> {
      return std::make_pair(large_M.inverse() * v, Vector());
    };
  }
};

/// Test STPCG with a small positive-definite Hessian, extremely tight
/// (effectively exact) stopping tolerance, and a numerically-infinite
/// trust-region radius, to ensure that the underlying CG method is implemented
/// properly
TEST_F(STPCGUnitTest, ExactSTPCG) {

  kappa = 1e-8;
  theta = .999;
  Delta = std::numeric_limits<Scalar>::max();

  Vector s = Optimization::LinearAlgebra::STPCG<Vector, Vector>(
      small_g, small_Pop, inner_product, update_step_norm, num_iterations,
      Delta, STPCG_small_dim, kappa, theta);

  // Compute ground-truth solution
  Vector s_gt = -(small_P.inverse() * small_g);

  // Compute error in returned step s
  Scalar error = (s - s_gt).norm();

  // Check the solution
  EXPECT_LT(error, eps_abs);

  // Check that the reported update step norm is correct
  EXPECT_LT(fabs(update_step_norm - s.norm()) / s.norm(), eps_rel);
}

/// Test STPCG with a small negative-definite Hessian, extremely tight
/// (effectively exact) stopping tolerance, and a numerically-infinite
/// trust-region radius, to ensure that the step to the boundary of the
/// trust-region is computed properly
TEST_F(STPCGUnitTest, ExactSTPCGwithNegativeCurvature) {

  kappa = 1e-8;
  theta = .999;
  Delta = 1000;

  Vector s = Optimization::LinearAlgebra::STPCG<Vector, Vector>(
      small_g, small_Nop, inner_product, update_step_norm, num_iterations,
      Delta, STPCG_small_dim, kappa, theta);

  // Compute ground-truth solution
  Vector s_gt = -((Delta / small_g.norm()) * small_g);

  // Compute error in returned step s
  Scalar error = (s - s_gt).norm();

  // Check the solution
  EXPECT_LT(error, eps_abs);

  // Check that the reported update step norm is correct
  EXPECT_LT(fabs(update_step_norm - s.norm()) / s.norm(), eps_rel);
}

/// Test STPCG using a positive-definite Hessian and a (nontrivial)
/// positive-definite preconditioning operator
TEST_F(STPCGUnitTest, ExactSTPCGwithPreconditioning) {

  kappa = 1e-8;
  theta = .999;
  Delta = std::numeric_limits<Scalar>::max();

  std::optional<
      Optimization::LinearAlgebra::STPCGPreconditioner<Vector, Vector>>
      precon_op(small_MinvOp);

  Vector s = Optimization::LinearAlgebra::STPCG<Vector, Vector>(
      small_g, small_Pop, inner_product, update_step_norm, num_iterations,
      Delta, STPCG_small_dim, kappa, theta, precon_op);

  // Compute ground-truth solution
  Vector s_gt = -(small_P.inverse() * small_g);

  // Compute error in returned step s
  Scalar error = (s - s_gt).norm();

  // Check the solution
  EXPECT_LT(error, eps_abs);

  // Check that the reported update step norm is correct
  Scalar s_M_norm = sqrt(s.dot(small_M * s));
  EXPECT_LT(fabs((update_step_norm - s_M_norm) / s_M_norm), eps_rel);
}

/// Test STPCG using a negative-definite Hessian and a (nontrivial)
/// positive-definite preconditioning operator
TEST_F(STPCGUnitTest, ExactSTPCGwithNegativeCurvatureAndPreconditioning) {

  kappa = 1e-8;
  theta = .999;
  Delta = 1000;

  std::optional<
      Optimization::LinearAlgebra::STPCGPreconditioner<Vector, Vector>>
      precon_op(small_MinvOp);

  Vector s = Optimization::LinearAlgebra::STPCG<Vector, Vector>(
      small_g, small_Nop, inner_product, update_step_norm, num_iterations,
      Delta, STPCG_small_dim, kappa, theta, precon_op);

  /// Compute ground-truth solution
  // Compute initial search direction
  Vector p = -(small_M.inverse() * small_g);
  // Compute M-norm of initial search direction
  Scalar p_M_norm = sqrt(p.dot(small_M * p));
  // Scale this search direction so that |alpha*p|_M = Delta
  Vector s_gt = (Delta / p_M_norm) * p;

  // Compute error in returned step s
  Scalar error = (s - s_gt).norm();

  // Check the solution
  EXPECT_LT(error, eps_abs);

  // Check that the reported update step norm is correct
  Scalar s_M_norm = sqrt(s.dot(small_M * s));
  EXPECT_LT(fabs(update_step_norm - s_M_norm) / s_M_norm, eps_rel);
}

/// Test a large-scale instance of the problem using truncation
TEST_F(STPCGUnitTest, STPCGwithTruncation) {

  kappa = .1;
  theta = .7;
  Delta = 1000;

  Vector s = Optimization::LinearAlgebra::STPCG<Vector, Vector>(
      large_g, large_Pop, inner_product, update_step_norm, num_iterations,
      Delta, STPCG_small_dim, kappa, theta);

  // Compute norm of residual
  Scalar r_norm = (large_g + large_P * s).norm();

  // Compute ratio of the final residual to the initial residual
  Scalar relative_residual = r_norm / large_g.norm();

  // Check that the final residual achieves the targeted relative reduction
  EXPECT_LT(relative_residual, kappa);

  // Check that the reported update step norm is correct
  EXPECT_LT(fabs(update_step_norm - s.norm()) / s.norm(), eps_rel);
}

/// Test a large-scale instance of the problem using preconditioning and
/// truncation
TEST_F(STPCGUnitTest, STPCGwithPreconditioningAndTruncation) {

  kappa = .1;
  theta = .7;
  Delta = 1000;

  std::optional<
      Optimization::LinearAlgebra::STPCGPreconditioner<Vector, Vector>>
      precon_op(large_MinvOp);

  Vector s = Optimization::LinearAlgebra::STPCG<Vector, Vector>(
      large_g, large_Pop, inner_product, update_step_norm, num_iterations,
      Delta, STPCG_large_dim, kappa, theta, precon_op);

  // Compute the Minv-norm of the intial residual large_g
  Scalar gMinv_norm = sqrt(large_g.dot(large_M.inverse() * large_g));

  // Compute Minv-norm of the final residual r := g + Hs
  Vector r = large_g + large_P * s;
  Scalar rMinv_norm = sqrt(r.dot(large_M.inverse() * r));

  // Compute ratio of the final residual to the initial residual
  Scalar relative_residual = rMinv_norm / gMinv_norm;

  // Check that the final residual achieves the targeted relative reduction
  EXPECT_LT(relative_residual, kappa);

  // Check that the reported update step norm is correct
  // Compute M-norm of returned update step
  Scalar s_M_norm = sqrt(s.dot(large_M * s));
  EXPECT_LT(fabs((update_step_norm - s_M_norm) / s_M_norm), eps_rel);
}

/// Test STPCG on a large problem with an equality constraint, using constraint
/// preconditiong, and with an extremely tight (effectively exact) stopping
/// tolerance and a numerically-infinite trust-region radius, in order to ensure
/// that the underlying method is implemented properly
TEST_F(STPCGUnitTest, ExactProjectedSTPCG) {

  kappa = 1e-8;
  theta = .7;
  Delta = std::numeric_limits<Scalar>::max();

  // Sample a random constraint operator A
  Matrix A = 1000 * Matrix::Random(STPCG_num_constraints, STPCG_large_dim);

  /// Compute the solution of this (strictly convex) quadratic program by
  /// solving the KKT system directly:
  ///
  /// [H A'][s] = [-g]
  /// [A 0 ][l] = [0]

  // Somewhat hacky but simple: construct KKT system matrix as a dense matrix by
  // assigning to its blocks
  Matrix Kdense = Matrix::Zero(STPCG_large_dim + STPCG_num_constraints,
                               STPCG_large_dim + STPCG_num_constraints);
  Kdense.topLeftCorner(STPCG_large_dim, STPCG_large_dim) = large_P;
  Kdense.topRightCorner(STPCG_large_dim, STPCG_num_constraints) = A.transpose();
  Kdense.bottomLeftCorner(STPCG_num_constraints, STPCG_large_dim) = A;

  // ... and then sparsify
  SparseMatrix K = Kdense.sparseView();
  K.makeCompressed();

  // Construct right-hand side vector for KKT system
  Vector rhs = Vector::Zero(STPCG_large_dim + STPCG_num_constraints);
  rhs.head(STPCG_large_dim) = -large_g;

  // Solve KKT system using LU factorization
  Eigen::UmfPackLU<SparseMatrix> Kfact(K);
  Vector z = Kfact.solve(rhs);

  Vector s_gt = z.head(STPCG_large_dim);
  Vector lambda_star = z.tail(STPCG_num_constraints);

  // Verify solution by checking the KKT conditions
  EXPECT_LT((large_g + large_P * s_gt + A.transpose() * lambda_star).norm(),
            eps_abs);
  EXPECT_LT((A * s_gt).norm(), eps_abs);

  /// Construct constraint preconditioning operator Mc, satisfying:
  ///
  /// Mc(s) = (x, l), where (x,l) is the solution of:
  ///
  /// [M A'][x] = [s]
  /// [A 0 ][l] = [0]

  Matrix Mcdense(STPCG_large_dim + STPCG_num_constraints,
                 STPCG_large_dim + STPCG_num_constraints);
  Mcdense.topLeftCorner(STPCG_large_dim, STPCG_large_dim) = large_M;
  Mcdense.topRightCorner(STPCG_large_dim, STPCG_num_constraints) =
      A.transpose();
  Mcdense.bottomLeftCorner(STPCG_num_constraints, STPCG_large_dim) = A;

  SparseMatrix Mc = Mcdense.sparseView();
  Mc.makeCompressed();

  // Construct and cache factorization of Pc
  Eigen::UmfPackLU<SparseMatrix> Mcfact(Mc);
  Vector w = Vector::Zero(STPCG_large_dim + STPCG_num_constraints);

  // Construct preconditioning operator
  Optimization::LinearAlgebra::STPCGPreconditioner<Vector, Vector> McOp =
      [&Mcfact, &w](const Vector &r) -> std::pair<Vector, Vector> {
    // Construct and solve constraint preconditioning system
    w.head(STPCG_large_dim) = r;
    Vector z = Mcfact.solve(w);

    return std::make_pair(z.head(STPCG_large_dim),
                          z.tail(STPCG_num_constraints));
  };

  std::optional<
      Optimization::LinearAlgebra::STPCGPreconditioner<Vector, Vector>>
      precon_op(McOp);

  /// Construct transpose constraint operator
  Matrix At = A.transpose();
  Optimization::LinearAlgebra::LinearOperator<Vector, Vector> At_func =
      [&At](const Vector &x) -> Vector { return At * x; };

  std::optional<Optimization::LinearAlgebra::LinearOperator<Vector, Vector>>
      At_op(At_func);

  /// Solve linearly-constrained trust-region subproblem *exactly* (i.e.,
  /// assuming an infinite trust-region radius and high-precision tolerance
  Vector s = Optimization::LinearAlgebra::STPCG<Vector, Vector>(
      large_g, large_Pop, inner_product, update_step_norm, num_iterations,
      Delta, 5 * STPCG_large_dim, kappa, theta, precon_op, At_op);

  // Verify that the computed update step lies in the null space of the
  // constraint operator A
  EXPECT_LT((A * s).norm(), eps_abs);

  // Verify that s agrees with the primal component of the primal-dual solution
  // z of the KKT system
  EXPECT_LT((s - s_gt).norm() / s_gt.norm(), eps_rel);

  // Verify that the M-norm of s is computed correctly
  Scalar s_M_norm = sqrt(s.dot(large_M * s));
  EXPECT_LT(fabs(s_M_norm - update_step_norm) / s_M_norm, eps_rel);
}

///// Test truncated STPCG on a large problem with an equality constraint, using
///// constraint preconditioning constraint
TEST_F(STPCGUnitTest, TruncatedPreconditionedProjectedSTPCG) {

  kappa = .1;
  theta = .7;
  Delta = std::numeric_limits<Scalar>::max();

  // Sample a random constraint operator A
  Matrix A = 1000 * Matrix::Random(STPCG_num_constraints, STPCG_large_dim);

  /// Construct constraint preconditioning operator Mc, satisfying:
  ///
  /// Mc(s) = (x, l), where (x,l) is the solution of:
  ///
  /// [M A'][x] = [s]
  /// [A 0 ][l] = [0]

  Matrix Mcdense(STPCG_large_dim + STPCG_num_constraints,
                 STPCG_large_dim + STPCG_num_constraints);
  Mcdense.topLeftCorner(STPCG_large_dim, STPCG_large_dim) = large_M;
  Mcdense.topRightCorner(STPCG_large_dim, STPCG_num_constraints) =
      A.transpose();
  Mcdense.bottomLeftCorner(STPCG_num_constraints, STPCG_large_dim) = A;

  SparseMatrix Mc = Mcdense.sparseView();
  Mc.makeCompressed();

  // Construct and cache factorization of Pc
  Eigen::UmfPackLU<SparseMatrix> Mcfact(Mc);
  Vector w = Vector::Zero(STPCG_large_dim + STPCG_num_constraints);

  // Construct preconditioning operator
  Optimization::LinearAlgebra::STPCGPreconditioner<Vector, Vector> McOp =
      [&Mcfact, &w](const Vector &r) -> std::pair<Vector, Vector> {
    // Construct and solve constraint preconditioning system
    w.head(STPCG_large_dim) = r;
    Vector z = Mcfact.solve(w);

    return std::make_pair(z.head(STPCG_large_dim),
                          z.tail(STPCG_num_constraints));
  };

  std::optional<
      Optimization::LinearAlgebra::STPCGPreconditioner<Vector, Vector>>
      precon_op(McOp);

  /// Construct transpose constraint operator
  Matrix At = A.transpose();
  Optimization::LinearAlgebra::LinearOperator<Vector, Vector> At_func =
      [&At](const Vector &x) -> Vector { return At * x; };

  std::optional<Optimization::LinearAlgebra::LinearOperator<Vector, Vector>>
      At_op(At_func);

  /// Approximately solve linearly-constrained trust-region subproblem
  Vector s = Optimization::LinearAlgebra::STPCG<Vector, Vector>(
      large_g, large_Pop, inner_product, update_step_norm, num_iterations,
      Delta, 5 * STPCG_large_dim, kappa, theta, precon_op, At_op);

  /// Verify that the target fractional reduction in the P-norm of the residual
  /// is satisfied by the returned update
  std::pair<Vector, Vector> Pr0 = McOp(large_g);
  Scalar r0_Pnorm = sqrt(large_g.dot(Pr0.first));

  Vector rk = large_g + large_P * s;
  std::pair<Vector, Vector> Prk = McOp(rk);
  Scalar rk_Pnorm = sqrt(rk.dot(Prk.first));
  EXPECT_LT(rk_Pnorm / r0_Pnorm, kappa);

  /// Verify that the M-norm of s is computed correctly
  Scalar s_M_norm = sqrt(s.dot(large_M * s));

  EXPECT_LT(fabs(s_M_norm - update_step_norm) / s_M_norm, eps_rel);
}

/// Test fixture for testing the LSQR linear least-squares solver
class LSQRUnitTest : public testing::Test {
protected:
  /// Set up variables and functions used in the following tests

  // Inner product between Vectors
  Optimization::LinearAlgebra::InnerProduct<Vector> inner_product;

  // Linear operators A and At
  Optimization::LinearAlgebra::LinearOperator<Vector, Vector> A_op;
  Optimization::LinearAlgebra::LinearOperator<Vector, Vector> At_op;

  virtual void SetUp() {

    // Inner product
    inner_product = [](const Vector &X, const Vector &Y) { return X.dot(Y); };
  }
};

/// Test LSQR on an inconsistent linear system where x = 0 is already a
/// stationary point for the loss function
TEST_F(LSQRUnitTest, TrivialSolution) {

  /// Simple test problem: try
  ///
  /// A = [0 0]
  ///     [1 0]
  ///     [0 1]
  ///
  /// b = [1, 0, 0];
  ///
  /// Note that this is an inconsistent linear system, but A'b = 0, and
  /// therefore x = 0 is already a stationary point for the least-squares loss
  /// function.  Consequently, LSQR should return immediately with x = 0

  Matrix A = Matrix::Zero(3, 2);
  A.bottomRows(2) = Matrix::Identity(2, 2);

  Vector b = Vector::Zero(3);
  b(0) = 1;

  // Set up linear operators
  A_op = [&A](const Vector &x) -> Vector { return A * x; };
  At_op = [&A](const Vector &x) -> Vector { return A.transpose() * x; };

  // Run LSQR!
  Scalar xnorm;
  size_t num_iterations;
  Vector x = Optimization::LinearAlgebra::LSQR<Vector>(
      A_op, At_op, b, inner_product, xnorm, num_iterations);

  /// Verify that LSQR terminated immediately
  EXPECT_EQ(num_iterations, 0);

  /// Verify that the reported norm of x is correct
  EXPECT_LT(fabs(x.norm() - xnorm), eps_abs);

  /// Verify that the returned x == 0
  EXPECT_LT(xnorm, eps_abs);
}

/// Test LSQR on a simple overdetermined but consistent least-squares problem
TEST_F(LSQRUnitTest, ConsistentOverdeterminedLeastSquares) {

  // Set up a small linear system
  Matrix A(4, 3);
  A << 10, 5, 10, 2, 9, 8, 10, 2, 10, 10, 5, 7;

  // Construct ground-truth vector x
  Vector xtrue(3);
  xtrue << 1.0, 2.0, 3.0;

  // Sample right-hand side vector b
  Vector b = A * xtrue;

  // Set up linear operators
  A_op = [&A](const Vector &x) -> Vector { return A * x; };
  At_op = [&A](const Vector &x) -> Vector { return A.transpose() * x; };

  // Run LSQR!
  size_t max_iterations = 1000;
  Scalar xnorm;
  size_t num_iterations;
  Vector x = Optimization::LinearAlgebra::LSQR<Vector>(
      A_op, At_op, b, inner_product, xnorm, num_iterations, max_iterations, 0.0,
      eps_rel);

  // Compute residual of returned solution
  Vector r = A * x - b;

  /// Verify that the returned solution satisfies the required reduction in
  /// relative residual
  EXPECT_LT(r.norm(), b.norm() * eps_rel);

  /// Verify that the reported norm of x is accurate
  EXPECT_LT(fabs(xnorm - x.norm()), eps_rel * x.norm());

  /// Verify that LSQR performed a reasonable number of iterations
  EXPECT_LT(num_iterations, 4 * A.cols());
}

/// Test LSQR on a simple *inconsistent* least-squares problem
TEST_F(LSQRUnitTest, InconsistentLeastSquares) {

  // Set up a small *inconsistent* linear system
  Matrix A(4, 3);
  A << 10, 5, 10, 2, 9, 8, 10, 2, 10, 10, 5, 7;

  Vector b(4);
  b << 1, 9, 10, 2;

  // Compute ground-truth solution x
  Eigen::FullPivHouseholderQR<Matrix> qr(A);
  Vector xtrue = qr.solve(b);

  // Set up linear operators
  A_op = [&A](const Vector &x) -> Vector { return A * x; };
  At_op = [&A](const Vector &x) -> Vector { return A.transpose() * x; };

  // Run LSQR!
  size_t max_iterations = 1000;
  Scalar xnorm;
  size_t num_iterations;
  Vector x = Optimization::LinearAlgebra::LSQR<Vector>(
      A_op, At_op, b, inner_product, xnorm, num_iterations, max_iterations, 0.0,
      0.0, eps_rel);

  /// Verify that the computed x is close to the correct solution
  EXPECT_LT((x - xtrue).norm(), x.norm());

  /// Verify that the reported norm of x is accurate
  EXPECT_LT(fabs(xnorm - x.norm()), eps_rel * x.norm());

  /// Verify that LSQR performed a reasonable number of iterations
  EXPECT_LT(num_iterations, 4 * A.cols());
}

/// Test LSQR on a simple *inconsistent* least-squares problem, enforcing a
/// trust-region constraint
TEST_F(LSQRUnitTest, InconsistentLeastSquaresWithTrustRegion) {

  // Set up a small *inconsistent* linear system
  Matrix A(4, 3);
  A << 10, 5, 10, 2, 9, 8, 10, 2, 10, 10, 5, 7;

  Vector b(4);
  b << 1, 9, 10, 2;

  // Compute ground-truth solution x
  Eigen::FullPivHouseholderQR<Matrix> qr(A);
  Vector xLS = qr.solve(b);

  // Get the norm of the LS solution
  Scalar xLS_norm = xLS.norm();

  // Note that since xLS solves the normal equations, it is *also* the
  // *minimum-norm* least-squares solution.  Therefore, by setting the
  // trust-region radius Delta to be less than xLS_norm, we can ensure that the
  // trust-region constraint will be binding for the resulting LS problem

  Scalar Delta = xLS_norm / 2;

  // Set up linear operators
  A_op = [&A](const Vector &x) -> Vector { return A * x; };
  At_op = [&A](const Vector &x) -> Vector { return A.transpose() * x; };

  // Run LSQR!
  size_t max_iterations = 1000;
  Scalar xnorm;
  size_t num_iterations;
  Vector x = Optimization::LinearAlgebra::LSQR<Vector>(
      A_op, At_op, b, inner_product, xnorm, num_iterations, max_iterations, 0.0,
      0.0, 0.0, 1e12, Delta);

  // Compute residual norm for solution
  Vector r = A * x - b;

  /// Verify that LSQR performed a reasonable number of iterations
  EXPECT_LT(num_iterations, 4 * A.cols());

  /// Verify that the reported norm of x is accurate
  EXPECT_LT(fabs(xnorm - x.norm()), eps_rel * x.norm());

  /// Verify that the solution returned by LSQR terminated on the trust-region
  /// boundary
  EXPECT_LT(fabs(xnorm - Delta), eps_abs);

  /// Verify that the returned solution does in fact reduce the least-squares
  /// residual
  EXPECT_LT(r.norm(), b.norm());
}

/// Test LSQR on a Tikhonov-regularized linear system
TEST_F(LSQRUnitTest, TikhonovRegularizedLeastSquares) {

  // Set up a small regularized linear least-squares problem
  Matrix A(4, 3);
  A << 10, 5, 10, 2, 9, 8, 10, 2, 10, 10, 5, 7;

  Vector b(4);
  b << 1, 9, 10, 2;

  Scalar lambda = 1.0;

  /// Compute solution for this small example using the normal equations

  // Compute Tikhonov-regularized coefficient matrix M := A'A + lambda*I
  Matrix M = A.transpose() * A + lambda * Matrix::Identity(3, 3);

  // Compute ground-truth solution x
  Eigen::FullPivHouseholderQR<Matrix> qr(M);
  Vector xtrue = qr.solve(A.transpose() * b);

  // Set up linear operators
  A_op = [&A](const Vector &x) -> Vector { return A * x; };
  At_op = [&A](const Vector &x) -> Vector { return A.transpose() * x; };

  // Run LSQR!
  size_t max_iterations = 1000;
  Scalar xnorm;
  size_t num_iterations;
  Vector x = Optimization::LinearAlgebra::LSQR<Vector>(
      A_op, At_op, b, inner_product, xnorm, num_iterations, max_iterations,
      lambda, 0.0, eps_rel);

  /// Verify that the computed x is close to the correct solution
  EXPECT_LT((x - xtrue).norm(), x.norm());

  /// Verify that the reported norm of x is accurate
  EXPECT_LT(fabs(xnorm - x.norm()), eps_rel * x.norm());

  /// Verify that LSQR performed a reasonable number of iterations
  EXPECT_LT(num_iterations, 4 * A.cols());
}
