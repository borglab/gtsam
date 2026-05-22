#include "Optimization/LinearAlgebra/LOBPCG.h"

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/Sparse>

#include <algorithm>
#include <limits>

#include "gtest/gtest.h"

// Typedef for the numerical type will use in the following tests
typedef double Scalar;

// Typedefs for Eigen matrix types
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
typedef Eigen::SparseMatrix<Scalar, Eigen::ColMajor> SparseMatrix;
typedef Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic> DiagonalMatrix;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
typedef Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>
    LinearOperator;
typedef Optimization::LinearAlgebra::LOBPCGUserFunction<Vector, Matrix> StopFun;

using namespace std;

class LOBPCGTest : public testing::Test {

protected:
  /// Test configuration

  // Dimension of linear operators
  size_t n = 1000;

  // Block size
  size_t m = 10;

  // Number of desired eigenvalues k
  size_t nev = 5;

  /// Termination criteria
  size_t max_iters = n;
  double tau = 1e-8;

  /// Test operators

  Vector Adiag, Bdiag;
  LinearOperator A, B, T;

  virtual void SetUp() {

    /// Set A to be multiplication by a diagonal matrix with a fixed spectrum
    // Set eigenvalues
    this->Adiag = Vector::LinSpaced(n, -.5 * n, .5 * n);

    A = [this](const Matrix &X) -> Matrix {
      return this->Adiag.asDiagonal() * X;
    };

    // Set B to be multiplication by a positive-definite matrix with a fixed
    // spectrum

    // Set diagonal
    this->Bdiag = Vector::LinSpaced(n, 1, n);

    B = [this](const Matrix &X) -> Matrix { return Bdiag.asDiagonal() * X; };

    /// Set preconditioner T to be multiplication by the inverse of the absolute
    /// values of A
    T = [this](const Matrix &X) -> Matrix {
      return this->Adiag.cwiseAbs().asDiagonal() * X;
    };
  }
};

/// Test the Rayleigh-Ritz helper function
TEST_F(LOBPCGTest, RayleighRitz) {

  // Sample two symmetric matrices of appropriate dimension
  size_t n = 7;

  /// Construct symmetric positive-definite metric matrix A
  Matrix AL = Matrix::Random(n, n);
  Matrix A = -AL * AL.transpose(); // Make A negative-definite

  // Construct symmetric positive-definite metric matrix B
  Matrix BL = Matrix::Random(n, n);
  Matrix B = BL * BL.transpose();

  Vector Theta;
  Matrix C;
  std::tie(Theta, C) =
      Optimization::LinearAlgebra::RayleighRitz<Vector, Matrix>(A, B);

  /// Verify that C'AC = Theta

  EXPECT_LT((C.transpose() * A * C - Matrix(Theta.asDiagonal())).norm(), 1e-8);

  /// Verify that C'BC = In
  EXPECT_LT((C.transpose() * B * C - Matrix::Identity(n, n)).norm(), 1e-8);
}

/// Test LOBPCG with a small standard eigenvalue problem
TEST_F(LOBPCGTest, SmallEigenValueProblem) {

  // Construct a diagonal matrix with the following spectrum
  Vector Lambda(4);
  Lambda << 1.0, 2.0, 3.0, 4.0;
  A = [&Lambda](const Matrix &X) -> Matrix { return Lambda.asDiagonal() * X; };

  // Set initial eigenvector estimates X0
  Matrix X0(4, 2);
  X0 << 0.8147, 0.6324, 0.9058, 0.0975, 0.1270, 0.2785, 0.9134, 0.5469;

  size_t nev = 2; // Number of requested eigenvalues

  // Run LOBPCG
  Vector Theta;
  Matrix X;
  size_t num_iters;
  size_t num_converged;
  std::tie(Theta, X) = Optimization::LinearAlgebra::LOBPCG<Vector, Matrix>(
      A, std::optional<LinearOperator>(nullopt),
      std::optional<LinearOperator>(nullopt), X0, nev, n, num_iters,
      num_converged, tau);

  /// Verify that the reported eigenvalues converged to the required tolerance
  EXPECT_EQ(num_converged, nev);

  /// Verify that the estimated eigenvalues are correct to high accuracy
  EXPECT_LT((Theta - Lambda.head(nev)).norm(), 1e-3);
}

/// Test LOBPCG with standard eigenvalue problem and no preconditioning
TEST_F(LOBPCGTest, EigenvalueProblem) {

  Vector Theta;
  Matrix X;
  size_t num_iters;
  size_t num_converged;
  std::tie(Theta, X) = Optimization::LinearAlgebra::LOBPCG<Vector, Matrix>(
      A, std::optional<LinearOperator>(nullopt),
      std::optional<LinearOperator>(nullopt), n, m, nev, 10 * n, num_iters,
      num_converged, tau);

  /// Verify that the reported eigenvalues converged to the required tolerance
  EXPECT_EQ(num_converged, nev);

  /// Verify that the estimated eigenvalues are correct to high accuracy
  Vector Lambda_true = Adiag.head(nev);
  EXPECT_LT((Theta - Lambda_true).norm(), 1e-4);
}

/// Test LOBPCG with standard eigenvalue problem and a simple (diagonal) PSD
/// preconditioner
TEST_F(LOBPCGTest, PreconditionedEigenvalueProblem) {

  Vector Theta;
  Matrix X;
  size_t num_iters;
  size_t num_converged;
  std::tie(Theta, X) = Optimization::LinearAlgebra::LOBPCG<Vector, Matrix>(
      A, std::optional<LinearOperator>(), std::optional<LinearOperator>(T), n,
      m, nev, n, num_iters, num_converged, tau);

  /// Verify that the method reported the correct number of converged
  /// eigenvalues
  EXPECT_EQ(num_converged, nev);

  /// Verify that the estimated eigenvalues are correct to high accuracy
  Vector Lambda_true = Adiag.head(nev);
  EXPECT_LT((Theta - Lambda_true).norm(), 1e-4);
}

/// Test LOBPCG on a generalized eigenvalue problem with preconditioning
TEST_F(LOBPCGTest, PreconditionedGeneralizedEigenvalueProblem) {

  Vector Theta;
  Matrix X;
  size_t num_iters;
  size_t num_converged;
  std::tie(Theta, X) = Optimization::LinearAlgebra::LOBPCG<Vector, Matrix>(
      A, std::optional<LinearOperator>(B), std::optional<LinearOperator>(T), n,
      m, nev, n, num_iters, num_converged, tau);

  // Calculate eigenvalues, using the fact that A and B are diagonal
  Vector Lambda_true = Bdiag.cwiseInverse().array() * Adiag.array();

  // Sort these values in-place in non-descending order
  std::sort(Lambda_true.data(), Lambda_true.data() + Lambda_true.size());

  /// Verify that the method reported the correct number of converged
  /// eigenvalues
  EXPECT_EQ(num_converged, nev);

  /// Verify that the estimated eigenvalues are correct to high accuracy
  EXPECT_LT((Theta - Lambda_true.head(nev)).norm(), 1e-4);
}

/// Test LOBPCG on a generalized eigenvalue problem with no preconditioner
TEST_F(LOBPCGTest, GeneralizedEigenvalueProblem) {

  Vector Theta;
  Matrix X;
  size_t num_iters;
  size_t num_converged;
  std::tie(Theta, X) = Optimization::LinearAlgebra::LOBPCG<Vector, Matrix>(
      A, std::optional<LinearOperator>(B), std::optional<LinearOperator>(), n,
      m, nev, n, num_iters, num_converged, tau);

  // Calculate eigenvalues, using the fact that A and B are diagonal
  Vector Lambda_true = Bdiag.cwiseInverse().array() * Adiag.array();

  // Sort these values in-place in non-descending order
  std::sort(Lambda_true.data(), Lambda_true.data() + Lambda_true.size());

  /// Verify that the method reported the correct number of converged
  /// eigenvalues
  EXPECT_EQ(num_converged, nev);

  /// Verify that the estimated eigenvalues are correct to high accuracy
  EXPECT_LT((Theta - Lambda_true.head(nev)).norm(), 1e-4);
}
