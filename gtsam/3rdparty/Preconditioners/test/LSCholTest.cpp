#include "LSChol/LSChol.h"
#include "gtest/gtest.h"

using namespace Preconditioners;
using namespace std;

class LSCholTest : public testing::Test {
protected:
  /// Test configuration

  double rel_tol = 1e-6; // Relative error tolerance
  double eps = 1e-6;     // Absolute error tolerance

  /// Test data

  SparseMatrix A;
  LSChol Afact;

  void SetUp() override {
    A.resize(4, 3);

    A.insert(0, 0) = 1;
    A.insert(0, 1) = 10;
    A.insert(0, 2) = 5;

    A.insert(1, 0) = 3;
    A.insert(1, 1) = 2;
    A.insert(1, 2) = 9;

    A.insert(2, 0) = 6;
    A.insert(2, 2) = 2;

    A.insert(3, 0) = 10;
  }
};

/// Basic test: check computation of factorization
TEST_F(LSCholTest, compute) {

  /// Perform factorization
  Afact.compute(A);

  /// Extract triangular factor
  const SparseMatrix &R = Afact.R();

  /// Check output
  EXPECT_EQ(R.rows(), A.cols());
  EXPECT_EQ(R.cols(), A.cols());
  EXPECT_EQ(Afact.rank(), A.cols());
  EXPECT_EQ(Afact.P().size(), A.cols());
}

/// Check computation of products with PR^{-1}
TEST_F(LSCholTest, PRinv) {

  /// Perform factorization
  Afact.compute(A);
  size_t d = Afact.R().rows();

  /// Compute the product matrix PRinv column-by-column
  Matrix Id = Matrix::Identity(Afact.R().rows(), Afact.R().cols());
  Matrix PRinv(d, d);
  for (size_t k = 0; k < d; ++k)
    PRinv.col(k) = Afact.PRinv(Id.col(k));

  /// Compute the product A*P*R^{-1}
  Matrix APRinv = A * PRinv;

  // AP = QR => APR^{-1} = Q, and therefore:
  //
  // (APR^{-1})^T * (APR^{-1} = Q^T * Q = Id
  //
  // So we check that this identity holds

  Matrix S = APRinv.transpose() * APRinv;

  EXPECT_LT((S - Id).norm(), eps);
}

/// Check computation of products with R^{-T} * P^{-1}
TEST_F(LSCholTest, RinvTPinv) {

  /// Perform factorization
  Afact.compute(A);
  size_t d = Afact.R().rows();

  /// Compute the product matrix R^{-T} * P^{-1} column-by-column
  Matrix Id = Matrix::Identity(Afact.R().rows(), Afact.R().cols());
  Matrix RinvTPinv(d, d);
  for (size_t k = 0; k < d; ++k)
    RinvTPinv.col(k) = Afact.RinvTPinv(Id.col(k));

  /// Compute the product R^{-T} * P^{-1} * A^T
  Matrix RinvTPinvAT = RinvTPinv * A.transpose();

  // AP = QR => R^{-T} * Pinv * A^T = Q^T, and therefore:
  //
  // (R^{-T} * Pinv * A^T) * (R^{-T} * Pinv * A^T)^T = Q^T * Q = Id
  //
  // So we check that this identity holds

  Matrix S = RinvTPinvAT * RinvTPinvAT.transpose();

  EXPECT_LT((S - Id).norm(), eps);
}

/// Check computation of products with R^{-T} * P^{-1}
TEST_F(LSCholTest, solve) {

  /// Perform factorization
  Afact.compute(A);
  size_t d = Afact.R().rows();

  /// Compute product A'*A
  Matrix M = A.transpose() * A;

  /// Verify that solve(A'A) = Id

  // Compute this result column-by-column
  Matrix prod(d, d);
  for (size_t k = 0; k < d; ++k)
    prod.col(k) = Afact.solve(M.col(k));

  Matrix Id = Matrix::Identity(d, d);

  EXPECT_LT((Id - prod).norm(), eps);
}
