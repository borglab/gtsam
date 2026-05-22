#include "ILDL/ILDL.h"
#include "ILDL/ILDL_utils.h"

#include "solver.h"

#include <Eigen/Eigenvalues>
#include <complex>

#include "gtest/gtest.h"

using namespace Preconditioners;
using namespace std;

class ILDLTest : public testing::Test {
protected:
  /// Test configuration

  double rel_tol = 1e-6; // Relative error tolerance
  double eps = 1e-6;     // Absolute error tolerance

  /// Test data

  // Coefficient matrix
  SparseMatrix A;

  // Test vector x
  Vector xtest;

  // Pardiso options struct
  ILDLOpts opts;

  ILDL Afact;

  void SetUp() override {
    /// Set the upper triangle of A to be:
    ///
    /// A = 1  2  0  3
    ///       -5  0  0
    ///           0  4
    ///              7

    SparseMatrix AUT(4, 4);
    AUT.resize(4, 4);

    AUT.insert(0, 0) = 1;
    AUT.insert(0, 1) = 2;
    AUT.insert(0, 3) = 3;

    AUT.insert(1, 1) = -5;

    AUT.insert(2, 3) = 4;

    AUT.insert(3, 3) = 7;

    A = AUT.selfadjointView<Eigen::Upper>();

    // Randomly sample test vector x
    xtest = Vector::Random(A.rows());

    /// Set factorization configurations
    // Setting max-fill to a be huge and drop tol = 0 results in an exact LDL
    // factorization
    opts.equilibration = Equilibration::Bunch;
    opts.order = Ordering::AMD;
    opts.pivot_type = PivotType::BunchKaufman;
    opts.max_fill_factor = 1e3;
    opts.BK_pivot_tol = 0;
    opts.drop_tol = 0;
  }
};

TEST_F(ILDLTest, toCSR) {

  // Construct CSR representation of A

  std::vector<int> row_ptr;
  std::vector<int> col_idx;
  std::vector<Scalar> val;

  toCSR(A, row_ptr, col_idx, val);

  /// Verify that these vectors are what they should be

  // Check row_ptr
  std::vector<int> row_ptr_true = {0, 3, 4, 5, 6};
  for (size_t i = 0; i < row_ptr_true.size(); ++i) {
    EXPECT_EQ(row_ptr_true[i], row_ptr[i]);
  }

  // Check col_idx
  std::vector<int> col_idx_true = {0, 1, 3, 1, 3, 3};
  for (size_t i = 0; i < col_idx_true.size(); ++i) {
    EXPECT_EQ(col_idx_true[i], col_idx[i]);
  }

  // Check val
  std::vector<double> val_true = {1, 2, 3, -5, 4, 7};
  for (size_t i = 0; i < val_true.size(); ++i) {
    EXPECT_FLOAT_EQ(val_true[i], val[i]);
  }
}

/// Compute an *exact* LDL factorization, and verify that the elements P, S, L,
/// and D are computed correctly
TEST_F(ILDLTest, ExactFactorizationElements) {

  /// Compute factorization using SYM-ILDL's built-in solver

  // Construct CSR representation of A
  std::vector<int> row_ptr;
  std::vector<int> col_idx;
  std::vector<Scalar> val;
  toCSR(A, row_ptr, col_idx, val);

  symildl::solver<Scalar> solver;

  // Turn off messaging
  solver.msg_lvl = 0;

  // Load in initial matrix
  solver.load(row_ptr, col_idx, val);

  // Save initial matrix A
  solver.A.save("A.txt", true);

  // Set reordering scheme
  switch (opts.order) {
  case Ordering::AMD:
    solver.reorder_type = symildl::reordering_type::AMD;
    break;
  case Ordering::RCM:
    solver.reorder_type = symildl::reordering_type::RCM;
    break;
  case Ordering::None:
    solver.reorder_type = symildl::reordering_type::NONE;
    break;
  }

  // Set equilibration scheme
  solver.equil_type = (opts.equilibration == Equilibration::Bunch
                           ? symildl::equilibration_type::BUNCH
                           : symildl::equilibration_type::NONE);

  // Set pivoting type
  solver.piv_type = (opts.pivot_type == PivotType::Rook
                         ? lilc_matrix<Scalar>::pivot_type::ROOK
                         : lilc_matrix<Scalar>::pivot_type::BKP);

  solver.has_rhs = false;
  solver.perform_inplace = false;
  solver.solve(opts.max_fill_factor, opts.drop_tol, opts.BK_pivot_tol);

  /// Compute factorization using ILDL

  // Set factorization options
  Afact.setOptions(opts);

  // Compute factorization
  Afact.compute(A);

  // Extract elements of this factorization
  SparseMatrix D = Afact.D();
  const SparseMatrix &L = Afact.L();

  /// Ensure that the elements P, S, L, and D of the factorizations computed by
  /// SYM-ILDL and ILDL coincide

  /// Ensure that the permutations P agree
  EXPECT_EQ(Afact.P().size(), solver.perm.size());
  for (int k = 0; k < Afact.P().size(); ++k)
    EXPECT_EQ(Afact.P()(k), solver.perm[k]);

  /// Ensure that the scaling matrices agree
  EXPECT_EQ(Afact.S().size(), solver.A.S.main_diag.size());
  for (int k = 0; k < Afact.S().size(); ++k)
    EXPECT_FLOAT_EQ(Afact.S()(k), solver.A.S.main_diag[k]);

  /// Ensure that the lower-triangular factors agree
  EXPECT_EQ(Afact.L().nonZeros(), solver.L.nnz());
  for (int k = 0; k < Afact.L().outerSize(); ++k)
    for (SparseMatrix::InnerIterator it(Afact.L(), k); it; ++it)
      EXPECT_FLOAT_EQ(it.value(), solver.L.coeff(it.row(), it.col()));

  /// Ensure that the block-diagonal matrices D agree

  // Extract lower triangle from D
  SparseMatrix DLT = D.triangularView<Eigen::Lower>();
  EXPECT_EQ(DLT.nonZeros(), solver.D.nnz());
  for (int k = 0; k < DLT.outerSize(); ++k)
    for (SparseMatrix::InnerIterator it(DLT, k); it; ++it) {
      int i = it.row();
      int j = it.col();
      if (i == j) {
        // This is a diagonal element
        EXPECT_LT(fabs(it.value() - solver.D.main_diag.at(i)), eps);
      } else {
        // This is the off-diagonal element *below* the element D(j,j)
        EXPECT_LT(fabs(it.value() - solver.D.off_diag.at(j)), eps);
      }
    }

  /// Save the matrices constructed by the SYM-ILDL solver

  std::ofstream perm_file("P.txt");
  for (const auto &i : solver.perm)
    perm_file << i << " ";
  perm_file << std::endl;
  perm_file.close();

  solver.A.S.save("S.txt");
  solver.L.save("L.txt", true);
  solver.D.save("D.txt");

  /// Verify that the elements of the factorization satisfy P'SASP = LDL'

  SparseMatrix SAS = Afact.S().asDiagonal() * A * Afact.S().asDiagonal();
  SparseMatrix PtSASP;
  PtSASP = SAS.twistedBy(Afact.P().asPermutation().inverse());

  SparseMatrix LDLt = Afact.L() * Afact.D() * Afact.L().transpose();

  EXPECT_LT((PtSASP - LDLt).norm(), rel_tol * PtSASP.norm());
}

/// Compute a modified LDL factorization, modifying D to ensure that it is
/// positive-definite
TEST_F(ILDLTest, PositiveDefiniteModification) {

  // Set factorization options
  Afact.setOptions(opts);

  // Compute factorization
  Afact.compute(A);

  // Extract diagonal matrix D
  Matrix D = Afact.D();

  // Extract diagonal matrix D, modifying it to ensure positive-definiteness
  Matrix Dpos = Afact.D(true);

  // Compute the matrix P = Dinv*Dpos
  Matrix P = D.inverse() * Dpos;

  /// We modify D by taking the absolute values of its eigenvalues -- therefore,
  /// the only eigenvalues of Dinv*Dpos should be +/- 1
  Eigen::EigenSolver<Matrix> eigs(P);

  for (int k = 0; k < eigs.eigenvalues().size(); ++k)
    EXPECT_TRUE(fabs(eigs.eigenvalues()(k) - 1.0) < eps ||
                (fabs(eigs.eigenvalues()(k) + 1.0) < eps));
}

TEST_F(ILDLTest, Inertia) {

  // Set factorization options
  Afact.setOptions(opts);

  // Compute factorization
  Afact.compute(A);

  Inertia inertia = Afact.inertia();

  // The test matrix A has 2 positive and 2 negative eigenvalues
  EXPECT_EQ(inertia.first, 2);
  EXPECT_EQ(inertia.second, 2);
}

/// Test computation of products with the diagonal matrix D
TEST_F(ILDLTest, DProduct) {

  // Set factorization options
  Afact.setOptions(opts);

  // Compute factorization
  Afact.compute(A);

  // Extract diagonal matrix D
  Matrix D = Afact.D();

  // Extract diagonal matrix D, modifying it to ensure positive-definiteness
  Matrix Dpos = Afact.D(true);

  /// Test computation of products with diagonal matrix
  Vector ygt = D * xtest;
  Vector y = Afact.Dproduct(xtest);
  EXPECT_LT((ygt - y).norm(), rel_tol * ygt.norm());

  /// Test computation of products with positive-definite modification of
  /// diagonal matrix
  ygt = Dpos * xtest;
  y = Afact.Dproduct(xtest, true);
  EXPECT_LT((ygt - y).norm(), rel_tol * ygt.norm());
}

/// Test solving linear systems of the form Dx = b
TEST_F(ILDLTest, Dsolve) {

  // Set factorization options
  Afact.setOptions(opts);

  // Compute factorization
  Afact.compute(A);

  // Extract diagonal matrix D
  Matrix D = Afact.D();

  // Extract diagonal matrix D, modifying it to ensure positive-definiteness
  Matrix Dpos = Afact.D(true);

  /// Test computation of products with diagonal matrix
  Vector ygt = D.inverse() * xtest;
  Vector y = Afact.Dsolve(xtest);
  EXPECT_LT((ygt - y).norm(), rel_tol * ygt.norm());

  /// Test computation of products with positive-definite modification of
  /// diagonal matrix
  ygt = Dpos.inverse() * xtest;
  y = Afact.Dsolve(xtest, true);
  EXPECT_LT((ygt - y).norm(), rel_tol * ygt.norm());
}

/// Test solving linear systems of the form (D+)^{1/2} x = b, were D+ is the
/// positive-definite modification of the block-diagonal matrix Ds
TEST_F(ILDLTest, sqrtDsolve) {

  // Set factorization options
  Afact.setOptions(opts);

  // Compute factorization
  Afact.compute(A);

  // Extract diagonal matrix D
  Matrix D = Afact.D();

  // Compute symmetric eigendecomposition of D
  Eigen::SelfAdjointEigenSolver<Matrix> eig(D);

  Matrix Q = eig.eigenvectors();
  Vector Lambda = eig.eigenvalues();

  // Compute ground-truth solution ygt
  Vector ygt = Q * Lambda.cwiseAbs().cwiseSqrt().cwiseInverse().asDiagonal() *
               Q.transpose() * xtest;

  Vector y = Afact.sqrtDsolve(xtest);
  EXPECT_LT((ygt - y).norm(), rel_tol * ygt.norm());
}

/// Test solving linear systems of the form LDL' x = b
TEST_F(ILDLTest, LDLTsolve) {

  // Set factorization options
  Afact.setOptions(opts);

  // Compute factorization
  Afact.compute(A);

  // Extract diagonal matrix D
  Matrix D = Afact.D();
  Matrix Dpos = Afact.D(true);

  Matrix LDLt = Afact.L() * D * Afact.L().transpose();
  Matrix LDposLt = Afact.L() * Dpos * Afact.L().transpose();

  /// Compute ground-truth solution
  Vector ygt = LDLt.inverse() * xtest;

  Vector y = Afact.LDLTsolve(xtest);
  EXPECT_LT((ygt - y).norm(), rel_tol * ygt.norm());

  /// Compute ground-truth solution for positive-definite modification
  ygt = LDposLt.inverse() * xtest;

  y = Afact.LDLTsolve(xtest, true);
  EXPECT_LT((ygt - y).norm(), rel_tol * ygt.norm());
}

/// Test solving linear systems of the form (D+)^{1/2} L' x = b
TEST_F(ILDLTest, sqrtDLTsolve) {

  // Set factorization options
  Afact.setOptions(opts);

  // Compute factorization
  Afact.compute(A);

  // Extract diagonal matrix Dpos
  Matrix Dpos = Afact.D(true);

  // Compute symmetric square square root of D
  Eigen::SelfAdjointEigenSolver<Matrix> eig(Dpos);
  Matrix sqrtD = eig.eigenvectors() *
                 eig.eigenvalues().cwiseSqrt().asDiagonal() *
                 eig.eigenvectors().transpose();

  Matrix sqrtDLt = sqrtD * Afact.L().transpose();
  Matrix sqrtDLt_inv = sqrtDLt.inverse();

  /// Compute ground-truth solution for (D+)^{1/2} L' x = b
  Vector ygt = sqrtDLt_inv * xtest;
  Vector y = Afact.sqrtDLTsolve(xtest);
  EXPECT_LT((ygt - y).norm(), rel_tol * ygt.norm());

  /// Compute ground-truth solution for L(D+)^{1/2} x = b
  ygt = sqrtDLt_inv.transpose() * xtest;
  y = Afact.sqrtDLTsolve(xtest, true);
  EXPECT_LT((ygt - y).norm(), rel_tol * ygt.norm());
}

/// Test approximate solution of Ax = b using incomplete factorization
TEST_F(ILDLTest, solve) {

  // Set factorization options
  Afact.setOptions(opts);

  // Compute factorization
  Afact.compute(A);

  // Compute A^-1
  Matrix Ainv_gt = A.toDense().inverse();

  // Compute A^-1 by applying the preconditioner Afact to each column of the
  // identity matrix Id
  Matrix Id = Matrix::Identity(A.rows(), A.cols());

  Matrix Ainv(Afact.dim(), Afact.dim());

  for (int k = 0; k < Afact.dim(); ++k)
    Ainv.col(k) = Afact.solve(Id.col(k));

  EXPECT_LT((Ainv - Ainv_gt).norm(), rel_tol * Ainv_gt.norm());
}
