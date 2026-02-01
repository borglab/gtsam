/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file   testSymmetricBlockMatrix.cpp
* @brief  Unit tests for SymmetricBlockMatrix class
* @author Richard Roberts
**/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/VerticalBlockMatrix.h>

using namespace std;
using namespace gtsam;

static SymmetricBlockMatrix testBlockMatrix(
  std::vector<size_t>{3, 2, 1},
  (Matrix(6, 6) <<
  1, 2, 3, 4, 5, 6,
  2, 8, 9, 10, 11, 12,
  3, 9, 15, 16, 17, 18,
  4, 10, 16, 22, 23, 24,
  5, 11, 17, 23, 29, 30,
  6, 12, 18, 24, 30, 36).finished());

/* ************************************************************************* */
// Read block accessors.
TEST(SymmetricBlockMatrix, ReadBlocks)
{
  // On the diagonal
  Matrix expected1 = (Matrix(2, 2) <<
    22, 23,
    23, 29).finished();
  Matrix actual1 = testBlockMatrix.diagonalBlock(1);
  EXPECT(assert_equal(expected1, actual1));

  // Above the diagonal
  Matrix expected2 = (Matrix(3, 2) <<
    4, 5,
    10, 11,
    16, 17).finished();
  Matrix actual2 = testBlockMatrix.aboveDiagonalBlock(0, 1);
  EXPECT(assert_equal(expected2, actual2));
}

/* ************************************************************************* */
// Write block setters.
TEST(SymmetricBlockMatrix, WriteBlocks)
{
  // On the diagonal
  Matrix expected1 = testBlockMatrix.diagonalBlock(1);
  SymmetricBlockMatrix bm1 = SymmetricBlockMatrix::LikeActiveViewOf(testBlockMatrix);

  bm1.setDiagonalBlock(1, expected1);
  Matrix actual1 = bm1.diagonalBlock(1);
  EXPECT(assert_equal(expected1, actual1));

  // Above the diagonal
  Matrix expected2 = testBlockMatrix.aboveDiagonalBlock(0, 1);
  SymmetricBlockMatrix bm2 = SymmetricBlockMatrix::LikeActiveViewOf(testBlockMatrix);
  bm2.setOffDiagonalBlock(0, 1, expected2);
  Matrix actual2 = bm2.aboveDiagonalBlock(0, 1);
  EXPECT(assert_equal(expected2, actual2));

  // Below the diagonal
  Matrix expected3 = testBlockMatrix.aboveDiagonalBlock(0, 1).transpose();
  SymmetricBlockMatrix bm3 = SymmetricBlockMatrix::LikeActiveViewOf(testBlockMatrix);
  bm3.setOffDiagonalBlock(1, 0, expected3);
  Matrix actual3 = bm3.aboveDiagonalBlock(0, 1).transpose();
  EXPECT(assert_equal(expected3, actual3));
}

/* ************************************************************************* */
// Verify block range access.
TEST(SymmetricBlockMatrix, Ranges)
{
  // On the diagonal
  Matrix expected1 = (Matrix(3, 3) <<
    22, 23, 24,
    23, 29, 30,
    24, 30, 36).finished();
  Matrix actual1 = testBlockMatrix.selfadjointView(1, 3);
  EXPECT(assert_equal(expected1, actual1));

  // Above the diagonal
  Matrix expected2 = (Matrix(3, 3) <<
    4, 5, 6,
    10, 11, 12,
    16, 17, 18).finished();
  Matrix actual2 = testBlockMatrix.aboveDiagonalRange(0, 1, 1, 3);
  EXPECT(assert_equal(expected2, actual2));
}

/* ************************************************************************* */
// Exercise block expression helpers.
TEST(SymmetricBlockMatrix, expressions)
{
  const std::vector<size_t> dimensions{2, 3, 1};
  SymmetricBlockMatrix expected1(dimensions, (Matrix(6, 6) <<
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 4, 6, 8, 0,
    0, 0, 0, 9, 12, 0,
    0, 0, 0, 0, 16, 0,
    0, 0, 0, 0, 0, 0).finished());

  SymmetricBlockMatrix expected2(dimensions, (Matrix(6, 6) <<
    0, 0, 10, 15, 20, 0,
    0, 0, 12, 18, 24, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0).finished());

  Matrix a = (Matrix(1, 3) << 2, 3, 4).finished();
  Matrix b = (Matrix(1, 2) << 5, 6).finished();

  SymmetricBlockMatrix bm1(dimensions);
  bm1.setZero();
  bm1.diagonalBlock(1).rankUpdate(a.transpose());
  EXPECT(assert_equal(Matrix(expected1.selfadjointView()), bm1.selfadjointView()));

  SymmetricBlockMatrix bm2(dimensions);
  bm2.setZero();
  bm2.updateOffDiagonalBlock(0, 1, b.transpose() * a);
  EXPECT(assert_equal(Matrix(expected2.selfadjointView()), bm2.selfadjointView()));

  SymmetricBlockMatrix bm3(dimensions);
  bm3.setZero();
  bm3.updateOffDiagonalBlock(1, 0, a.transpose() * b);
  EXPECT(assert_equal(Matrix(expected2.selfadjointView()), bm3.selfadjointView()));

  SymmetricBlockMatrix bm4(dimensions);
  bm4.setZero();
  bm4.updateDiagonalBlock(1, expected1.diagonalBlock(1));
  EXPECT(assert_equal(Matrix(expected1.selfadjointView()), bm4.selfadjointView()));

  SymmetricBlockMatrix bm5(dimensions);
  bm5.setZero();
  bm5.updateOffDiagonalBlock(0, 1, expected2.aboveDiagonalBlock(0, 1));
  EXPECT(assert_equal(Matrix(expected2.selfadjointView()), bm5.selfadjointView()));

  SymmetricBlockMatrix bm6(dimensions);
  bm6.setZero();
  bm6.updateOffDiagonalBlock(1, 0, expected2.aboveDiagonalBlock(0, 1).transpose());
  EXPECT(assert_equal(Matrix(expected2.selfadjointView()), bm6.selfadjointView()));
}

/* ************************************************************************* */
// Verify diagonal-only update helpers.
TEST(SymmetricBlockMatrix, AddDiagonal) {
  const std::vector<size_t> dimensions{2, 1};
  SymmetricBlockMatrix bm(dimensions);
  bm.setZero();

  bm.addScaledIdentity(0, 2.0);
  bm.addScaledIdentity(1, 3.0);

  Vector delta0(2);
  delta0 << 1.0, 4.0;
  bm.addToDiagonalBlock(0, delta0);

  Vector delta1(1);
  delta1 << -1.0;
  bm.addToDiagonalBlock(1, delta1);

  Matrix expected = Matrix::Zero(3, 3);
  expected(0, 0) = 3.0;
  expected(1, 1) = 6.0;
  expected(2, 2) = 2.0;

  EXPECT(assert_equal(expected, Matrix(bm.selfadjointView())));
}

/* ************************************************************************* */
// Update via block mapping.
TEST(SymmetricBlockMatrix, UpdateFromMappedBlocks)
{
  const std::vector<size_t> destDims{1, 3, 2};
  const std::vector<DenseIndex> mapping{1, 2, 0};

  SymmetricBlockMatrix actual(destDims);
  actual.setZero();
  actual.updateFromMappedBlocks(testBlockMatrix, mapping);

  SymmetricBlockMatrix expected(destDims);
  expected.setZero();
  for (DenseIndex i = 0; i < testBlockMatrix.nBlocks(); ++i) {
    const DenseIndex I = static_cast<DenseIndex>(mapping[i]);
    expected.updateDiagonalBlock(I, testBlockMatrix.diagonalBlock(i));
    for (DenseIndex j = i + 1; j < testBlockMatrix.nBlocks(); ++j) {
      const DenseIndex J = static_cast<DenseIndex>(mapping[j]);
      expected.setOffDiagonalBlock(I, J,
                                   testBlockMatrix.aboveDiagonalBlock(i, j));
    }
  }
  EXPECT(assert_equal(Matrix(expected.selfadjointView()),
                      actual.selfadjointView()));

  SymmetricBlockMatrix doubled(destDims);
  doubled.setZero();
  doubled.updateFromMappedBlocks(testBlockMatrix, mapping);
  doubled.updateFromMappedBlocks(testBlockMatrix, mapping);
  EXPECT(assert_equal(2.0 * Matrix(expected.selfadjointView()),
                      Matrix(doubled.selfadjointView())));
}

/* ************************************************************************* */
// Update via blockwise outer products from a VerticalBlockMatrix view.
TEST(SymmetricBlockMatrix, UpdateFromOuterProductBlocks)
{
  const std::vector<size_t> vbmDims{2, 1};
  VerticalBlockMatrix vbm(vbmDims, 4, true);
  vbm.matrix() = (Matrix(4, 4) <<
    1, 2, 3, 4,
    5, 6, 7, 8,
    9, 10, 11, 12,
    13, 14, 15, 16).finished();

  const std::vector<size_t> destDims{1};
  SymmetricBlockMatrix actual(destDims, true);
  actual.setZero();
  const std::vector<DenseIndex> mapping{0, 1};
  const Matrix S = vbm.range(1, 3);
  const Matrix expected = S.transpose() * S;
  vbm.firstBlock() = 1;
  actual.updateFromOuterProductBlocks(vbm, mapping);
  EXPECT(assert_equal(expected, Matrix(actual.selfadjointView())));
}

/* ************************************************************************* */
// In-place inversion path.
TEST(SymmetricBlockMatrix, inverseInPlace) {
  // generate an invertible matrix
  const Vector3 a(1.0, 0.2, 2.0), b(0.3, 0.8, -1.0), c(0.1, 0.2, 0.7);
  Matrix inputMatrix(3, 3);
  inputMatrix.setZero();
  inputMatrix += a * a.transpose();
  inputMatrix += b * b.transpose();
  inputMatrix += c * c.transpose();
  const Matrix expectedInverse = inputMatrix.inverse();

  const std::vector<size_t> dimensions{2, 1};
  SymmetricBlockMatrix symmMatrix(dimensions, inputMatrix);
  // invert in place
  symmMatrix.invertInPlace();
  EXPECT(assert_equal(expectedInverse, symmMatrix.selfadjointView()));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
