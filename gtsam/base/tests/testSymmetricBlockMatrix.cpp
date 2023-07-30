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

