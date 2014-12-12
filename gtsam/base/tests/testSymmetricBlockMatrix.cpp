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
#include <boost/assign/list_of.hpp>

using namespace std;
using namespace gtsam;
using boost::assign::list_of;

static SymmetricBlockMatrix testBlockMatrix(
  list_of(3)(2)(1),
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
  Matrix actual1 = testBlockMatrix(1, 1);
  // Test only writing the upper triangle for efficiency
  Matrix actual1t = Matrix::Zero(2, 2);
  actual1t.triangularView<Eigen::Upper>() = testBlockMatrix(1, 1).triangularView();
  EXPECT(assert_equal(expected1, actual1));
  EXPECT(assert_equal(Matrix(expected1.triangularView<Eigen::Upper>()), actual1t));

  // Above the diagonal
  Matrix expected2 = (Matrix(3, 2) <<
    4, 5,
    10, 11,
    16, 17).finished();
  Matrix actual2 = testBlockMatrix(0, 1);
  EXPECT(assert_equal(expected2, actual2));

  // Below the diagonal
  Matrix expected3 = (Matrix(2, 3) <<
    4, 10, 16,
    5, 11, 17).finished();
  Matrix actual3 = testBlockMatrix(1, 0);
  EXPECT(assert_equal(expected3, actual3));
}

/* ************************************************************************* */
TEST(SymmetricBlockMatrix, WriteBlocks)
{
  // On the diagonal
  Matrix expected1 = testBlockMatrix(1, 1);
  SymmetricBlockMatrix bm1 = SymmetricBlockMatrix::LikeActiveViewOf(testBlockMatrix);
  bm1(1, 1) = expected1.selfadjointView<Eigen::Upper>(); // Verified with debugger that this only writes the upper triangle
  Matrix actual1 = bm1(1, 1);
  EXPECT(assert_equal(expected1, actual1));

  // On the diagonal
  Matrix expected1p = testBlockMatrix(1, 1);
  SymmetricBlockMatrix bm1p = SymmetricBlockMatrix::LikeActiveViewOf(testBlockMatrix);
  bm1p(1, 1) = expected1p; // Verified with debugger that this only writes the upper triangle
  Matrix actual1p = bm1p(1, 1);
  EXPECT(assert_equal(expected1p, actual1p));

  // Above the diagonal
  Matrix expected2 = testBlockMatrix(0, 1);
  SymmetricBlockMatrix bm2 = SymmetricBlockMatrix::LikeActiveViewOf(testBlockMatrix);
  bm2(0, 1) = expected2;
  Matrix actual2 = bm2(0, 1);
  EXPECT(assert_equal(expected2, actual2));

  // Below the diagonal
  Matrix expected3 = testBlockMatrix(1, 0);
  SymmetricBlockMatrix bm3 = SymmetricBlockMatrix::LikeActiveViewOf(testBlockMatrix);
  bm3(1, 0) = expected3;
  Matrix actual3 = bm3(1, 0);
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
  Matrix actual1 = testBlockMatrix.range(1, 3, 1, 3).selfadjointView();
  Matrix actual1a = testBlockMatrix.range(1, 3, 1, 3);
  EXPECT(assert_equal(expected1, actual1));
  EXPECT(assert_equal(expected1, actual1a));

  // Above the diagonal
  Matrix expected2 = (Matrix(3, 1) <<
    24,
    30,
    36).finished();
  Matrix actual2 = testBlockMatrix.range(1, 3, 2, 3).knownOffDiagonal();
  Matrix actual2a = testBlockMatrix.range(1, 3, 2, 3);
  EXPECT(assert_equal(expected2, actual2));
  EXPECT(assert_equal(expected2, actual2a));

  // Below the diagonal
  Matrix expected3 = (Matrix(3, 3) <<
    4, 10, 16,
    5, 11, 17,
    6, 12, 18).finished();
  Matrix actual3 = testBlockMatrix.range(1, 3, 0, 1).knownOffDiagonal();
  Matrix actual3a = testBlockMatrix.range(1, 3, 0, 1);
  EXPECT(assert_equal(expected3, actual3));
  EXPECT(assert_equal(expected3, actual3a));
}

/* ************************************************************************* */
TEST(SymmetricBlockMatrix, expressions)
{
  SymmetricBlockMatrix expected1(list_of(2)(3)(1), (Matrix(6, 6) <<
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 4, 6, 8, 0,
    0, 0, 0, 9, 12, 0,
    0, 0, 0, 0, 16, 0,
    0, 0, 0, 0, 0, 0).finished());

  SymmetricBlockMatrix expected2(list_of(2)(3)(1), (Matrix(6, 6) <<
    0, 0, 10, 15, 20, 0,
    0, 0, 12, 18, 24, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0).finished());

  Matrix a = (Matrix(1, 3) << 2, 3, 4).finished();
  Matrix b = (Matrix(1, 2) << 5, 6).finished();

  SymmetricBlockMatrix bm1(list_of(2)(3)(1));
  bm1.full().triangularView().setZero();
  bm1(1, 1).selfadjointView().rankUpdate(a.transpose());
  EXPECT(assert_equal(Matrix(expected1.full().selfadjointView()), bm1.full().selfadjointView()));

  SymmetricBlockMatrix bm2(list_of(2)(3)(1));
  bm2.full().triangularView().setZero();
  bm2(0, 1).knownOffDiagonal() += b.transpose() * a;
  EXPECT(assert_equal(Matrix(expected2.full().selfadjointView()), bm2.full().selfadjointView()));

  SymmetricBlockMatrix bm3(list_of(2)(3)(1));
  bm3.full().triangularView().setZero();
  bm3(1, 0).knownOffDiagonal() += a.transpose() * b;
  EXPECT(assert_equal(Matrix(expected2.full().selfadjointView()), bm3.full().selfadjointView()));

  SymmetricBlockMatrix bm4(list_of(2)(3)(1));
  bm4.full().triangularView().setZero();
  bm4(1, 1) += expected1(1, 1);
  EXPECT(assert_equal(Matrix(expected1.full().selfadjointView()), bm4.full().selfadjointView()));

  SymmetricBlockMatrix bm5(list_of(2)(3)(1));
  bm5.full().triangularView().setZero();
  bm5(0, 1) += expected2(0, 1);
  EXPECT(assert_equal(Matrix(expected2.full().selfadjointView()), bm5.full().selfadjointView()));

  SymmetricBlockMatrix bm6(list_of(2)(3)(1));
  bm6.full().triangularView().setZero();
  bm6(1, 0) += expected2(1, 0);
  EXPECT(assert_equal(Matrix(expected2.full().selfadjointView()), bm6.full().selfadjointView()));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

