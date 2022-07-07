/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testParameterMatrix.cpp
 * @date Sep 22, 2020
 * @author Varun Agrawal, Frank Dellaert
 * @brief Unit tests for ParameterMatrix.h
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/basis/BasisFactors.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/basis/ParameterMatrix.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;

using Parameters = Chebyshev2::Parameters;

const size_t M = 2, N = 5;

//******************************************************************************
TEST(ParameterMatrix, Constructor) {
  ParameterMatrix<2> actual1(3);
  ParameterMatrix<2> expected1(Matrix::Zero(2, 3));
  EXPECT(assert_equal(expected1, actual1));

  ParameterMatrix<2> actual2(Matrix::Ones(2, 3));
  ParameterMatrix<2> expected2(Matrix::Ones(2, 3));
  EXPECT(assert_equal(expected2, actual2));
  EXPECT(assert_equal(expected2.matrix(), actual2.matrix()));
}

//******************************************************************************
TEST(ParameterMatrix, Dimensions) {
  ParameterMatrix<M> params(N);
  EXPECT_LONGS_EQUAL(params.rows(), M);
  EXPECT_LONGS_EQUAL(params.cols(), N);
  EXPECT_LONGS_EQUAL(params.dim(), M * N);
}

//******************************************************************************
TEST(ParameterMatrix, Getters) {
  ParameterMatrix<M> params(N);

  Matrix expectedMatrix = Matrix::Zero(2, 5);
  EXPECT(assert_equal(expectedMatrix, params.matrix()));

  Matrix expectedMatrixTranspose = Matrix::Zero(5, 2);
  EXPECT(assert_equal(expectedMatrixTranspose, params.transpose()));

  ParameterMatrix<M> p2(Matrix::Ones(M, N));
  Vector expectedRowVector = Vector::Ones(N);
  for (size_t r = 0; r < M; ++r) {
    EXPECT(assert_equal(p2.row(r), expectedRowVector));
  }

  ParameterMatrix<M> p3(2 * Matrix::Ones(M, N));
  Vector expectedColVector = 2 * Vector::Ones(M);
  for (size_t c = 0; c < M; ++c) {
    EXPECT(assert_equal(p3.col(c), expectedColVector));
  }
}

//******************************************************************************
TEST(ParameterMatrix, Setters) {
  ParameterMatrix<M> params(Matrix::Zero(M, N));
  Matrix expected = Matrix::Zero(M, N);

  // row
  params.row(0) = Vector::Ones(N);
  expected.row(0) = Vector::Ones(N);
  EXPECT(assert_equal(expected, params.matrix()));

  // col
  params.col(2) = Vector::Ones(M);
  expected.col(2) = Vector::Ones(M);

  EXPECT(assert_equal(expected, params.matrix()));

  // setZero
  params.setZero();
  expected.setZero();
  EXPECT(assert_equal(expected, params.matrix()));
}

//******************************************************************************
TEST(ParameterMatrix, Addition) {
  ParameterMatrix<M> params(Matrix::Ones(M, N));
  ParameterMatrix<M> expected(2 * Matrix::Ones(M, N));

  // Add vector
  EXPECT(assert_equal(expected, params + Vector::Ones(M * N)));
  // Add another ParameterMatrix
  ParameterMatrix<M> actual = params + ParameterMatrix<M>(Matrix::Ones(M, N));
  EXPECT(assert_equal(expected, actual));
}

//******************************************************************************
TEST(ParameterMatrix, Subtraction) {
  ParameterMatrix<M> params(2 * Matrix::Ones(M, N));
  ParameterMatrix<M> expected(Matrix::Ones(M, N));

  // Subtract vector
  EXPECT(assert_equal(expected, params - Vector::Ones(M * N)));
  // Subtract another ParameterMatrix
  ParameterMatrix<M> actual = params - ParameterMatrix<M>(Matrix::Ones(M, N));
  EXPECT(assert_equal(expected, actual));
}

//******************************************************************************
TEST(ParameterMatrix, Multiplication) {
  ParameterMatrix<M> params(Matrix::Ones(M, N));
  Matrix multiplier = 2 * Matrix::Ones(N, 2);
  Matrix expected = 2 * N * Matrix::Ones(M, 2);
  EXPECT(assert_equal(expected, params * multiplier));
}

//******************************************************************************
TEST(ParameterMatrix, VectorSpace) {
  ParameterMatrix<M> params(Matrix::Ones(M, N));
  // vector
  EXPECT(assert_equal(Vector::Ones(M * N), params.vector()));
  // identity
  EXPECT(assert_equal(ParameterMatrix<M>::Identity(),
                      ParameterMatrix<M>(Matrix::Zero(M, 0))));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
