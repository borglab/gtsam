/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testShonanGaugeFactor.cpp
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Unit tests for ShonanGaugeFactor class
 */

#include <gtsam/sfm/ShonanGaugeFactor.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <map>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Check dimensions of all low-dim GaugeFactors
TEST(ShonanAveraging, GaugeFactorLows) {
  constexpr Key key(123);
  EXPECT_LONGS_EQUAL(0, ShonanGaugeFactor(key, 2, 2).dim());
  EXPECT_LONGS_EQUAL(0, ShonanGaugeFactor(key, 3, 2).dim());
  EXPECT_LONGS_EQUAL(1, ShonanGaugeFactor(key, 4, 2).dim()); // SO(4-2) -> 1
  EXPECT_LONGS_EQUAL(3, ShonanGaugeFactor(key, 5, 2).dim()); // SO(5-2) -> 3

  EXPECT_LONGS_EQUAL(0, ShonanGaugeFactor(key, 3, 3).dim());
  EXPECT_LONGS_EQUAL(0, ShonanGaugeFactor(key, 4, 3).dim());
  EXPECT_LONGS_EQUAL(1, ShonanGaugeFactor(key, 5, 3).dim()); // SO(5-3) -> 1
}

/* ************************************************************************* */
// Check ShonanGaugeFactor for SO(6)
TEST(ShonanAveraging, GaugeFactorSO6) {
  constexpr Key key(666);
  ShonanGaugeFactor factor(key, 6); // For SO(6)
  Matrix A = Matrix::Zero(3, 15);   // SO(6-3) = SO(3) == 3-dimensional gauge
  A(0, 0) = 1; // first 2 of 6^th skew column, which has 5 non-zero entries
  A(1, 1) = 1; // then we skip 3 tangent dimensions
  A(2, 5) = 1; // first of 5th skew colum, which has 4 non-zero entries above
               // diagonal.
  JacobianFactor linearized(key, A, Vector::Zero(3));
  Values values;
  EXPECT_LONGS_EQUAL(3, factor.dim());
  EXPECT(assert_equal(linearized, *std::dynamic_pointer_cast<JacobianFactor>(
                                      factor.linearize(values))));
}

/* ************************************************************************* */
// Check ShonanGaugeFactor for SO(7)
TEST(ShonanAveraging, GaugeFactorSO7) {
  constexpr Key key(777);
  ShonanGaugeFactor factor(key, 7); // For SO(7)
  Matrix A = Matrix::Zero(6, 21);   // SO(7-3) = SO(4) == 6-dimensional gauge
  A(0, 0) = 1; // first 3 of 7^th skew column, which has 6 non-zero entries
  A(1, 1) = 1;
  A(2, 2) = 1;  // then we skip 3 tangent dimensions
  A(3, 6) = 1;  // first 2 of 6^th skew column, which has 5 non-zero entries
  A(4, 7) = 1;  // then we skip 3 tangent dimensions
  A(5, 11) = 1; // first of 5th skew colum, which has 4 non-zero entries above
                // diagonal.
  JacobianFactor linearized(key, A, Vector::Zero(6));
  Values values;
  EXPECT_LONGS_EQUAL(6, factor.dim());
  EXPECT(assert_equal(linearized, *std::dynamic_pointer_cast<JacobianFactor>(
                                      factor.linearize(values))));
}

/* ************************************************************************* */
// Check ShonanGaugeFactor for SO(6), with base SO(2)
TEST(ShonanAveraging, GaugeFactorSO6over2) {
  constexpr Key key(602);
  double gamma = 4;
  ShonanGaugeFactor factor(key, 6, 2, gamma); // For SO(6), base SO(2)
  Matrix A = Matrix::Zero(6, 15); // SO(6-2) = SO(4) == 6-dimensional gauge
  A(0, 0) = 2; // first 3 of 6^th skew column, which has 5 non-zero entries
  A(1, 1) = 2;
  A(2, 2) = 2; // then we skip only 2 tangent dimensions
  A(3, 5) = 2; // first 2 of 5^th skew column, which has 4 non-zero entries
  A(4, 6) = 2; // then we skip only 2 tangent dimensions
  A(5, 9) = 2; // first of 4th skew colum, which has 3 non-zero entries above
               // diagonal.
  JacobianFactor linearized(key, A, Vector::Zero(6));
  Values values;
  EXPECT_LONGS_EQUAL(6, factor.dim());
  EXPECT(assert_equal(linearized, *std::dynamic_pointer_cast<JacobianFactor>(
                                      factor.linearize(values))));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
