/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testScatter.cpp
 * @author  Frank Dellaert
 * @date    June, 2015
 */

#include <gtsam/linear/Scatter.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;

/* ************************************************************************* */
TEST(HessianFactor, CombineAndEliminate) {
  static const size_t m = 3, n = 3;
  Matrix A01 =
      (Matrix(m, n) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0).finished();
  Vector3 b0(1.5, 1.5, 1.5);
  Vector3 s0(1.6, 1.6, 1.6);

  Matrix A10 =
      (Matrix(m, n) << 2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0).finished();
  Matrix A11 = (Matrix(m, n) << -2.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, -2.0)
                   .finished();
  Vector3 b1(2.5, 2.5, 2.5);
  Vector3 s1(2.6, 2.6, 2.6);

  Matrix A21 =
      (Matrix(m, n) << 3.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0).finished();
  Vector3 b2(3.5, 3.5, 3.5);
  Vector3 s2(3.6, 3.6, 3.6);

  GaussianFactorGraph gfg;
  gfg.add(X(1), A01, b0, noiseModel::Diagonal::Sigmas(s0, true));
  gfg.add(X(0), A10, X(1), A11, b1, noiseModel::Diagonal::Sigmas(s1, true));
  gfg.add(X(1), A21, b2, noiseModel::Diagonal::Sigmas(s2, true));

  Scatter scatter(gfg);
  EXPECT_LONGS_EQUAL(2, scatter.size());
  EXPECT(assert_equal(X(0), scatter.at(0).key));
  EXPECT(assert_equal(X(1), scatter.at(1).key));
  EXPECT_LONGS_EQUAL(n, scatter.at(0).dimension);
  EXPECT_LONGS_EQUAL(n, scatter.at(1).dimension);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
