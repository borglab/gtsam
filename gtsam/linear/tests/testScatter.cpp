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
  Matrix3 A01{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  Vector3 b0(1.5, 1.5, 1.5);
  Vector3 s0(1.6, 1.6, 1.6);

  Matrix3 A10{{2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 2.0}};
  Matrix3 A11{{-2.0, 0.0, 0.0}, {0.0, -2.0, 0.0}, {0.0, 0.0, -2.0}};
  Vector3 b1(2.5, 2.5, 2.5);
  Vector3 s1(2.6, 2.6, 2.6);

  Matrix3 A21{{3.0, 0.0, 0.0}, {0.0, 3.0, 0.0}, {0.0, 0.0, 3.0}};
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
  EXPECT_LONGS_EQUAL(3, scatter.at(0).dimension);
  EXPECT_LONGS_EQUAL(3, scatter.at(1).dimension);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
