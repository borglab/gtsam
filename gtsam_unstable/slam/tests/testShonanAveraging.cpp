/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testShonanAveraging.cpp
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Unit tests for Shonan Averaging algorithm
 */

#include <gtsam_unstable/slam/ShonanAveraging.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <map>

using namespace std;
using namespace gtsam;

string g2oFile = findExampleDataFile("toyExample.g2o");
static const ShonanAveraging kShonan(g2oFile);

/* ************************************************************************* */
TEST(ShonanAveraging, checkConstructor) {
  EXPECT_LONGS_EQUAL(5, kShonan.nrPoses());

  EXPECT_LONGS_EQUAL(15, kShonan.D().rows());
  EXPECT_LONGS_EQUAL(15, kShonan.D().cols());
  auto D = kShonan.denseD();
  EXPECT_LONGS_EQUAL(15, D.rows());
  EXPECT_LONGS_EQUAL(15, D.cols());

  EXPECT_LONGS_EQUAL(15, kShonan.Q().rows());
  EXPECT_LONGS_EQUAL(15, kShonan.Q().cols());
  auto Q = kShonan.denseQ();
  EXPECT_LONGS_EQUAL(15, Q.rows());
  EXPECT_LONGS_EQUAL(15, Q.cols());

  EXPECT_LONGS_EQUAL(15, kShonan.L().rows());
  EXPECT_LONGS_EQUAL(15, kShonan.L().cols());
  auto L = kShonan.denseL();
  EXPECT_LONGS_EQUAL(15, L.rows());
  EXPECT_LONGS_EQUAL(15, L.cols());
}

/* ************************************************************************* */
TEST(ShonanAveraging, buildGraphAt) {
  auto graph = kShonan.buildGraphAt(5);
  EXPECT_LONGS_EQUAL(6, graph.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging, checkOptimality) {
  const Values random = kShonan.initializeRandomlyAt(4);
  auto Lambda = kShonan.computeLambda(random);
  EXPECT_LONGS_EQUAL(15, Lambda.rows());
  EXPECT_LONGS_EQUAL(15, Lambda.cols());
  EXPECT_LONGS_EQUAL(45, Lambda.nonZeros());
  auto lambdaMin = kShonan.computeMinEigenValue(random);
  // EXPECT_DOUBLES_EQUAL(-5.2964625490657866, lambdaMin,
  //                      1e-4);  // Regression test
  EXPECT_DOUBLES_EQUAL(-3.3013346196100195, lambdaMin,
                       1e-4);  // Regression test
  EXPECT(!kShonan.checkOptimality(random));
}

/* ************************************************************************* */
TEST(ShonanAveraging, tryOptimizingAt3) {
  const Values initial = kShonan.initializeRandomlyAt(3);
  EXPECT(!kShonan.checkOptimality(initial));
  const Values result = kShonan.tryOptimizingAt(3, initial);
  EXPECT(kShonan.checkOptimality(result));
  auto lambdaMin = kShonan.computeMinEigenValue(result);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, lambdaMin,
                       1e-4);  // Regression test
  EXPECT_DOUBLES_EQUAL(0, kShonan.costAt(3, result), 1e-4);
  const Values SO3Values = kShonan.roundSolution(result);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(SO3Values), 1e-4);
}

/* ************************************************************************* */
TEST(ShonanAveraging, tryOptimizingAt4) {
  const Values result = kShonan.tryOptimizingAt(4);
  EXPECT(kShonan.checkOptimality(result));
  EXPECT_DOUBLES_EQUAL(0, kShonan.costAt(4, result), 1e-3);
  auto lambdaMin = kShonan.computeMinEigenValue(result);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, lambdaMin,
                       1e-4);  // Regression test
  const Values SO3Values = kShonan.roundSolution(result);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(SO3Values), 1e-4);
}

/* ************************************************************************* */
// TEST(ShonanAveraging, tryOptimizingAt5) {
//   const Values result = kShonan.tryOptimizingAt(5);
//   EXPECT_DOUBLES_EQUAL(0, kShonan.costAt(5, result), 1e-3);
//   auto lambdaMin = kShonan.computeMinEigenValue(result);
//   EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, lambdaMin,
//                        1e-4);  // Regression test
// }

/* ************************************************************************* */
TEST(ShonanAveraging, runWithRandom) {
  auto result = kShonan.runWithRandom(5);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(result.first), 1e-3);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, result.second,
                       1e-4);  // Regression test
}

/* ************************************************************************* */
TEST(ShonanAveraging, MakeATangentVector) {
  Vector9 v;
  v << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Matrix expected(5, 5);
  expected << 0, 0, 0, 0, -4,  //
      0, 0, 0, 0, -5,          //
      0, 0, 0, 0, -6,          //
      0, 0, 0, 0, 0,           //
      4, 5, 6, 0, 0;
  const Vector xi_1 = ShonanAveraging::MakeATangentVector(5, v, 1);
  const auto actual = SOn::Hat(xi_1);
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(ShonanAveraging, dimensionLifting) {
  const Values Qstar3 = kShonan.tryOptimizingAt(3);
  Vector minEigenVector;
  kShonan.computeMinEigenValue(Qstar3, &minEigenVector);
  Values initialQ4 = kShonan.dimensionLifting(4, Qstar3, minEigenVector);
  EXPECT_LONGS_EQUAL(5, initialQ4.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging, initializeWithDescent) {
  const Values Qstar3 = kShonan.tryOptimizingAt(3);
  Vector minEigenVector;
  double lambdaMin = kShonan.computeMinEigenValue(Qstar3, &minEigenVector);
  Values initialQ4 =
      kShonan.initializeWithDescent(4, Qstar3, minEigenVector, lambdaMin);
  EXPECT_LONGS_EQUAL(5, initialQ4.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging, runWithDescent) {
  auto result = kShonan.runWithDescent(5);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(result.first), 1e-3);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, result.second,
                       1e-4);  // Regression test
}

/* ************************************************************************* */
TEST(ShonanAveraging, runWithRandomKlaus) {
  // Load 3 pose example taken in Klaus by Shicong
  string g2oFile = findExampleDataFile("Klaus3.g2o");

  // Initialize a Shonan instance without the Karcher mean
  ShonanAveragingParameters parameters;
  parameters.setKarcher(false);
  static const ShonanAveraging shonan(g2oFile, parameters);

  // Check nr poses
  EXPECT_LONGS_EQUAL(3, shonan.nrPoses());

  // The data in the file is the Colmap solution
  const auto &poses = shonan.poses();
  const Rot3 wR0 = poses.at(0).rotation();
  const Rot3 wR1 = poses.at(1).rotation();
  const Rot3 wR2 = poses.at(2).rotation();

  // Colmap uses the Y-down vision frame, and the first 3 rotations are close to
  // identity. We check that below. Note tolerance is quite high.
  static const Rot3 identity;
  EXPECT(assert_equal(identity, wR0, 0.2));
  EXPECT(assert_equal(identity, wR1, 0.2));
  EXPECT(assert_equal(identity, wR2, 0.2));

  // Get measurements
  const Rot3 R01 = shonan.measured(0).rotation();
  const Rot3 R12 = shonan.measured(1).rotation();
  const Rot3 R02 = shonan.measured(2).rotation();

  // Regression test to make sure data did not change.
  EXPECT(assert_equal(Rot3(0.9995433591728293, -0.022048798853273946,
                           -0.01796327847857683, 0.010210006313668573),
                      R01));

  // Check Colmap solution agrees OK with relative rotation measurements.
  EXPECT(assert_equal(R01, wR0.between(wR1), 0.1));
  EXPECT(assert_equal(R12, wR1.between(wR2), 0.1));
  EXPECT(assert_equal(R02, wR0.between(wR2), 0.1));

  // Run Shonan (with prior on first rotation)
  auto result = shonan.runWithRandom(5);
  EXPECT_DOUBLES_EQUAL(0, shonan.cost(result.first), 1e-2);
  EXPECT_DOUBLES_EQUAL(-9.2259161494467889e-05, result.second,
                       1e-4);  // Regression

  // Get Shonan solution in new frame R (R for result)
  const Rot3 rR0 = Rot3(result.first.at<SO3>(0));
  const Rot3 rR1 = Rot3(result.first.at<SO3>(1));
  const Rot3 rR2 = Rot3(result.first.at<SO3>(2));

  // rR0 = rRw * wR0 => rRw = rR0 * wR0.inverse()
  // rR1 = rRw * wR1
  // rR2 = rRw * wR2

  const Rot3 rRw = rR0 * wR0.inverse();
  EXPECT(assert_equal(rRw * wR1, rR1, 0.1))
  EXPECT(assert_equal(rRw * wR2, rR2, 0.1))
}

/* ************************************************************************* */
TEST(ShonanAveraging, runWithRandomKlausKarcher) {
  // Load 3 pose example taken in Klaus by Shicong
  string g2oFile = findExampleDataFile("Klaus3.g2o");

  // Initialize a Shonan instance with the Karcher mean (default true)
  static const ShonanAveraging shonan(g2oFile);
  const auto &poses = shonan.poses();
  const Rot3 wR0 = poses.at(0).rotation();
  const Rot3 wR1 = poses.at(1).rotation();
  const Rot3 wR2 = poses.at(2).rotation();

  // Run Shonan (with Karcher mean prior)
  auto result = shonan.runWithRandom(5);
  EXPECT_DOUBLES_EQUAL(0, shonan.cost(result.first), 1e-2);
  EXPECT_DOUBLES_EQUAL(-1.361402670507772e-05, result.second,
                       1e-4);  // Regression test

  // Get Shonan solution in new frame R (R for result)
  const Rot3 rR0 = Rot3(result.first.at<SO3>(0));
  const Rot3 rR1 = Rot3(result.first.at<SO3>(1));
  const Rot3 rR2 = Rot3(result.first.at<SO3>(2));

  const Rot3 rRw = rR0 * wR0.inverse();
  EXPECT(assert_equal(rRw * wR1, rR1, 0.1))
  EXPECT(assert_equal(rRw * wR2, rR2, 0.1))
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
