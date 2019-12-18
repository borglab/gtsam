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

// string g2oFile = findExampleDataFile("toyExample.g2o");
string g2oFile = "/home/jingwu/git/gtsam/examples/Data/toyExample.g2o";
static const ShonanAveraging kShonan(g2oFile);

/* ************************************************************************* */
TEST(ShonanAveraging, buildGraphAt) {
  auto graph = kShonan.buildGraphAt(5);
  EXPECT_LONGS_EQUAL(5, kShonan.nrPoses());
  EXPECT_LONGS_EQUAL(6, graph.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging, checkOptimality) {
  auto Q = kShonan.buildQ();
  EXPECT_LONGS_EQUAL(3 * 5, Q.rows());
  EXPECT_LONGS_EQUAL(3 * 5, Q.cols());
  const Values random = kShonan.initializeRandomlyAt(4);
  auto Lambda = kShonan.computeLambda(random);
  EXPECT_LONGS_EQUAL(3 * 5, Lambda.rows());
  EXPECT_LONGS_EQUAL(3 * 5, Lambda.cols());
  EXPECT_LONGS_EQUAL(45, Lambda.nonZeros());
  auto lambdaMin = kShonan.computeMinEigenValue(random);
  EXPECT_DOUBLES_EQUAL(-5.2964625490657866, lambdaMin,
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
TEST(ShonanAveraging, tryOptimizingAt5) {
  const Values result = kShonan.tryOptimizingAt(5);
  EXPECT_DOUBLES_EQUAL(0, kShonan.costAt(3, result), 1e-3);
  auto lambdaMin = kShonan.computeMinEigenValue(result);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, lambdaMin,
                       1e-4);  // Regression test
}

/* ************************************************************************* */
TEST(ShonanAveraging, runWithRandom) {
  auto result = kShonan.runWithRandom(5);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(result.first), 1e-3);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, result.second,
                       1e-4);  // Regression test
}

/* ************************************************************************* */
TEST(ShonanAveraging, initializeWithDescent) {
  const Values Qstar3 = kShonan.tryOptimizingAt(3);
  Vector minEigenVector;
  kShonan.computeMinEigenValue(Qstar3, &minEigenVector);
  Values initialQ4 = kShonan.initializeWithDescent(4, Qstar3, minEigenVector);
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
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
