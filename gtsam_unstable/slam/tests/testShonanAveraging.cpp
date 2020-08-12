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

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/slam/ShonanAveraging.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <algorithm>
#include <iostream>
#include <map>

using namespace std;
using namespace gtsam;

// Convert Pose3 constraints to Rot3 constraints
ShonanAveraging3::Factors
betweenFactorRot3s(const BetweenFactorPose3s &factors3) {
  ShonanAveraging3::Factors result;
  result.reserve(factors3.size());
  for (auto f : factors3) {
    result.emplace_back(
        new BetweenFactor<Rot3>(f->key1(), f->key2(), f->measured().rotation(),
                                ConvertNoiseModel(f->noiseModel(), 3)));
  }
  return result;
}

ShonanAveraging3 fromExampleName(
    const std::string &name,
    ShonanAveraging3::Parameters parameters = ShonanAveraging3::Parameters()) {
  string g2oFile = findExampleDataFile(name);
  auto factors3 = parse3DFactors(g2oFile);
  auto factors = betweenFactorRot3s(factors3);
  return ShonanAveraging3(factors, parameters);
}

static const ShonanAveraging3 kShonan = fromExampleName("toyExample.g2o");

/* ************************************************************************* */
TEST(ShonanAveraging3, checkConstructor) {
  EXPECT_LONGS_EQUAL(5, kShonan.nrUnknowns());

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
TEST(ShonanAveraging3, buildGraphAt) {
  auto graph = kShonan.buildGraphAt(5);
  EXPECT_LONGS_EQUAL(6, graph.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging3, checkOptimality) {
  const Values random = kShonan.initializeRandomlyAt(4);
  auto Lambda = kShonan.computeLambda(random);
  EXPECT_LONGS_EQUAL(15, Lambda.rows());
  EXPECT_LONGS_EQUAL(15, Lambda.cols());
  EXPECT_LONGS_EQUAL(45, Lambda.nonZeros());
  auto lambdaMin = kShonan.computeMinEigenValue(random);
  // EXPECT_DOUBLES_EQUAL(-5.2964625490657866, lambdaMin,
  //                      1e-4);  // Regression test
  EXPECT_DOUBLES_EQUAL(-330.13332247232307, lambdaMin,
                       1e-4);  // Regression test
  EXPECT(!kShonan.checkOptimality(random));
}

/* ************************************************************************* */
TEST(ShonanAveraging3, tryOptimizingAt3) {
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
TEST(ShonanAveraging3, tryOptimizingAt4) {
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
TEST(ShonanAveraging3, runWithRandom) {
  auto result = kShonan.runWithRandom(5);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(result.first), 1e-3);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, result.second,
                       1e-4);  // Regression test
}

/* ************************************************************************* */
TEST(ShonanAveraging3, MakeATangentVector) {
  Vector9 v;
  v << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Matrix expected(5, 5);
  expected << 0, 0, 0, 0, -4,  //
      0, 0, 0, 0, -5,          //
      0, 0, 0, 0, -6,          //
      0, 0, 0, 0, 0,           //
      4, 5, 6, 0, 0;
  const Vector xi_1 = ShonanAveraging3::MakeATangentVector(5, v, 1);
  const auto actual = SOn::Hat(xi_1);
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(ShonanAveraging3, LiftTo) {
  auto I = genericValue(Rot3());
  Values initial {{0, I}, {1, I}, {2, I}};
  Values lifted = ShonanAveraging3::LiftTo<Rot3>(5, initial);
  EXPECT(assert_equal(SOn(5), lifted.at<SOn>(0)));
}

/* ************************************************************************* */
TEST(ShonanAveraging3, LiftwithDescent) {
  const Values Qstar3 = kShonan.tryOptimizingAt(3);
  Vector minEigenVector;
  kShonan.computeMinEigenValue(Qstar3, &minEigenVector);
  Values initialQ4 =
      ShonanAveraging3::LiftwithDescent(4, Qstar3, minEigenVector);
  EXPECT_LONGS_EQUAL(5, initialQ4.size());
  Matrix expected(4, 4);
  expected << 0.65649, -0.556278, -0.509486, -0.000102, //
      0.0460064, -0.596212, 0.710175, 0.371573,             //
      -0.737652, -0.460499, -0.447739, 0.208182,            //
      0.15091, 0.350752, -0.188693, 0.904762;
  EXPECT(assert_equal(SOn(expected), initialQ4.at<SOn>(0), 1e-5));
}

/* ************************************************************************* */
TEST(ShonanAveraging3, initializeWithDescent) {
  const Values Qstar3 = kShonan.tryOptimizingAt(3);
  Vector minEigenVector;
  double lambdaMin = kShonan.computeMinEigenValue(Qstar3, &minEigenVector);
  Values initialQ4 =
      kShonan.initializeWithDescent(4, Qstar3, minEigenVector, lambdaMin);
  EXPECT_LONGS_EQUAL(5, initialQ4.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging3, runWithDescent) {
  auto result = kShonan.runWithDescent(5);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(result.first), 1e-3);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, result.second,
                       1e-4);  // Regression test
}

/* ************************************************************************* */
namespace klaus {
// The data in the file is the Colmap solution
const Rot3 wR0(0.9992281076190063, -0.02676080288219576, -0.024497002638379624,
               -0.015064701622500615);
const Rot3 wR1(0.998239108728862, -0.049543805396343954, -0.03232420352077356,
               -0.004386230477751116);
const Rot3 wR2(0.9925378735259738, -0.07993768981394891, 0.0825062894866454,
               -0.04088089479075661);
} // namespace klaus

TEST(ShonanAveraging3, runWithRandomKlaus) {
  using namespace klaus;

  // Initialize a Shonan instance without the Karcher mean
  ShonanAveraging3::Parameters parameters;
  parameters.setKarcherWeight(0);

  // Load 3 pose example taken in Klaus by Shicong
  static const ShonanAveraging3 shonan = fromExampleName("Klaus3.g2o", parameters);

  // Check nr poses
  EXPECT_LONGS_EQUAL(3, shonan.nrUnknowns());

  // Colmap uses the Y-down vision frame, and the first 3 rotations are close to
  // identity. We check that below. Note tolerance is quite high.
  static const Rot3 identity;
  EXPECT(assert_equal(identity, wR0, 0.2));
  EXPECT(assert_equal(identity, wR1, 0.2));
  EXPECT(assert_equal(identity, wR2, 0.2));

  // Get measurements
  const Rot3 R01 = shonan.measured(0);
  const Rot3 R12 = shonan.measured(1);
  const Rot3 R02 = shonan.measured(2);

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
  const Rot3 rR0 = result.first.at<Rot3>(0);
  const Rot3 rR1 = result.first.at<Rot3>(1);
  const Rot3 rR2 = result.first.at<Rot3>(2);

  // rR0 = rRw * wR0 => rRw = rR0 * wR0.inverse()
  // rR1 = rRw * wR1
  // rR2 = rRw * wR2

  const Rot3 rRw = rR0 * wR0.inverse();
  EXPECT(assert_equal(rRw * wR1, rR1, 0.1))
  EXPECT(assert_equal(rRw * wR2, rR2, 0.1))
}

/* ************************************************************************* */
TEST(ShonanAveraging3, runWithRandomKlausKarcher) {
  using namespace klaus;
  
  // Load 3 pose example taken in Klaus by Shicong
  static const ShonanAveraging3 shonan = fromExampleName("Klaus3.g2o");

  // Run Shonan (with Karcher mean prior)
  auto result = shonan.runWithRandom(5);
  EXPECT_DOUBLES_EQUAL(0, shonan.cost(result.first), 1e-2);
  EXPECT_DOUBLES_EQUAL(-1.361402670507772e-05, result.second,
                       1e-4);  // Regression test

  // Get Shonan solution in new frame R (R for result)
  const Rot3 rR0 = result.first.at<Rot3>(0);
  const Rot3 rR1 = result.first.at<Rot3>(1);
  const Rot3 rR2 = result.first.at<Rot3>(2);

  const Rot3 rRw = rR0 * wR0.inverse();
  EXPECT(assert_equal(rRw * wR1, rR1, 0.1))
  EXPECT(assert_equal(rRw * wR2, rR2, 0.1))
}

/* ************************************************************************* */
TEST(ShonanAveraging2, runWithRandomKlausKarcher) {
}

/* ************************************************************************* */
// Test alpha/beta/gamma prior weighting.
TEST(ShonanAveraging3, PriorWeights) {
  auto lmParams = LevenbergMarquardtParams::CeresDefaults();
  ShonanAveraging3::Parameters params(lmParams);
  EXPECT_DOUBLES_EQUAL(0, params.alpha, 1e-9);
  EXPECT_DOUBLES_EQUAL(1, params.beta, 1e-9);
  EXPECT_DOUBLES_EQUAL(0, params.gamma, 1e-9);
  double alpha = 100.0, beta = 200.0, gamma = 300.0;
  params.setAnchorWeight(alpha);
  params.setKarcherWeight(beta);
  params.setGaugesWeight(gamma);
  EXPECT_DOUBLES_EQUAL(alpha, params.alpha, 1e-9);
  EXPECT_DOUBLES_EQUAL(beta, params.beta, 1e-9);
  EXPECT_DOUBLES_EQUAL(gamma, params.gamma, 1e-9);
  params.setKarcherWeight(0);
  static const ShonanAveraging3 shonan = fromExampleName("Klaus3.g2o", params);
  auto I = genericValue(Rot3());
  Values initial {{0, I}, {1, I}, {2, I}};
  EXPECT_DOUBLES_EQUAL(3.0756, shonan.cost(initial), 1e-4);
  auto result = shonan.runWithDescent(3, 3, initial);
  EXPECT_DOUBLES_EQUAL(0.0015, shonan.cost(result.first), 1e-4);
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
