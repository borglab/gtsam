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
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <random>

using namespace std;
using namespace gtsam;

template <size_t d>
using Rot = typename std::conditional<d == 2, Rot2, Rot3>::type;

template <size_t d>
using Pose = typename std::conditional<d == 2, Pose2, Pose3>::type;

ShonanAveraging3 fromExampleName(
    const std::string &name,
    ShonanAveraging3::Parameters parameters = ShonanAveraging3::Parameters()) {
  string g2oFile = findExampleDataFile(name);
  return ShonanAveraging3(g2oFile, parameters);
}

static const ShonanAveraging3 kShonan = fromExampleName("toyExample.g2o");

static std::mt19937 kRandomNumberGenerator(42);

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
  EXPECT_LONGS_EQUAL(7, graph.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging3, checkOptimality) {
  const Values randomRotations = kShonan.initializeRandomly(kRandomNumberGenerator);
  Values random = ShonanAveraging3::LiftTo<Rot3>(4, randomRotations);  // lift to 4!
  auto Lambda = kShonan.computeLambda(random);
  EXPECT_LONGS_EQUAL(15, Lambda.rows());
  EXPECT_LONGS_EQUAL(15, Lambda.cols());
  EXPECT_LONGS_EQUAL(45, Lambda.nonZeros());
  auto lambdaMin = kShonan.computeMinEigenValue(random);
  // EXPECT_DOUBLES_EQUAL(-5.2964625490657866, lambdaMin,
  //                      1e-4);  // Regression test
  EXPECT_DOUBLES_EQUAL(-414.87376657555996, lambdaMin,
                       1e-4); // Regression test
  EXPECT(!kShonan.checkOptimality(random));
}

/* ************************************************************************* */
TEST(ShonanAveraging3, checkSubgraph) {
  // Create parameter with solver set to SUBGRAPH
  auto params = ShonanAveragingParameters3(
      gtsam::LevenbergMarquardtParams::CeresDefaults(), "SUBGRAPH");
  ShonanAveraging3::Measurements measurements;

  // The toyExample.g2o has 5 vertices, from 0-4
  // The edges are: 1-2, 2-3, 3-4, 3-1, 1-4, 0-1,
  // which can build a connected graph
  auto subgraphShonan = fromExampleName("toyExample.g2o", params);

  // Create initial random estimation
  Values initial;
  initial = subgraphShonan.initializeRandomly(kRandomNumberGenerator);

  // Run Shonan with SUBGRAPH solver
  auto result = subgraphShonan.run(initial, 3, 3);
  EXPECT_DOUBLES_EQUAL(1e-11, subgraphShonan.cost(result.first), 1e-4);
}

/* ************************************************************************* */
TEST(ShonanAveraging3, tryOptimizingAt3) {
  const Values randomRotations = kShonan.initializeRandomly(kRandomNumberGenerator);
  Values initial = ShonanAveraging3::LiftTo<Rot3>(3, randomRotations);  // convert to SOn
  EXPECT(!kShonan.checkOptimality(initial));
  const Values result = kShonan.tryOptimizingAt(3, initial);
  EXPECT(kShonan.checkOptimality(result));
  auto lambdaMin = kShonan.computeMinEigenValue(result);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, lambdaMin,
                       1e-4); // Regression test
  EXPECT_DOUBLES_EQUAL(0, kShonan.costAt(3, result), 1e-4);
  const Values SO3Values = kShonan.roundSolution(result);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(SO3Values), 1e-4);
}

/* ************************************************************************* */
TEST(ShonanAveraging3, tryOptimizingAt4) {
  const Values randomRotations = kShonan.initializeRandomly(kRandomNumberGenerator);
  Values random = ShonanAveraging3::LiftTo<Rot3>(4, randomRotations);  // lift to 4!
  const Values result = kShonan.tryOptimizingAt(4, random);
  EXPECT(kShonan.checkOptimality(result));
  EXPECT_DOUBLES_EQUAL(0, kShonan.costAt(4, result), 1e-3);
  auto lambdaMin = kShonan.computeMinEigenValue(result);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, lambdaMin,
                       1e-4); // Regression test
  const Values SO3Values = kShonan.roundSolution(result);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(SO3Values), 1e-4);
}

/* ************************************************************************* */
TEST(ShonanAveraging3, TangentVectorValues) {
  Vector9 v;
  v << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Vector expected0(10), expected1(10), expected2(10);
  expected0 << 0, 3, -2, 1, 0, 0, 0, 0, 0, 0;
  expected1 << 0, 6, -5, 4, 0, 0, 0, 0, 0, 0;
  expected2 << 0, 9, -8, 7, 0, 0, 0, 0, 0, 0;
  const VectorValues xi = ShonanAveraging3::TangentVectorValues(5, v);
  EXPECT(assert_equal(expected0, xi[0]));
  EXPECT(assert_equal(expected1, xi[1]));
  EXPECT(assert_equal(expected2, xi[2]));
}

/* ************************************************************************* */
TEST(ShonanAveraging3, LiftTo) {
  auto I = genericValue(Rot3());
  Values initial{{0, I}, {1, I}, {2, I}};
  Values lifted = ShonanAveraging3::LiftTo<Rot3>(5, initial);
  EXPECT(assert_equal(SOn(5), lifted.at<SOn>(0)));
}

/* ************************************************************************* */
TEST(ShonanAveraging3, CheckWithEigen) {
  // control randomness
  static std::mt19937 rng(0);
  Vector descentDirection = Vector::Random(15); // for use below
  const Values randomRotations = kShonan.initializeRandomly(rng);
  Values random = ShonanAveraging3::LiftTo<Rot3>(3, randomRotations);

  // Optimize
  const Values Qstar3 = kShonan.tryOptimizingAt(3, random);

  // Compute Eigenvalue with Spectra solver
  double lambda = kShonan.computeMinEigenValue(Qstar3);

  // Check Eigenvalue with slow Eigen version, converts matrix A to dense matrix!
  const Matrix S = ShonanAveraging3::StiefelElementMatrix(Qstar3);
  auto A = kShonan.computeA(S);
  bool computeEigenvectors = false;
  Eigen::EigenSolver<Matrix> eigenSolver(Matrix(A), computeEigenvectors);
  auto lambdas = eigenSolver.eigenvalues().real();
  double minEigenValue = lambdas(0);
  for (int i = 1; i < lambdas.size(); i++)
      minEigenValue = min(lambdas(i), minEigenValue);

  // Compute Eigenvalue with Accelerated Power method
  double lambdaAP = kShonan.computeMinEigenValueAP(Qstar3);

  // Actual check
  EXPECT_DOUBLES_EQUAL(0, lambda, 1e-11);
  EXPECT_DOUBLES_EQUAL(0, minEigenValue, 1e-11);
  EXPECT_DOUBLES_EQUAL(0, lambdaAP, 1e-11);


  // Construct test descent direction (as minEigenVector is not predictable
  // across platforms, being one from a basically flat 3d- subspace)

  // Check descent
  Values initialQ4 =
      ShonanAveraging3::LiftwithDescent(4, Qstar3, descentDirection);
  EXPECT_LONGS_EQUAL(5, initialQ4.size());

  // TODO(frank): uncomment this regression test: currently not repeatable
  // across platforms.
  // Matrix expected(4, 4);
  // expected << 0.0459224, -0.688689, -0.216922, 0.690321, //
  //     0.92381, 0.191931, 0.255854, 0.21042,              //
  //     -0.376669, 0.301589, 0.687953, 0.542111,           //
  //     -0.0508588, 0.630804, -0.643587, 0.43046;
  // EXPECT(assert_equal(SOn(expected), initialQ4.at<SOn>(0), 1e-5));
}

/* ************************************************************************* */
TEST(ShonanAveraging3, initializeWithDescent) {
  const Values randomRotations = kShonan.initializeRandomly(kRandomNumberGenerator);
  Values random = ShonanAveraging3::LiftTo<Rot3>(3, randomRotations);
  const Values Qstar3 = kShonan.tryOptimizingAt(3, random);
  Vector minEigenVector;
  double lambdaMin = kShonan.computeMinEigenValue(Qstar3, &minEigenVector);
  Values initialQ4 =
      kShonan.initializeWithDescent(4, Qstar3, minEigenVector, lambdaMin);
  EXPECT_LONGS_EQUAL(5, initialQ4.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging3, run) {
  auto initial = kShonan.initializeRandomly(kRandomNumberGenerator); 
  auto result = kShonan.run(initial, 5);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(result.first), 1e-3);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, result.second,
                       1e-4); // Regression test
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

TEST(ShonanAveraging3, runKlaus) {
  using namespace klaus;

  // Initialize a Shonan instance without the Karcher mean
  ShonanAveraging3::Parameters parameters;
  parameters.setKarcherWeight(0);

  // Load 3 pose example taken in Klaus by Shicong
  static const ShonanAveraging3 shonan =
      fromExampleName("Klaus3.g2o", parameters);

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
  auto initial = shonan.initializeRandomly(kRandomNumberGenerator); 
  auto result = shonan.run(initial, 5);
  EXPECT_DOUBLES_EQUAL(0, shonan.cost(result.first), 1e-2);
  EXPECT_DOUBLES_EQUAL(-9.2259161494467889e-05, result.second,
                       1e-4); // Regression

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
TEST(ShonanAveraging3, runKlausKarcher) {
  using namespace klaus;

  // Load 3 pose example taken in Klaus by Shicong
  static const ShonanAveraging3 shonan = fromExampleName("Klaus3.g2o");

  // Run Shonan (with Karcher mean prior)
  auto initial = shonan.initializeRandomly(kRandomNumberGenerator); 
  auto result = shonan.run(initial, 5);
  EXPECT_DOUBLES_EQUAL(0, shonan.cost(result.first), 1e-2);
  EXPECT_DOUBLES_EQUAL(-1.361402670507772e-05, result.second,
                       1e-4); // Regression test

  // Get Shonan solution in new frame R (R for result)
  const Rot3 rR0 = result.first.at<Rot3>(0);
  const Rot3 rR1 = result.first.at<Rot3>(1);
  const Rot3 rR2 = result.first.at<Rot3>(2);

  const Rot3 rRw = rR0 * wR0.inverse();
  EXPECT(assert_equal(rRw * wR1, rR1, 0.1))
  EXPECT(assert_equal(rRw * wR2, rR2, 0.1))
}

/* ************************************************************************* */
TEST(ShonanAveraging2, noisyToyGraph) {
  // Load 2D toy example
  auto lmParams = LevenbergMarquardtParams::CeresDefaults();
  // lmParams.setVerbosityLM("SUMMARY");
  string g2oFile = findExampleDataFile("noisyToyGraph.txt");
  ShonanAveraging2::Parameters parameters(lmParams);
  auto measurements = parseMeasurements<Rot2>(g2oFile);
  ShonanAveraging2 shonan(measurements, parameters);
  EXPECT_LONGS_EQUAL(4, shonan.nrUnknowns());

  // Check graph building
  NonlinearFactorGraph graph = shonan.buildGraphAt(2);
  EXPECT_LONGS_EQUAL(6, graph.size());
  auto initial = shonan.initializeRandomly(kRandomNumberGenerator); 
  auto result = shonan.run(initial, 2);
  EXPECT_DOUBLES_EQUAL(0.0008211, shonan.cost(result.first), 1e-6);
  EXPECT_DOUBLES_EQUAL(0, result.second, 1e-10); // certificate!
}

/* ************************************************************************* */
TEST(ShonanAveraging2, noisyToyGraphWithHuber) {
  // Load 2D toy example
  auto lmParams = LevenbergMarquardtParams::CeresDefaults();
  string g2oFile = findExampleDataFile("noisyToyGraph.txt");
  ShonanAveraging2::Parameters parameters(lmParams);
  auto measurements = parseMeasurements<Rot2>(g2oFile);
  parameters.setUseHuber(true);
  parameters.setCertifyOptimality(false);

  string parameters_print =
      " ShonanAveragingParameters: \n alpha: 0\n beta: 1\n gamma: 0\n "
      "useHuber: 1\n";
  assert_print_equal(parameters_print, parameters);

  ShonanAveraging2 shonan(measurements, parameters);
  EXPECT_LONGS_EQUAL(4, shonan.nrUnknowns());

  // Check graph building
  NonlinearFactorGraph graph = shonan.buildGraphAt(2);
  EXPECT_LONGS_EQUAL(6, graph.size());

  // test that each factor is actually robust
  for (size_t i=0; i<=4; i++) { // note: last is the Gauge factor and is not robust
	  const auto &robust = boost::dynamic_pointer_cast<noiseModel::Robust>(
			  boost::dynamic_pointer_cast<NoiseModelFactor>(graph[i])->noiseModel());
	  EXPECT(robust); // we expect the factors to be use a robust noise model (in particular, Huber)
  }

  // test result
  auto initial = shonan.initializeRandomly(kRandomNumberGenerator);
  auto result = shonan.run(initial, 2,2);
  EXPECT_DOUBLES_EQUAL(0.0008211, shonan.cost(result.first), 1e-6);
  EXPECT_DOUBLES_EQUAL(0, result.second, 1e-10); // certificate!
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
  for (auto i : {0,1,2}) {
    const auto& m = shonan.measurement(i);
    auto isotropic =
        boost::static_pointer_cast<noiseModel::Isotropic>(m.noiseModel());
    CHECK(isotropic != nullptr);
    EXPECT_LONGS_EQUAL(3, isotropic->dim());
    EXPECT_DOUBLES_EQUAL(0.2, isotropic->sigma(), 1e-9);
  }
  auto I = genericValue(Rot3());
  Values initial{{0, I}, {1, I}, {2, I}};
  EXPECT_DOUBLES_EQUAL(3.0756, shonan.cost(initial), 1e-4);
  auto result = shonan.run(initial, 3, 3);
  EXPECT_DOUBLES_EQUAL(0.0015, shonan.cost(result.first), 1e-4);
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
