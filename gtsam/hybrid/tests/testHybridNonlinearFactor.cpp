/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridNonlinearFactor.cpp
 * @brief   Unit tests for HybridNonlinearFactor
 * @author  Varun Agrawal
 * @date    October 2022
 */

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ************************************************************************* */
// Check iterators of empty mixture.
TEST(HybridNonlinearFactor, Constructor) {
  HybridNonlinearFactor factor;
  HybridNonlinearFactor::const_iterator const_it = factor.begin();
  CHECK(const_it == factor.end());
  HybridNonlinearFactor::iterator it = factor.begin();
  CHECK(it == factor.end());
}

/* ************************************************************************* */
// Test .print() output.
TEST(HybridNonlinearFactor, Printing) {
  DiscreteKey m1(1, 2);
  double between0 = 0.0;
  double between1 = 1.0;

  Vector1 sigmas = Vector1(1.0);
  auto model = noiseModel::Diagonal::Sigmas(sigmas, false);

  auto f0 =
      std::make_shared<BetweenFactor<double>>(X(1), X(2), between0, model);
  auto f1 =
      std::make_shared<BetweenFactor<double>>(X(1), X(2), between1, model);
  std::vector<std::pair<NonlinearFactor::shared_ptr, double>> factors{
      {f0, 0.0}, {f1, 0.0}};

  HybridNonlinearFactor mixtureFactor({X(1), X(2)}, {m1}, factors);

  std::string expected =
      R"(Hybrid [x1 x2; 1]
HybridNonlinearFactor
 Choice(1) 
 0 Leaf Nonlinear factor on 2 keys
 1 Leaf Nonlinear factor on 2 keys
)";
  EXPECT(assert_print_equal(expected, mixtureFactor));
}

/* ************************************************************************* */
static HybridNonlinearFactor getHybridNonlinearFactor() {
  DiscreteKey m1(1, 2);

  double between0 = 0.0;
  double between1 = 1.0;

  Vector1 sigmas = Vector1(1.0);
  auto model = noiseModel::Diagonal::Sigmas(sigmas, false);

  auto f0 =
      std::make_shared<BetweenFactor<double>>(X(1), X(2), between0, model);
  auto f1 =
      std::make_shared<BetweenFactor<double>>(X(1), X(2), between1, model);
  std::vector<std::pair<NonlinearFactor::shared_ptr, double>> factors{
      {f0, 0.0}, {f1, 0.0}};

  return HybridNonlinearFactor({X(1), X(2)}, {m1}, factors);
}

/* ************************************************************************* */
// Test the error of the HybridNonlinearFactor
TEST(HybridNonlinearFactor, Error) {
  auto mixtureFactor = getHybridNonlinearFactor();

  Values continuousValues;
  continuousValues.insert<double>(X(1), 0);
  continuousValues.insert<double>(X(2), 1);

  AlgebraicDecisionTree<Key> error_tree =
      mixtureFactor.errorTree(continuousValues);

  DiscreteKey m1(1, 2);
  std::vector<DiscreteKey> discrete_keys = {m1};
  std::vector<double> errors = {0.5, 0};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, errors);

  EXPECT(assert_equal(expected_error, error_tree));
}

/* ************************************************************************* */
// Test dim of the HybridNonlinearFactor
TEST(HybridNonlinearFactor, Dim) {
  auto mixtureFactor = getHybridNonlinearFactor();
  EXPECT_LONGS_EQUAL(1, mixtureFactor.dim());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
