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
// Check iterators of empty hybrid factor.
TEST(HybridNonlinearFactor, Constructor) {
  HybridNonlinearFactor factor;
  HybridNonlinearFactor::const_iterator const_it = factor.begin();
  CHECK(const_it == factor.end());
  HybridNonlinearFactor::iterator it = factor.begin();
  CHECK(it == factor.end());
}
/* ************************************************************************* */
namespace test_constructor {
DiscreteKey m1(1, 2);
double between0 = 0.0;
double between1 = 1.0;

Vector1 sigmas = Vector1(1.0);
auto model = noiseModel::Diagonal::Sigmas(sigmas, false);

auto f0 = std::make_shared<BetweenFactor<double>>(X(1), X(2), between0, model);
auto f1 = std::make_shared<BetweenFactor<double>>(X(1), X(2), between1, model);
}  // namespace test_constructor

/* ************************************************************************* */
// Test simple to complex constructors...
TEST(HybridGaussianFactor, ConstructorVariants) {
  using namespace test_constructor;
  HybridNonlinearFactor fromFactors({X(1), X(2)}, m1, {f0, f1});

  std::vector<NonlinearFactorValuePair> pairs{{f0, 0.0}, {f1, 0.0}};
  HybridNonlinearFactor fromPairs({X(1), X(2)}, m1, pairs);
  assert_equal(fromFactors, fromPairs);

  HybridNonlinearFactor::FactorValuePairs decisionTree({m1}, pairs);
  HybridNonlinearFactor fromDecisionTree({X(1), X(2)}, {m1}, decisionTree);
  assert_equal(fromDecisionTree, fromPairs);
}

/* ************************************************************************* */
// Test .print() output.
TEST(HybridNonlinearFactor, Printing) {
  using namespace test_constructor;
  HybridNonlinearFactor hybridFactor({X(1), X(2)}, {m1}, {f0, f1});

  std::string expected =
      R"(Hybrid [x1 x2; 1]
HybridNonlinearFactor
 Choice(1) 
 0 Leaf Nonlinear factor on 2 keys
 1 Leaf Nonlinear factor on 2 keys
)";
  EXPECT(assert_print_equal(expected, hybridFactor));
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
  return HybridNonlinearFactor({X(1), X(2)}, m1, {f0, f1});
}

/* ************************************************************************* */
// Test the error of the HybridNonlinearFactor
TEST(HybridNonlinearFactor, Error) {
  auto hybridFactor = getHybridNonlinearFactor();

  Values continuousValues;
  continuousValues.insert<double>(X(1), 0);
  continuousValues.insert<double>(X(2), 1);

  AlgebraicDecisionTree<Key> error_tree =
      hybridFactor.errorTree(continuousValues);

  DiscreteKey m1(1, 2);
  std::vector<DiscreteKey> discrete_keys = {m1};
  std::vector<double> errors = {0.5, 0};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, errors);

  EXPECT(assert_equal(expected_error, error_tree));
}

/* ************************************************************************* */
// Test dim of the HybridNonlinearFactor
TEST(HybridNonlinearFactor, Dim) {
  auto hybridFactor = getHybridNonlinearFactor();
  EXPECT_LONGS_EQUAL(1, hybridFactor.dim());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
