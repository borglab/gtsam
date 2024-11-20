/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridGaussianFactor.cpp
 * @brief   Unit tests for HybridGaussianFactor
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

#include <memory>

using namespace std;
using namespace gtsam;
using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

/* ************************************************************************* */
// Check iterators of empty hybrid factor.
TEST(HybridGaussianFactor, Constructor) {
  HybridGaussianFactor factor;
  HybridGaussianFactor::const_iterator const_it = factor.begin();
  CHECK(const_it == factor.end());
  HybridGaussianFactor::iterator it = factor.begin();
  CHECK(it == factor.end());
}

/* ************************************************************************* */
namespace test_constructor {
DiscreteKey m1(1, 2);

auto A1 = Matrix::Zero(2, 1);
auto A2 = Matrix::Zero(2, 2);
auto b = Matrix::Zero(2, 1);

auto f10 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
auto f11 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
}  // namespace test_constructor

/* ************************************************************************* */
// Test simple to complex constructors...
TEST(HybridGaussianFactor, ConstructorVariants) {
  using namespace test_constructor;
  HybridGaussianFactor fromFactors(m1, {f10, f11});

  std::vector<GaussianFactorValuePair> pairs{{f10, 0.0}, {f11, 0.0}};
  HybridGaussianFactor fromPairs(m1, pairs);
  assert_equal(fromFactors, fromPairs);

  HybridGaussianFactor::FactorValuePairs decisionTree({m1}, pairs);
  HybridGaussianFactor fromDecisionTree({m1}, decisionTree);
  assert_equal(fromDecisionTree, fromPairs);
}

/* ************************************************************************* */
TEST(HybridGaussianFactor, Keys) {
  using namespace test_constructor;
  HybridGaussianFactor hybridFactorA(m1, {f10, f11});
  // Check the number of keys matches what we expect
  EXPECT_LONGS_EQUAL(3, hybridFactorA.keys().size());
  EXPECT_LONGS_EQUAL(2, hybridFactorA.continuousKeys().size());
  EXPECT_LONGS_EQUAL(1, hybridFactorA.discreteKeys().size());

  DiscreteKey m2(2, 3);
  auto A3 = Matrix::Zero(2, 3);
  auto f20 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  auto f21 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  auto f22 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  HybridGaussianFactor hybridFactorB(m2, {f20, f21, f22});

  // Check the number of keys matches what we expect
  EXPECT_LONGS_EQUAL(3, hybridFactorB.keys().size());
  EXPECT_LONGS_EQUAL(2, hybridFactorB.continuousKeys().size());
  EXPECT_LONGS_EQUAL(1, hybridFactorB.discreteKeys().size());
}

/* ************************************************************************* */
TEST(HybridGaussianFactor, Printing) {
  using namespace test_constructor;
  HybridGaussianFactor hybridFactor(m1, {f10, f11});

  std::string expected =
      R"(Hybrid [x1 x2; 1]{
 Choice(1) 
 0 Leaf :
  A[x1] = [
	0;
	0
]
  A[x2] = [
	0, 0;
	0, 0
]
  b = [ 0 0 ]
  No noise model
scalar: 0

 1 Leaf :
  A[x1] = [
	0;
	0
]
  A[x2] = [
	0, 0;
	0, 0
]
  b = [ 0 0 ]
  No noise model
scalar: 0

}
)";
  EXPECT(assert_print_equal(expected, hybridFactor));
}

/* ************************************************************************* */
TEST(HybridGaussianFactor, HybridGaussianConditional) {
  DiscreteKeys dKeys;
  dKeys.emplace_back(M(0), 2);
  dKeys.emplace_back(M(1), 2);

  auto gaussians = std::make_shared<GaussianConditional>();
  HybridGaussianConditional::Conditionals conditionals(gaussians);
  HybridGaussianConditional gm(dKeys, conditionals);

  EXPECT_LONGS_EQUAL(2, gm.discreteKeys().size());
}

/* ************************************************************************* */
// Test the error of the HybridGaussianFactor
TEST(HybridGaussianFactor, Error) {
  DiscreteKey m1(1, 2);

  auto A01 = Matrix2::Identity();
  auto A02 = Matrix2::Identity();

  auto A11 = Matrix2::Identity();
  auto A12 = Matrix2::Identity() * 2;

  auto b = Vector2::Zero();

  auto f0 = std::make_shared<JacobianFactor>(X(1), A01, X(2), A02, b);
  auto f1 = std::make_shared<JacobianFactor>(X(1), A11, X(2), A12, b);
  HybridGaussianFactor hybridFactor(m1, {f0, f1});

  VectorValues continuousValues;
  continuousValues.insert(X(1), Vector2(0, 0));
  continuousValues.insert(X(2), Vector2(1, 1));

  // error should return a tree of errors, with nodes for each discrete value.
  AlgebraicDecisionTree<Key> error_tree =
      hybridFactor.errorTree(continuousValues);

  std::vector<DiscreteKey> discrete_keys = {m1};
  // Error values for regression test
  std::vector<double> errors = {1, 4};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, errors);

  EXPECT(assert_equal(expected_error, error_tree));

  // Test for single leaf given discrete assignment P(X|M,Z).
  DiscreteValues discreteValues;
  discreteValues[m1.first] = 1;
  EXPECT_DOUBLES_EQUAL(
      4.0, hybridFactor.error({continuousValues, discreteValues}), 1e-9);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
