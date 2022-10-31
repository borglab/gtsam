/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testMixtureFactor.cpp
 * @brief   Unit tests for MixtureFactor
 * @author  Varun Agrawal
 * @date    October 2022
 */

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/MixtureFactor.h>
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
TEST(MixtureFactor, Constructor) {
  MixtureFactor factor;
  MixtureFactor::const_iterator const_it = factor.begin();
  CHECK(const_it == factor.end());
  MixtureFactor::iterator it = factor.begin();
  CHECK(it == factor.end());
}


TEST(MixtureFactor, Printing) {
  DiscreteKey m1(1, 2);
  double between0 = 0.0;
  double between1 = 1.0;

  Vector1 sigmas = Vector1(1.0);
  auto model = noiseModel::Diagonal::Sigmas(sigmas, false);

  auto f0 =
      boost::make_shared<BetweenFactor<double>>(X(1), X(2), between0, model);
  auto f1 =
      boost::make_shared<BetweenFactor<double>>(X(1), X(2), between1, model);
  std::vector<NonlinearFactor::shared_ptr> factors{f0, f1};

  MixtureFactor mixtureFactor({X(1), X(2)}, {m1}, factors);

  std::string expected =
      R"(Hybrid [x1 x2; 1]
MixtureFactor
 Choice(1) 
 0 Leaf Nonlinear factor on 2 keys
 1 Leaf Nonlinear factor on 2 keys
)";
  EXPECT(assert_print_equal(expected, mixtureFactor));
}

/* ************************************************************************* */
// Test the error of the MixtureFactor
TEST(MixtureFactor, Error) {
  DiscreteKey m1(1, 2);

  double between0 = 0.0;
  double between1 = 1.0;

  Vector1 sigmas = Vector1(1.0);
  auto model = noiseModel::Diagonal::Sigmas(sigmas, false);

  auto f0 =
      boost::make_shared<BetweenFactor<double>>(X(1), X(2), between0, model);
  auto f1 =
      boost::make_shared<BetweenFactor<double>>(X(1), X(2), between1, model);
  std::vector<NonlinearFactor::shared_ptr> factors{f0, f1};

  MixtureFactor mixtureFactor({X(1), X(2)}, {m1}, factors);

  Values continuousVals;
  continuousVals.insert<double>(X(1), 0);
  continuousVals.insert<double>(X(2), 1);

  AlgebraicDecisionTree<Key> error_tree = mixtureFactor.error(continuousVals);

  std::vector<DiscreteKey> discrete_keys = {m1};
  std::vector<double> errors = {0.5, 0};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, errors);

  EXPECT(assert_equal(expected_error, error_tree));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
