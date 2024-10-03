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
// "Add" two hybrid factors together.
TEST(HybridGaussianFactor, Sum) {
  using namespace test_constructor;
  DiscreteKey m2(2, 3);

  auto A3 = Matrix::Zero(2, 3);
  auto f20 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  auto f21 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  auto f22 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);

  // TODO(Frank): why specify keys at all? And: keys in factor should be *all*
  // keys, deviating from Kevin's scheme. Should we index DT on DiscreteKey?
  // Design review!
  HybridGaussianFactor hybridFactorA(m1, {f10, f11});
  HybridGaussianFactor hybridFactorB(m2, {f20, f21, f22});

  // Check the number of keys matches what we expect
  EXPECT_LONGS_EQUAL(3, hybridFactorA.keys().size());
  EXPECT_LONGS_EQUAL(2, hybridFactorA.continuousKeys().size());
  EXPECT_LONGS_EQUAL(1, hybridFactorA.discreteKeys().size());

  // Create sum of two hybrid factors: it will be a decision tree now on both
  // discrete variables m1 and m2:
  GaussianFactorGraphTree sum;
  sum += hybridFactorA;
  sum += hybridFactorB;

  // Let's check that this worked:
  Assignment<Key> mode;
  mode[m1.first] = 1;
  mode[m2.first] = 2;
  auto actual = sum(mode);
  EXPECT(actual.at(0) == f11);
  EXPECT(actual.at(1) == f22);
}

/* ************************************************************************* */
TEST(HybridGaussianFactor, Printing) {
  using namespace test_constructor;
  HybridGaussianFactor hybridFactor(m1, {f10, f11});

  std::string expected =
      R"(HybridGaussianFactor
Hybrid [x1 x2; 1]{
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
namespace test_direct_factor_graph {
/**
 * @brief Create a Factor Graph by directly specifying all
 * the factors instead of creating conditionals first.
 * This way we can directly provide the likelihoods and
 * then perform linearization.
 *
 * @param values Initial values to linearize around.
 * @param means The means of the HybridGaussianFactor components.
 * @param sigmas The covariances of the HybridGaussianFactor components.
 * @param m1 The discrete key.
 * @return HybridGaussianFactorGraph
 */
static HybridGaussianFactorGraph CreateFactorGraph(
    const gtsam::Values &values, const std::vector<double> &means,
    const std::vector<double> &sigmas, DiscreteKey &m1,
    double measurement_noise = 1e-3) {
  auto model0 = noiseModel::Isotropic::Sigma(1, sigmas[0]);
  auto model1 = noiseModel::Isotropic::Sigma(1, sigmas[1]);
  auto prior_noise = noiseModel::Isotropic::Sigma(1, measurement_noise);

  auto f0 =
      std::make_shared<BetweenFactor<double>>(X(0), X(1), means[0], model0)
          ->linearize(values);
  auto f1 =
      std::make_shared<BetweenFactor<double>>(X(0), X(1), means[1], model1)
          ->linearize(values);

  // Create HybridGaussianFactor
  // We take negative since we want
  // the underlying scalar to be log(\sqrt(|2πΣ|))
  std::vector<GaussianFactorValuePair> factors{{f0, model0->negLogConstant()},
                                               {f1, model1->negLogConstant()}};
  HybridGaussianFactor motionFactor(m1, factors);

  HybridGaussianFactorGraph hfg;
  hfg.push_back(motionFactor);

  hfg.push_back(PriorFactor<double>(X(0), values.at<double>(X(0)), prior_noise)
                    .linearize(values));

  return hfg;
}
}  // namespace test_direct_factor_graph

/* ************************************************************************* */
/**
 * @brief Test components with differing means but the same covariances.
 * The factor graph is
 *     *-X1-*-X2
 *          |
 *          M1
 */
TEST(HybridGaussianFactor, DifferentMeansFG) {
  using namespace test_direct_factor_graph;

  DiscreteKey m1(M(1), 2);

  Values values;
  double x1 = 0.0, x2 = 1.75;
  values.insert(X(0), x1);
  values.insert(X(1), x2);

  std::vector<double> means = {0.0, 2.0}, sigmas = {1e-0, 1e-0};

  HybridGaussianFactorGraph hfg = CreateFactorGraph(values, means, sigmas, m1);

  {
    auto bn = hfg.eliminateSequential();
    HybridValues actual = bn->optimize();

    HybridValues expected(
        VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(-1.75)}},
        DiscreteValues{{M(1), 0}});

    EXPECT(assert_equal(expected, actual));

    DiscreteValues dv0{{M(1), 0}};
    VectorValues cont0 = bn->optimize(dv0);
    double error0 = bn->error(HybridValues(cont0, dv0));
    // regression
    EXPECT_DOUBLES_EQUAL(0.69314718056, error0, 1e-9);

    DiscreteValues dv1{{M(1), 1}};
    VectorValues cont1 = bn->optimize(dv1);
    double error1 = bn->error(HybridValues(cont1, dv1));
    EXPECT_DOUBLES_EQUAL(error0, error1, 1e-9);
  }

  {
    auto prior_noise = noiseModel::Isotropic::Sigma(1, 1e-3);
    hfg.push_back(
        PriorFactor<double>(X(1), means[1], prior_noise).linearize(values));

    auto bn = hfg.eliminateSequential();
    HybridValues actual = bn->optimize();

    HybridValues expected(
        VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(0.25)}},
        DiscreteValues{{M(1), 1}});

    EXPECT(assert_equal(expected, actual));

    {
      DiscreteValues dv{{M(1), 0}};
      VectorValues cont = bn->optimize(dv);
      double error = bn->error(HybridValues(cont, dv));
      // regression
      EXPECT_DOUBLES_EQUAL(2.12692448787, error, 1e-9);
    }
    {
      DiscreteValues dv{{M(1), 1}};
      VectorValues cont = bn->optimize(dv);
      double error = bn->error(HybridValues(cont, dv));
      // regression
      EXPECT_DOUBLES_EQUAL(0.126928487854, error, 1e-9);
    }
  }
}

/* ************************************************************************* */
/**
 * @brief Test components with differing covariances but the same means.
 * The factor graph is
 *     *-X1-*-X2
 *          |
 *          M1
 */
TEST(HybridGaussianFactor, DifferentCovariancesFG) {
  using namespace test_direct_factor_graph;

  DiscreteKey m1(M(1), 2);

  Values values;
  double x1 = 1.0, x2 = 1.0;
  values.insert(X(0), x1);
  values.insert(X(1), x2);

  std::vector<double> means = {0.0, 0.0}, sigmas = {1e2, 1e-2};

  // Create FG with HybridGaussianFactor and prior on X1
  HybridGaussianFactorGraph fg = CreateFactorGraph(values, means, sigmas, m1);
  auto hbn = fg.eliminateSequential();

  VectorValues cv;
  cv.insert(X(0), Vector1(0.0));
  cv.insert(X(1), Vector1(0.0));

  // Check that the error values at the MLE point μ.
  AlgebraicDecisionTree<Key> errorTree = hbn->errorTree(cv);

  DiscreteValues dv0{{M(1), 0}};
  DiscreteValues dv1{{M(1), 1}};

  // regression
  EXPECT_DOUBLES_EQUAL(9.90348755254, errorTree(dv0), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.69314718056, errorTree(dv1), 1e-9);

  DiscreteConditional expected_m1(m1, "0.5/0.5");
  DiscreteConditional actual_m1 = *(hbn->at(2)->asDiscrete());

  EXPECT(assert_equal(expected_m1, actual_m1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
