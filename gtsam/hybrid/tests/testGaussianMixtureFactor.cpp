/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianMixtureFactor.cpp
 * @brief   Unit tests for GaussianMixtureFactor
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
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
TEST(GaussianMixtureFactor, Constructor) {
  GaussianMixtureFactor factor;
  GaussianMixtureFactor::const_iterator const_it = factor.begin();
  CHECK(const_it == factor.end());
  GaussianMixtureFactor::iterator it = factor.begin();
  CHECK(it == factor.end());
}

/* ************************************************************************* */
// "Add" two mixture factors together.
TEST(GaussianMixtureFactor, Sum) {
  DiscreteKey m1(1, 2), m2(2, 3);

  auto A1 = Matrix::Zero(2, 1);
  auto A2 = Matrix::Zero(2, 2);
  auto A3 = Matrix::Zero(2, 3);
  auto b = Matrix::Zero(2, 1);
  Vector2 sigmas;
  sigmas << 1, 2;

  auto f10 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
  auto f11 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
  auto f20 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  auto f21 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  auto f22 = std::make_shared<JacobianFactor>(X(1), A1, X(3), A3, b);
  std::vector<GaussianFactor::shared_ptr> factorsA{f10, f11};
  std::vector<GaussianFactor::shared_ptr> factorsB{f20, f21, f22};

  // TODO(Frank): why specify keys at all? And: keys in factor should be *all*
  // keys, deviating from Kevin's scheme. Should we index DT on DiscreteKey?
  // Design review!
  GaussianMixtureFactor mixtureFactorA({X(1), X(2)}, {m1}, factorsA);
  GaussianMixtureFactor mixtureFactorB({X(1), X(3)}, {m2}, factorsB);

  // Check that number of keys is 3
  EXPECT_LONGS_EQUAL(3, mixtureFactorA.keys().size());

  // Check that number of discrete keys is 1
  EXPECT_LONGS_EQUAL(1, mixtureFactorA.discreteKeys().size());

  // Create sum of two mixture factors: it will be a decision tree now on both
  // discrete variables m1 and m2:
  GaussianFactorGraphTree sum;
  sum += mixtureFactorA;
  sum += mixtureFactorB;

  // Let's check that this worked:
  Assignment<Key> mode;
  mode[m1.first] = 1;
  mode[m2.first] = 2;
  auto actual = sum(mode);
  EXPECT(actual.at(0) == f11);
  EXPECT(actual.at(1) == f22);
}

/* ************************************************************************* */
TEST(GaussianMixtureFactor, Printing) {
  DiscreteKey m1(1, 2);
  auto A1 = Matrix::Zero(2, 1);
  auto A2 = Matrix::Zero(2, 2);
  auto b = Matrix::Zero(2, 1);
  auto f10 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
  auto f11 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
  std::vector<GaussianFactor::shared_ptr> factors{f10, f11};

  GaussianMixtureFactor mixtureFactor({X(1), X(2)}, {m1}, factors);

  std::string expected =
      R"(GaussianMixtureFactor
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
  EXPECT(assert_print_equal(expected, mixtureFactor));
}

/* ************************************************************************* */
TEST(GaussianMixtureFactor, GaussianMixture) {
  KeyVector keys;
  keys.push_back(X(0));
  keys.push_back(X(1));

  DiscreteKeys dKeys;
  dKeys.emplace_back(M(0), 2);
  dKeys.emplace_back(M(1), 2);

  auto gaussians = std::make_shared<GaussianConditional>();
  GaussianMixture::Conditionals conditionals(gaussians);
  GaussianMixture gm({}, keys, dKeys, conditionals);

  EXPECT_LONGS_EQUAL(2, gm.discreteKeys().size());
}

/* ************************************************************************* */
// Test the error of the GaussianMixtureFactor
TEST(GaussianMixtureFactor, Error) {
  DiscreteKey m1(1, 2);

  auto A01 = Matrix2::Identity();
  auto A02 = Matrix2::Identity();

  auto A11 = Matrix2::Identity();
  auto A12 = Matrix2::Identity() * 2;

  auto b = Vector2::Zero();

  auto f0 = std::make_shared<JacobianFactor>(X(1), A01, X(2), A02, b);
  auto f1 = std::make_shared<JacobianFactor>(X(1), A11, X(2), A12, b);
  std::vector<GaussianFactor::shared_ptr> factors{f0, f1};

  GaussianMixtureFactor mixtureFactor({X(1), X(2)}, {m1}, factors);

  VectorValues continuousValues;
  continuousValues.insert(X(1), Vector2(0, 0));
  continuousValues.insert(X(2), Vector2(1, 1));

  // error should return a tree of errors, with nodes for each discrete value.
  AlgebraicDecisionTree<Key> error_tree =
      mixtureFactor.errorTree(continuousValues);

  std::vector<DiscreteKey> discrete_keys = {m1};
  // Error values for regression test
  std::vector<double> errors = {1, 4};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, errors);

  EXPECT(assert_equal(expected_error, error_tree));

  // Test for single leaf given discrete assignment P(X|M,Z).
  DiscreteValues discreteValues;
  discreteValues[m1.first] = 1;
  EXPECT_DOUBLES_EQUAL(
      4.0, mixtureFactor.error({continuousValues, discreteValues}), 1e-9);
}

/* ************************************************************************* */
// Test components with differing means
TEST(GaussianMixtureFactor, DifferentMeans) {
  DiscreteKey m1(M(1), 2), m2(M(2), 2);

  Values values;
  double x1 = 0.0, x2 = 1.75, x3 = 2.60;
  values.insert(X(1), x1);
  values.insert(X(2), x2);
  values.insert(X(3), x3);

  auto model0 = noiseModel::Isotropic::Sigma(1, 1e-0);
  auto model1 = noiseModel::Isotropic::Sigma(1, 1e-0);
  auto prior_noise = noiseModel::Isotropic::Sigma(1, 1e-0);

  auto f0 = std::make_shared<BetweenFactor<double>>(X(1), X(2), 0.0, model0)
                ->linearize(values);
  auto f1 = std::make_shared<BetweenFactor<double>>(X(1), X(2), 2.0, model1)
                ->linearize(values);
  std::vector<GaussianFactor::shared_ptr> factors{f0, f1};

  GaussianMixtureFactor mixtureFactor({X(1), X(2)}, {m1}, factors, true);
  HybridGaussianFactorGraph hfg;
  hfg.push_back(mixtureFactor);

  f0 = std::make_shared<BetweenFactor<double>>(X(2), X(3), 0.0, model0)
           ->linearize(values);
  f1 = std::make_shared<BetweenFactor<double>>(X(2), X(3), 2.0, model1)
           ->linearize(values);
  std::vector<GaussianFactor::shared_ptr> factors23{f0, f1};
  hfg.push_back(GaussianMixtureFactor({X(2), X(3)}, {m2}, factors23, true));

  auto prior = PriorFactor<double>(X(1), x1, prior_noise).linearize(values);
  hfg.push_back(prior);

  hfg.push_back(PriorFactor<double>(X(2), 2.0, prior_noise).linearize(values));

  auto bn = hfg.eliminateSequential();
  HybridValues actual = bn->optimize();

  HybridValues expected(
      VectorValues{
          {X(1), Vector1(0.0)}, {X(2), Vector1(0.25)}, {X(3), Vector1(-0.6)}},
      DiscreteValues{{M(1), 1}, {M(2), 0}});

  EXPECT(assert_equal(expected, actual));

  {
    DiscreteValues dv{{M(1), 0}, {M(2), 0}};
    VectorValues cont = bn->optimize(dv);
    double error = bn->error(HybridValues(cont, dv));
    // regression
    EXPECT_DOUBLES_EQUAL(1.77418393408, error, 1e-9);
  }
  {
    DiscreteValues dv{{M(1), 0}, {M(2), 1}};
    VectorValues cont = bn->optimize(dv);
    double error = bn->error(HybridValues(cont, dv));
    // regression
    EXPECT_DOUBLES_EQUAL(1.77418393408, error, 1e-9);
  }
  {
    DiscreteValues dv{{M(1), 1}, {M(2), 0}};
    VectorValues cont = bn->optimize(dv);
    double error = bn->error(HybridValues(cont, dv));
    // regression
    EXPECT_DOUBLES_EQUAL(1.10751726741, error, 1e-9);
  }
  {
    DiscreteValues dv{{M(1), 1}, {M(2), 1}};
    VectorValues cont = bn->optimize(dv);
    double error = bn->error(HybridValues(cont, dv));
    // regression
    EXPECT_DOUBLES_EQUAL(1.10751726741, error, 1e-9);
  }
}

/* ************************************************************************* */
/**
 * @brief Test components with differing covariances.
 * The factor graph is
 *     *-X1-*-X2
 *          |
 *          M1
 */
TEST(GaussianMixtureFactor, DifferentCovariances) {
  DiscreteKey m1(M(1), 2);

  Values values;
  double x1 = 1.0, x2 = 1.0;
  values.insert(X(1), x1);
  values.insert(X(2), x2);

  double between = 0.0;

  auto model0 = noiseModel::Isotropic::Sigma(1, 1e2);
  auto model1 = noiseModel::Isotropic::Sigma(1, 1e-2);
  auto prior_noise = noiseModel::Isotropic::Sigma(1, 1e-3);

  auto f0 =
      std::make_shared<BetweenFactor<double>>(X(1), X(2), between, model0);
  auto f1 =
      std::make_shared<BetweenFactor<double>>(X(1), X(2), between, model1);
  std::vector<NonlinearFactor::shared_ptr> factors{f0, f1};

  // Create via toFactorGraph
  using symbol_shorthand::Z;
  Matrix H0_1, H0_2, H1_1, H1_2;
  Vector d0 = f0->evaluateError(x1, x2, &H0_1, &H0_2);
  std::vector<std::pair<Key, Matrix>> terms0 = {{Z(1), gtsam::I_1x1 /*Rx*/},
                                                //
                                                {X(1), H0_1 /*Sp1*/},
                                                {X(2), H0_2 /*Tp2*/}};

  Vector d1 = f1->evaluateError(x1, x2, &H1_1, &H1_2);
  std::vector<std::pair<Key, Matrix>> terms1 = {{Z(1), gtsam::I_1x1 /*Rx*/},
                                                //
                                                {X(1), H1_1 /*Sp1*/},
                                                {X(2), H1_2 /*Tp2*/}};
  gtsam::GaussianMixtureFactor gmf(
      {X(1), X(2)}, {m1},
      {std::make_shared<JacobianFactor>(X(1), H0_1, X(2), H0_2, -d0, model0),
       std::make_shared<JacobianFactor>(X(1), H1_1, X(2), H1_2, -d1, model1)},
      true);

  // Create FG with single GaussianMixtureFactor
  HybridGaussianFactorGraph mixture_fg;
  mixture_fg.add(gmf);

  // Linearized prior factor on X1
  auto prior = PriorFactor<double>(X(1), x1, prior_noise).linearize(values);
  mixture_fg.push_back(prior);

  auto hbn = mixture_fg.eliminateSequential();
  // hbn->print();

  VectorValues cv;
  cv.insert(X(1), Vector1(0.0));
  cv.insert(X(2), Vector1(0.0));

  // Check that the error values at the MLE point μ.
  AlgebraicDecisionTree<Key> errorTree = hbn->errorTree(cv);

  DiscreteValues dv0{{M(1), 0}};
  DiscreteValues dv1{{M(1), 1}};

  // regression
  EXPECT_DOUBLES_EQUAL(0.69314718056, errorTree(dv0), 1e-9);
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