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
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
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

/* ************************************************************************* */
// Test .print() output.
TEST(MixtureFactor, Printing) {
  DiscreteKey m1(1, 2);
  double between0 = 0.0;
  double between1 = 1.0;

  Vector1 sigmas = Vector1(1.0);
  auto model = noiseModel::Diagonal::Sigmas(sigmas, false);

  auto f0 =
      std::make_shared<BetweenFactor<double>>(X(1), X(2), between0, model);
  auto f1 =
      std::make_shared<BetweenFactor<double>>(X(1), X(2), between1, model);
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
static MixtureFactor getMixtureFactor() {
  DiscreteKey m1(1, 2);

  double between0 = 0.0;
  double between1 = 1.0;

  Vector1 sigmas = Vector1(1.0);
  auto model = noiseModel::Diagonal::Sigmas(sigmas, false);

  auto f0 =
      std::make_shared<BetweenFactor<double>>(X(1), X(2), between0, model);
  auto f1 =
      std::make_shared<BetweenFactor<double>>(X(1), X(2), between1, model);
  std::vector<NonlinearFactor::shared_ptr> factors{f0, f1};

  return MixtureFactor({X(1), X(2)}, {m1}, factors);
}

/* ************************************************************************* */
// Test the error of the MixtureFactor
TEST(MixtureFactor, Error) {
  auto mixtureFactor = getMixtureFactor();

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
// Test dim of the MixtureFactor
TEST(MixtureFactor, Dim) {
  auto mixtureFactor = getMixtureFactor();
  EXPECT_LONGS_EQUAL(1, mixtureFactor.dim());
}

/* ************************************************************************* */
// Test components with differing means
TEST(MixtureFactor, DifferentMeans) {
  DiscreteKey m1(M(1), 2), m2(M(2), 2);

  Values values;
  double x1 = 0.0, x2 = 1.75, x3 = 2.60;
  values.insert(X(1), x1);
  values.insert(X(2), x2);
  values.insert(X(3), x3);

  auto model0 = noiseModel::Isotropic::Sigma(1, 1e-0);
  auto model1 = noiseModel::Isotropic::Sigma(1, 1e-0);
  auto prior_noise = noiseModel::Isotropic::Sigma(1, 1e-0);

  auto f0 = std::make_shared<BetweenFactor<double>>(X(1), X(2), 0.0, model0);
  auto f1 = std::make_shared<BetweenFactor<double>>(X(1), X(2), 2.0, model1);
  std::vector<NonlinearFactor::shared_ptr> factors{f0, f1};

  MixtureFactor mixtureFactor({X(1), X(2)}, {m1}, factors);
  HybridNonlinearFactorGraph hnfg;
  hnfg.push_back(mixtureFactor);

  f0 = std::make_shared<BetweenFactor<double>>(X(2), X(3), 0.0, model0);
  f1 = std::make_shared<BetweenFactor<double>>(X(2), X(3), 2.0, model1);
  std::vector<NonlinearFactor::shared_ptr> factors23{f0, f1};
  hnfg.push_back(MixtureFactor({X(2), X(3)}, {m2}, factors23));

  auto prior = PriorFactor<double>(X(1), x1, prior_noise);
  hnfg.push_back(prior);

  hnfg.emplace_shared<PriorFactor<double>>(X(2), 2.0, prior_noise);

  auto hgfg = hnfg.linearize(values);
  auto bn = hgfg->eliminateSequential();
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
// Test components with differing covariances
TEST(MixtureFactor, DifferentCovariances) {
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
  auto gm = new gtsam::GaussianMixture(
      {Z(1)}, {X(1), X(2)}, {m1},
      {std::make_shared<GaussianConditional>(terms0, 1, -d0, model0),
       std::make_shared<GaussianConditional>(terms1, 1, -d1, model1)});
  gtsam::HybridBayesNet bn;
  bn.emplace_back(gm);

  gtsam::VectorValues measurements;
  measurements.insert(Z(1), gtsam::Z_1x1);
  // Create FG with single GaussianMixtureFactor
  HybridGaussianFactorGraph mixture_fg = bn.toFactorGraph(measurements);

  // Linearized prior factor on X1
  auto prior = PriorFactor<double>(X(1), x1, prior_noise).linearize(values);
  mixture_fg.push_back(prior);

  auto hbn = mixture_fg.eliminateSequential();

  VectorValues cv;
  cv.insert(X(1), Vector1(0.0));
  cv.insert(X(2), Vector1(0.0));

  // Check that we get different error values at the MLE point μ.
  AlgebraicDecisionTree<Key> errorTree = hbn->errorTree(cv);
  auto cond0 = hbn->at(0)->asMixture();
  auto cond1 = hbn->at(1)->asMixture();
  auto discrete_cond = hbn->at(2)->asDiscrete();

  HybridValues hv0(cv, DiscreteValues{{M(1), 0}});
  HybridValues hv1(cv, DiscreteValues{{M(1), 1}});
  AlgebraicDecisionTree<Key> expectedErrorTree(
      m1,
      cond0->error(hv0)  // cond0(0)->logNormalizationConstant()
                         // - cond0(1)->logNormalizationConstant
          + cond1->error(hv0) + discrete_cond->error(DiscreteValues{{M(1), 0}}),
      cond0->error(hv1)  // cond1(0)->logNormalizationConstant()
                         // - cond1(1)->logNormalizationConstant
          + cond1->error(hv1) +
          discrete_cond->error(DiscreteValues{{M(1), 0}}));
  EXPECT(assert_equal(expectedErrorTree, errorTree));

  DiscreteValues dv;
  dv.insert({M(1), 1});
  HybridValues expected_values(cv, dv);

  HybridValues actual_values = hbn->optimize();

  EXPECT(assert_equal(expected_values, actual_values));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
