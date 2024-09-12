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
using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

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

namespace test_gmm {

/**
 * Function to compute P(m=1|z). For P(m=0|z), swap mus and sigmas.
 * If sigma0 == sigma1, it simplifies to a sigmoid function.
 *
 * Follows equation 7.108 since it is more generic.
 */
double prob_m_z(double mu0, double mu1, double sigma0, double sigma1,
                double z) {
  double x1 = ((z - mu0) / sigma0), x2 = ((z - mu1) / sigma1);
  double d = sigma1 / sigma0;
  double e = d * std::exp(-0.5 * (x1 * x1 - x2 * x2));
  return 1 / (1 + e);
};

static HybridBayesNet GetGaussianMixtureModel(double mu0, double mu1,
                                              double sigma0, double sigma1) {
  DiscreteKey m(M(0), 2);
  Key z = Z(0);

  auto model0 = noiseModel::Isotropic::Sigma(1, sigma0);
  auto model1 = noiseModel::Isotropic::Sigma(1, sigma1);

  auto c0 = make_shared<GaussianConditional>(z, Vector1(mu0), I_1x1, model0),
       c1 = make_shared<GaussianConditional>(z, Vector1(mu1), I_1x1, model1);

  HybridBayesNet hbn;
  hbn.emplace_shared<GaussianMixture>(KeyVector{z}, KeyVector{},
                                      DiscreteKeys{m}, std::vector{c0, c1});

  auto mixing = make_shared<DiscreteConditional>(m, "0.5/0.5");
  hbn.push_back(mixing);

  return hbn;
}

}  // namespace test_gmm

/* ************************************************************************* */
/**
 * Test a simple Gaussian Mixture Model represented as P(m)P(z|m)
 * where m is a discrete variable and z is a continuous variable.
 * m is binary and depending on m, we have 2 different means
 * μ1 and μ2 for the Gaussian distribution around which we sample z.
 *
 * The resulting factor graph should eliminate to a Bayes net
 * which represents a sigmoid function.
 */
TEST(GaussianMixtureFactor, GaussianMixtureModel) {
  using namespace test_gmm;

  double mu0 = 1.0, mu1 = 3.0;
  double sigma = 2.0;

  DiscreteKey m(M(0), 2);
  Key z = Z(0);

  auto hbn = GetGaussianMixtureModel(mu0, mu1, sigma, sigma);

  // The result should be a sigmoid.
  // So should be P(m=1|z) = 0.5 at z=3.0 - 1.0=2.0
  double midway = mu1 - mu0, lambda = 4;
  {
    VectorValues given;
    given.insert(z, Vector1(midway));

    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    EXPECT_DOUBLES_EQUAL(
        prob_m_z(mu0, mu1, sigma, sigma, midway),
        bn->at(0)->asDiscrete()->operator()(DiscreteValues{{m.first, 1}}),
        1e-8);

    // At the halfway point between the means, we should get P(m|z)=0.5
    HybridBayesNet expected;
    expected.emplace_shared<DiscreteConditional>(m, "0.5/0.5");

    EXPECT(assert_equal(expected, *bn));
  }
  {
    // Shift by -lambda
    VectorValues given;
    given.insert(z, Vector1(midway - lambda));

    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    EXPECT_DOUBLES_EQUAL(
        prob_m_z(mu0, mu1, sigma, sigma, midway - lambda),
        bn->at(0)->asDiscrete()->operator()(DiscreteValues{{m.first, 1}}),
        1e-8);
  }
  {
    // Shift by lambda
    VectorValues given;
    given.insert(z, Vector1(midway + lambda));

    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    EXPECT_DOUBLES_EQUAL(
        prob_m_z(mu0, mu1, sigma, sigma, midway + lambda),
        bn->at(0)->asDiscrete()->operator()(DiscreteValues{{m.first, 1}}),
        1e-8);
  }
}

/* ************************************************************************* */
/**
 * Test a simple Gaussian Mixture Model represented as P(m)P(z|m)
 * where m is a discrete variable and z is a continuous variable.
 * m is binary and depending on m, we have 2 different means
 * and covariances each for the
 * Gaussian distribution around which we sample z.
 *
 * The resulting factor graph should eliminate to a Bayes net
 * which represents a Gaussian-like function
 * where m1>m0 close to 3.1333.
 */
TEST(GaussianMixtureFactor, GaussianMixtureModel2) {
  using namespace test_gmm;

  double mu0 = 1.0, mu1 = 3.0;
  double sigma0 = 8.0, sigma1 = 4.0;

  DiscreteKey m(M(0), 2);
  Key z = Z(0);

  auto hbn = GetGaussianMixtureModel(mu0, mu1, sigma0, sigma1);

  double m1_high = 3.133, lambda = 4;
  {
    // The result should be a bell curve like function
    // with m1 > m0 close to 3.1333.
    // We get 3.1333 by finding the maximum value of the function.
    VectorValues given;
    given.insert(z, Vector1(3.133));

    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    EXPECT_DOUBLES_EQUAL(
        prob_m_z(mu0, mu1, sigma0, sigma1, m1_high),
        bn->at(0)->asDiscrete()->operator()(DiscreteValues{{M(0), 1}}), 1e-8);

    // At the halfway point between the means
    HybridBayesNet expected;
    expected.emplace_shared<DiscreteConditional>(
        m, DiscreteKeys{},
        vector<double>{prob_m_z(mu1, mu0, sigma1, sigma0, m1_high),
                       prob_m_z(mu0, mu1, sigma0, sigma1, m1_high)});

    EXPECT(assert_equal(expected, *bn));
  }
  {
    // Shift by -lambda
    VectorValues given;
    given.insert(z, Vector1(m1_high - lambda));

    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    EXPECT_DOUBLES_EQUAL(
        prob_m_z(mu0, mu1, sigma0, sigma1, m1_high - lambda),
        bn->at(0)->asDiscrete()->operator()(DiscreteValues{{m.first, 1}}),
        1e-8);
  }
  {
    // Shift by lambda
    VectorValues given;
    given.insert(z, Vector1(m1_high + lambda));

    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    EXPECT_DOUBLES_EQUAL(
        prob_m_z(mu0, mu1, sigma0, sigma1, m1_high + lambda),
        bn->at(0)->asDiscrete()->operator()(DiscreteValues{{m.first, 1}}),
        1e-8);
  }
}

namespace test_two_state_estimation {

DiscreteKey m1(M(1), 2);

/// Create Two State Bayes Network with measurements
static HybridBayesNet CreateBayesNet(double mu0, double mu1, double sigma0,
                                     double sigma1,
                                     bool add_second_measurement = false,
                                     double prior_sigma = 1e-3,
                                     double measurement_sigma = 3.0) {
  HybridBayesNet hbn;

  auto measurement_model = noiseModel::Isotropic::Sigma(1, measurement_sigma);
  // Add measurement P(z0 | x0)
  auto p_z0 = std::make_shared<GaussianConditional>(
      Z(0), Vector1(0.0), -I_1x1, X(0), I_1x1, measurement_model);
  hbn.push_back(p_z0);

  // Add hybrid motion model
  auto model0 = noiseModel::Isotropic::Sigma(1, sigma0);
  auto model1 = noiseModel::Isotropic::Sigma(1, sigma1);
  auto c0 = make_shared<GaussianConditional>(X(1), Vector1(mu0), I_1x1, X(0),
                                             -I_1x1, model0),
       c1 = make_shared<GaussianConditional>(X(1), Vector1(mu1), I_1x1, X(0),
                                             -I_1x1, model1);

  auto motion = std::make_shared<GaussianMixture>(
      KeyVector{X(1)}, KeyVector{X(0)}, DiscreteKeys{m1}, std::vector{c0, c1});
  hbn.push_back(motion);

  if (add_second_measurement) {
    // Add second measurement
    auto p_z1 = std::make_shared<GaussianConditional>(
        Z(1), Vector1(0.0), -I_1x1, X(1), I_1x1, measurement_model);
    hbn.push_back(p_z1);
  }

  // Discrete uniform prior.
  auto p_m1 = std::make_shared<DiscreteConditional>(m1, "0.5/0.5");
  hbn.push_back(p_m1);

  return hbn;
}

}  // namespace test_two_state_estimation

/* ************************************************************************* */
/**
 * Test a model P(z0|x0)P(x1|x0,m1)P(z1|x1)P(m1).
 *
 * P(f01|x1,x0,m1) has different means and same covariance.
 *
 * Converting to a factor graph gives us
 * ϕ(x0)ϕ(x1,x0,m1)ϕ(x1)P(m1)
 *
 * If we only have a measurement on z0, then
 * the probability of m1 should be 0.5/0.5.
 * Getting a measurement on z1 gives use more information.
 */
TEST(GaussianMixtureFactor, TwoStateModel) {
  using namespace test_two_state_estimation;

  double mu0 = 1.0, mu1 = 3.0;
  double sigma = 2.0;

  // Start with no measurement on x1, only on x0
  HybridBayesNet hbn = CreateBayesNet(mu0, mu1, sigma, sigma, false);

  VectorValues given;
  given.insert(Z(0), Vector1(0.5));

  {
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Since no measurement on x1, we hedge our bets
    DiscreteConditional expected(m1, "0.5/0.5");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete())));
  }

  {
    // Now we add a measurement z1 on x1
    hbn = CreateBayesNet(mu0, mu1, sigma, sigma, true);

    // If we see z1=2.6 (> 2.5 which is the halfway point),
    // discrete mode should say m1=1
    given.insert(Z(1), Vector1(2.6));
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Since we have a measurement on z2, we get a definite result
    DiscreteConditional expected(m1, "0.49772729/0.50227271");
    // regression
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 1e-6));
  }
}

/* ************************************************************************* */
/**
 * Test a model P(z0|x0)P(x1|x0,m1)P(z1|x1)P(m1).
 *
 * P(f01|x1,x0,m1) has different means and different covariances.
 *
 * Converting to a factor graph gives us
 * ϕ(x0)ϕ(x1,x0,m1)ϕ(x1)P(m1)
 *
 * If we only have a measurement on z0, then
 * the P(m1) should be 0.5/0.5.
 * Getting a measurement on z1 gives use more information.
 */
TEST(GaussianMixtureFactor, TwoStateModel2) {
  using namespace test_two_state_estimation;

  double mu0 = 1.0, mu1 = 3.0;
  double sigma0 = 6.0, sigma1 = 4.0;
  auto model0 = noiseModel::Isotropic::Sigma(1, sigma0);
  auto model1 = noiseModel::Isotropic::Sigma(1, sigma1);

  // Start with no measurement on x1, only on x0
  HybridBayesNet hbn = CreateBayesNet(mu0, mu1, sigma0, sigma1, false);

  VectorValues given;
  given.insert(Z(0), Vector1(0.5));

  {
    // Start with no measurement on x1, only on x0
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);

    {
      VectorValues vv{
          {X(0), Vector1(0.0)}, {X(1), Vector1(1.0)}, {Z(0), Vector1(0.5)}};
      HybridValues hv0(vv, DiscreteValues{{M(1), 0}}),
          hv1(vv, DiscreteValues{{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }
    {
      VectorValues vv{
          {X(0), Vector1(0.5)}, {X(1), Vector1(3.0)}, {Z(0), Vector1(0.5)}};
      HybridValues hv0(vv, DiscreteValues{{M(1), 0}}),
          hv1(vv, DiscreteValues{{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }

    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Since no measurement on x1, we a 50/50 probability
    auto p_m = bn->at(2)->asDiscrete();
    EXPECT_DOUBLES_EQUAL(0.5, p_m->operator()(DiscreteValues{{m1.first, 0}}),
                         1e-9);
    EXPECT_DOUBLES_EQUAL(0.5, p_m->operator()(DiscreteValues{{m1.first, 1}}),
                         1e-9);
  }

  {
    // Now we add a measurement z1 on x1
    hbn = CreateBayesNet(mu0, mu1, sigma0, sigma1, true);

    given.insert(Z(1), Vector1(2.2));
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);

    {
      VectorValues vv{{X(0), Vector1(0.0)},
                      {X(1), Vector1(1.0)},
                      {Z(0), Vector1(0.5)},
                      {Z(1), Vector1(2.2)}};
      HybridValues hv0(vv, DiscreteValues{{M(1), 0}}),
          hv1(vv, DiscreteValues{{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }
    {
      VectorValues vv{{X(0), Vector1(0.5)},
                      {X(1), Vector1(3.0)},
                      {Z(0), Vector1(0.5)},
                      {Z(1), Vector1(2.2)}};
      HybridValues hv0(vv, DiscreteValues{{M(1), 0}}),
          hv1(vv, DiscreteValues{{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }

    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Since we have a measurement on z2, we get a definite result
    DiscreteConditional expected(m1, "0.44744586/0.55255414");
    // regression
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 1e-6));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
