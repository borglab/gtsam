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
// "Add" two hybrid factors together.
TEST(HybridGaussianFactor, Sum) {
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
  std::vector<GaussianFactorValuePair> factorsA{{f10, 0.0}, {f11, 0.0}};
  std::vector<GaussianFactorValuePair> factorsB{
      {f20, 0.0}, {f21, 0.0}, {f22, 0.0}};

  // TODO(Frank): why specify keys at all? And: keys in factor should be *all*
  // keys, deviating from Kevin's scheme. Should we index DT on DiscreteKey?
  // Design review!
  HybridGaussianFactor hybridFactorA({X(1), X(2)}, {m1}, factorsA);
  HybridGaussianFactor hybridFactorB({X(1), X(3)}, {m2}, factorsB);

  // Check that number of keys is 3
  EXPECT_LONGS_EQUAL(3, hybridFactorA.keys().size());

  // Check that number of discrete keys is 1
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
  DiscreteKey m1(1, 2);
  auto A1 = Matrix::Zero(2, 1);
  auto A2 = Matrix::Zero(2, 2);
  auto b = Matrix::Zero(2, 1);
  auto f10 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
  auto f11 = std::make_shared<JacobianFactor>(X(1), A1, X(2), A2, b);
  std::vector<GaussianFactorValuePair> factors{{f10, 0.0}, {f11, 0.0}};

  HybridGaussianFactor hybridFactor({X(1), X(2)}, {m1}, factors);

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
  KeyVector keys;
  keys.push_back(X(0));
  keys.push_back(X(1));

  DiscreteKeys dKeys;
  dKeys.emplace_back(M(0), 2);
  dKeys.emplace_back(M(1), 2);

  auto gaussians = std::make_shared<GaussianConditional>();
  HybridGaussianConditional::Conditionals conditionals(gaussians);
  HybridGaussianConditional gm({}, keys, dKeys, conditionals);

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
  std::vector<GaussianFactorValuePair> factors{{f0, 0.0}, {f1, 0.0}};

  HybridGaussianFactor hybridFactor({X(1), X(2)}, {m1}, factors);

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
  DiscreteKeys discreteParents{m};
  hbn.emplace_shared<HybridGaussianConditional>(
      KeyVector{z}, KeyVector{}, discreteParents,
      HybridGaussianConditional::Conditionals(discreteParents,
                                              std::vector{c0, c1}));

  auto mixing = make_shared<DiscreteConditional>(m, "50/50");
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
TEST(HybridGaussianFactor, GaussianMixtureModel) {
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
    expected.emplace_shared<DiscreteConditional>(m, "50/50");

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
TEST(HybridGaussianFactor, GaussianMixtureModel2) {
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

void addMeasurement(HybridBayesNet &hbn, Key z_key, Key x_key, double sigma) {
  auto measurement_model = noiseModel::Isotropic::Sigma(1, sigma);
  hbn.emplace_shared<GaussianConditional>(z_key, Vector1(0.0), I_1x1, x_key,
                                          -I_1x1, measurement_model);
}

/// Create hybrid motion model p(x1 | x0, m1)
static HybridGaussianConditional::shared_ptr CreateHybridMotionModel(
    double mu0, double mu1, double sigma0, double sigma1) {
  auto model0 = noiseModel::Isotropic::Sigma(1, sigma0);
  auto model1 = noiseModel::Isotropic::Sigma(1, sigma1);
  auto c0 = make_shared<GaussianConditional>(X(1), Vector1(mu0), I_1x1, X(0),
                                             -I_1x1, model0),
       c1 = make_shared<GaussianConditional>(X(1), Vector1(mu1), I_1x1, X(0),
                                             -I_1x1, model1);
  DiscreteKeys discreteParents{m1};
  return std::make_shared<HybridGaussianConditional>(
      KeyVector{X(1)}, KeyVector{X(0)}, discreteParents,
      HybridGaussianConditional::Conditionals(discreteParents,
                                              std::vector{c0, c1}));
}

/// Create two state Bayes network with 1 or two measurement models
HybridBayesNet CreateBayesNet(
    const HybridGaussianConditional::shared_ptr &hybridMotionModel,
    bool add_second_measurement = false) {
  HybridBayesNet hbn;

  // Add measurement model p(z0 | x0)
  addMeasurement(hbn, Z(0), X(0), 3.0);

  // Optionally add second measurement model p(z1 | x1)
  if (add_second_measurement) {
    addMeasurement(hbn, Z(1), X(1), 3.0);
  }

  // Add hybrid motion model
  hbn.push_back(hybridMotionModel);

  // Discrete uniform prior.
  hbn.emplace_shared<DiscreteConditional>(m1, "50/50");

  return hbn;
}

/// Approximate the discrete marginal P(m1) using importance sampling
std::pair<double, double> approximateDiscreteMarginal(
    const HybridBayesNet &hbn,
    const HybridGaussianConditional::shared_ptr &hybridMotionModel,
    const VectorValues &given, size_t N = 100000) {
  /// Create importance sampling network q(x0,x1,m) = p(x1|x0,m1) q(x0) P(m1),
  /// using q(x0) = N(z0, sigmaQ) to sample x0.
  HybridBayesNet q;
  q.push_back(hybridMotionModel);  // Add hybrid motion model
  q.emplace_shared<GaussianConditional>(GaussianConditional::FromMeanAndStddev(
      X(0), given.at(Z(0)), /* sigmaQ = */ 3.0));  // Add proposal q(x0) for x0
  q.emplace_shared<DiscreteConditional>(m1, "50/50");  // Discrete prior.

  // Do importance sampling
  double w0 = 0.0, w1 = 0.0;
  std::mt19937_64 rng(42);
  for (int i = 0; i < N; i++) {
    HybridValues sample = q.sample(&rng);
    sample.insert(given);
    double weight = hbn.evaluate(sample) / q.evaluate(sample);
    (sample.atDiscrete(M(1)) == 0) ? w0 += weight : w1 += weight;
  }
  double pm1 = w1 / (w0 + w1);
  std::cout << "p(m0) = " << 100 * (1.0 - pm1) << std::endl;
  std::cout << "p(m1) = " << 100 * pm1 << std::endl;
  return {1.0 - pm1, pm1};
}

}  // namespace test_two_state_estimation

/* ************************************************************************* */
/**
 * Test a model p(z0|x0)p(z1|x1)p(x1|x0,m1)P(m1).
 *
 * p(x1|x0,m1) has mode-dependent mean but same covariance.
 *
 * Converting to a factor graph gives us ϕ(x0;z0)ϕ(x1;z1)ϕ(x1,x0,m1)P(m1)
 *
 * If we only have a measurement on x0, then
 * the posterior probability of m1 should be 0.5/0.5.
 * Getting a measurement on z1 gives use more information.
 */
TEST(HybridGaussianFactor, TwoStateModel) {
  using namespace test_two_state_estimation;

  double mu0 = 1.0, mu1 = 3.0;
  double sigma = 0.5;
  auto hybridMotionModel = CreateHybridMotionModel(mu0, mu1, sigma, sigma);

  // Start with no measurement on x1, only on x0
  const Vector1 z0(0.5);

  VectorValues given;
  given.insert(Z(0), z0);

  {
    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Since no measurement on x1, we hedge our bets
    // Importance sampling run with 100k samples gives 50.051/49.949
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "50/50");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete())));
  }

  {
    // If we set z1=4.5 (>> 2.5 which is the halfway point),
    // probability of discrete mode should be leaning to m1==1.
    const Vector1 z1(4.5);
    given.insert(Z(1), z1);

    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Since we have a measurement on x1, we get a definite result
    // Values taken from an importance sampling run with 100k samples:
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "44.3854/55.6146");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.002));
  }
}

/* ************************************************************************* */
/**
 * Test a model P(z0|x0)P(x1|x0,m1)P(z1|x1)P(m1).
 *
 * P(x1|x0,m1) has different means and different covariances.
 *
 * Converting to a factor graph gives us
 * ϕ(x0)ϕ(x1,x0,m1)ϕ(x1)P(m1)
 *
 * If we only have a measurement on z0, then
 * the P(m1) should be 0.5/0.5.
 * Getting a measurement on z1 gives use more information.
 */
TEST(HybridGaussianFactor, TwoStateModel2) {
  using namespace test_two_state_estimation;

  double mu0 = 1.0, mu1 = 3.0;
  double sigma0 = 0.5, sigma1 = 2.0;
  auto hybridMotionModel = CreateHybridMotionModel(mu0, mu1, sigma0, sigma1);

  // Start with no measurement on x1, only on x0
  const Vector1 z0(0.5);
  VectorValues given;
  given.insert(Z(0), z0);

  {
    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);

    // Check that ratio of Bayes net and factor graph for different modes is
    // equal for several values of {x0,x1}.
    for (VectorValues vv :
         {VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(1.0)}},
          VectorValues{{X(0), Vector1(0.5)}, {X(1), Vector1(3.0)}}}) {
      vv.insert(given);  // add measurements for HBN
      HybridValues hv0(vv, {{M(1), 0}}), hv1(vv, {{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }

    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Importance sampling run with 100k samples gives 50.095/49.905
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);

    // Since no measurement on x1, we a 50/50 probability
    auto p_m = bn->at(2)->asDiscrete();
    EXPECT_DOUBLES_EQUAL(0.5, p_m->operator()({{M(1), 0}}), 1e-9);
    EXPECT_DOUBLES_EQUAL(0.5, p_m->operator()({{M(1), 1}}), 1e-9);
  }

  {
    // Now we add a measurement z1 on x1
    const Vector1 z1(4.0);  // favors m==1
    given.insert(Z(1), z1);

    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);

    // Check that ratio of Bayes net and factor graph for different modes is
    // equal for several values of {x0,x1}.
    for (VectorValues vv :
         {VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(1.0)}},
          VectorValues{{X(0), Vector1(0.5)}, {X(1), Vector1(3.0)}}}) {
      vv.insert(given);  // add measurements for HBN
      HybridValues hv0(vv, {{M(1), 0}}), hv1(vv, {{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }

    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Values taken from an importance sampling run with 100k samples:
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "48.3158/51.6842");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.002));
  }

  {
    // Add a different measurement z1 on x1 that favors m==0
    const Vector1 z1(1.1);
    given.insert_or_assign(Z(1), z1);

    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Values taken from an importance sampling run with 100k samples:
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "55.396/44.604");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.002));
  }
}

/* ************************************************************************* */
/**
 * Test a model p(z0|x0)p(x1|x0,m1)p(z1|x1)p(m1).
 *
 * p(x1|x0,m1) has the same means but different covariances.
 *
 * Converting to a factor graph gives us
 * ϕ(x0)ϕ(x1,x0,m1)ϕ(x1)p(m1)
 *
 * If we only have a measurement on z0, then
 * the p(m1) should be 0.5/0.5.
 * Getting a measurement on z1 gives use more information.
 */
TEST(HybridGaussianFactor, TwoStateModel3) {
  using namespace test_two_state_estimation;

  double mu = 1.0;
  double sigma0 = 0.5, sigma1 = 2.0;
  auto hybridMotionModel = CreateHybridMotionModel(mu, mu, sigma0, sigma1);

  // Start with no measurement on x1, only on x0
  const Vector1 z0(0.5);
  VectorValues given;
  given.insert(Z(0), z0);

  {
    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);

    // Check that ratio of Bayes net and factor graph for different modes is
    // equal for several values of {x0,x1}.
    for (VectorValues vv :
         {VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(1.0)}},
          VectorValues{{X(0), Vector1(0.5)}, {X(1), Vector1(3.0)}}}) {
      vv.insert(given);  // add measurements for HBN
      HybridValues hv0(vv, {{M(1), 0}}), hv1(vv, {{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }

    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Importance sampling run with 100k samples gives 50.095/49.905
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);

    // Since no measurement on x1, we a 50/50 probability
    auto p_m = bn->at(2)->asDiscrete();
    EXPECT_DOUBLES_EQUAL(0.5, p_m->operator()({{M(1), 0}}), 1e-9);
    EXPECT_DOUBLES_EQUAL(0.5, p_m->operator()({{M(1), 1}}), 1e-9);
  }

  {
    // Now we add a measurement z1 on x1
    const Vector1 z1(4.0);  // favors m==1
    given.insert(Z(1), z1);

    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);

    // Check that ratio of Bayes net and factor graph for different modes is
    // equal for several values of {x0,x1}.
    for (VectorValues vv :
         {VectorValues{{X(0), Vector1(0.0)}, {X(1), Vector1(1.0)}},
          VectorValues{{X(0), Vector1(0.5)}, {X(1), Vector1(3.0)}}}) {
      vv.insert(given);  // add measurements for HBN
      HybridValues hv0(vv, {{M(1), 0}}), hv1(vv, {{M(1), 1}});
      EXPECT_DOUBLES_EQUAL(gfg.error(hv0) / hbn.error(hv0),
                           gfg.error(hv1) / hbn.error(hv1), 1e-9);
    }

    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Values taken from an importance sampling run with 100k samples:
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "51.7762/48.2238");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.002));
  }

  {
    // Add a different measurement z1 on x1 that favors m==1
    const Vector1 z1(7.0);
    given.insert_or_assign(Z(1), z1);

    HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
    HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
    HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

    // Values taken from an importance sampling run with 100k samples:
    // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
    DiscreteConditional expected(m1, "49.0762/50.9238");
    EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.005));
  }
}

/* ************************************************************************* */
/**
 * Same model, P(z0|x0)P(x1|x0,m1)P(z1|x1)P(m1), but now with very informative
 * measurements and vastly different motion model: either stand still or move
 * far. This yields a very informative posterior.
 */
TEST(HybridGaussianFactor, TwoStateModel4) {
  using namespace test_two_state_estimation;

  double mu0 = 0.0, mu1 = 10.0;
  double sigma0 = 0.2, sigma1 = 5.0;
  auto hybridMotionModel = CreateHybridMotionModel(mu0, mu1, sigma0, sigma1);

  // We only check the 2-measurement case
  const Vector1 z0(0.0), z1(10.0);
  VectorValues given{{Z(0), z0}, {Z(1), z1}};

  HybridBayesNet hbn = CreateBayesNet(hybridMotionModel, true);
  HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
  HybridBayesNet::shared_ptr bn = gfg.eliminateSequential();

  // Values taken from an importance sampling run with 100k samples:
  // approximateDiscreteMarginal(hbn, hybridMotionModel, given);
  DiscreteConditional expected(m1, "8.91527/91.0847");
  EXPECT(assert_equal(expected, *(bn->at(2)->asDiscrete()), 0.002));
}

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
  std::vector<GaussianFactorValuePair> factors{
      {f0, ComputeLogNormalizer(model0)}, {f1, ComputeLogNormalizer(model1)}};
  HybridGaussianFactor motionFactor({X(0), X(1)}, m1, factors);

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
