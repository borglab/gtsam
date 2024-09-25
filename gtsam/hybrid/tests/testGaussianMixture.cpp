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

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
