/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianMixture.cpp
 * @brief   Test hybrid elimination with a simple mixture model
 * @author  Varun Agrawal
 * @author  Frank Dellaert
 * @date    September 2024
 */

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/NoiseModel.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using symbol_shorthand::M;
using symbol_shorthand::Z;

// Define mode key and an assignment m==1
const DiscreteKey m(M(0), 2);
const DiscreteValues m1Assignment{{M(0), 1}};

// Define a 50/50 prior on the mode
DiscreteConditional::shared_ptr mixing =
    std::make_shared<DiscreteConditional>(m, "60/40");

/// Gaussian density function
double Gaussian(double mu, double sigma, double z) {
  return exp(-0.5 * pow((z - mu) / sigma, 2)) / sqrt(2 * M_PI * sigma * sigma);
};

/**
 * Closed form computation of P(m=1|z).
 * If sigma0 == sigma1, it simplifies to a sigmoid function.
 * Hardcodes 60/40 prior on mode.
 */
double prob_m_z(double mu0, double mu1, double sigma0, double sigma1,
                double z) {
  const double p0 = 0.6 * Gaussian(mu0, sigma0, z);
  const double p1 = 0.4 * Gaussian(mu1, sigma1, z);
  return p1 / (p0 + p1);
};

/*
 * Test a Gaussian Mixture Model P(m)p(z|m) with same sigma.
 * The posterior, as a function of z, should be a sigmoid function.
 */
TEST(GaussianMixture, GaussianMixtureModel) {
  double mu0 = 1.0, mu1 = 3.0;
  double sigma = 2.0;

  // Create a Gaussian mixture model p(z|m) with same sigma.
  HybridBayesNet gmm;
  std::vector<std::pair<Vector, double>> parameters{{Vector1(mu0), sigma},
                                                    {Vector1(mu1), sigma}};
  gmm.emplace_shared<HybridGaussianConditional>(m, Z(0), parameters);
  gmm.push_back(mixing);

  // At the halfway point between the means, we should get P(m|z)=0.5
  double midway = mu1 - mu0;
  auto eliminationResult =
      gmm.toFactorGraph({{Z(0), Vector1(midway)}}).eliminateSequential();
  auto pMid = *eliminationResult->at(0)->asDiscrete();
  EXPECT(assert_equal(DiscreteConditional(m, "60/40"), pMid));

  // Everywhere else, the result should be a sigmoid.
  for (const double shift : {-4, -2, 0, 2, 4}) {
    const double z = midway + shift;
    const double expected = prob_m_z(mu0, mu1, sigma, sigma, z);

    // Workflow 1: convert HBN to HFG and solve
    auto eliminationResult1 =
        gmm.toFactorGraph({{Z(0), Vector1(z)}}).eliminateSequential();
    auto posterior1 = *eliminationResult1->at(0)->asDiscrete();
    EXPECT_DOUBLES_EQUAL(expected, posterior1(m1Assignment), 1e-8);

    // Workflow 2: directly specify HFG and solve
    HybridGaussianFactorGraph hfg1;
    hfg1.emplace_shared<DecisionTreeFactor>(
        m, std::vector{Gaussian(mu0, sigma, z), Gaussian(mu1, sigma, z)});
    hfg1.push_back(mixing);
    auto eliminationResult2 = hfg1.eliminateSequential();
    auto posterior2 = *eliminationResult2->at(0)->asDiscrete();
    EXPECT_DOUBLES_EQUAL(expected, posterior2(m1Assignment), 1e-8);
  }
}

/*
 * Test a Gaussian Mixture Model P(m)p(z|m) with different sigmas.
 * The posterior, as a function of z, should be a unimodal function.
 */
TEST(GaussianMixture, GaussianMixtureModel2) {
  double mu0 = 1.0, mu1 = 3.0;
  double sigma0 = 8.0, sigma1 = 4.0;

  // Create a Gaussian mixture model p(z|m) with same sigma.
  HybridBayesNet gmm;
  std::vector<std::pair<Vector, double>> parameters{{Vector1(mu0), sigma0},
                                                    {Vector1(mu1), sigma1}};
  gmm.emplace_shared<HybridGaussianConditional>(m, Z(0), parameters);
  gmm.push_back(mixing);

  // We get zMax=3.1333 by finding the maximum value of the function, at which
  // point the mode m==1 is about twice as probable as m==0.
  double zMax = 3.133;
  const VectorValues vv{{Z(0), Vector1(zMax)}};
  auto gfg = gmm.toFactorGraph(vv);

  // Equality of posteriors asserts that the elimination is correct (same ratios
  // for all modes)
  const auto& expectedDiscretePosterior = gmm.discretePosterior(vv);
  EXPECT(assert_equal(expectedDiscretePosterior, gfg.discretePosterior(vv)));

  // Eliminate the graph!
  auto eliminationResultMax = gfg.eliminateSequential();

  // Equality of posteriors asserts that the elimination is correct (same ratios
  // for all modes)
  EXPECT(assert_equal(expectedDiscretePosterior,
                      eliminationResultMax->discretePosterior(vv)));

  auto pMax = *eliminationResultMax->at(0)->asDiscrete();
  EXPECT(assert_equal(DiscreteConditional(m, "42/58"), pMax, 1e-4));

  // Everywhere else, the result should be a bell curve like function.
  for (const double shift : {-4, -2, 0, 2, 4}) {
    const double z = zMax + shift;
    const double expected = prob_m_z(mu0, mu1, sigma0, sigma1, z);

    // Workflow 1: convert HBN to HFG and solve
    auto eliminationResult1 =
        gmm.toFactorGraph({{Z(0), Vector1(z)}}).eliminateSequential();
    auto posterior1 = *eliminationResult1->at(0)->asDiscrete();
    EXPECT_DOUBLES_EQUAL(expected, posterior1(m1Assignment), 1e-8);

    // Workflow 2: directly specify HFG and solve
    HybridGaussianFactorGraph hfg;
    hfg.emplace_shared<DecisionTreeFactor>(
        m, std::vector{Gaussian(mu0, sigma0, z), Gaussian(mu1, sigma1, z)});
    hfg.push_back(mixing);
    auto eliminationResult2 = hfg.eliminateSequential();
    auto posterior2 = *eliminationResult2->at(0)->asDiscrete();
    EXPECT_DOUBLES_EQUAL(expected, posterior2(m1Assignment), 1e-8);
  }
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
