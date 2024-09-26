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

/**
 * Create a simple Gaussian Mixture Model represented as p(z|m)P(m)
 * where m is a discrete variable and z is a continuous variable.
 * The "mode" m is binary and depending on m, we have 2 different means
 * μ1 and μ2 for the Gaussian density p(z|m).
 */
HybridBayesNet GaussianMixtureModel(double mu0, double mu1, double sigma0,
                                    double sigma1) {
  HybridBayesNet hbn;
  auto model0 = noiseModel::Isotropic::Sigma(1, sigma0);
  auto model1 = noiseModel::Isotropic::Sigma(1, sigma1);
  auto c0 = std::make_shared<GaussianConditional>(Z(0), Vector1(mu0), I_1x1,
                                                  model0),
       c1 = std::make_shared<GaussianConditional>(Z(0), Vector1(mu1), I_1x1,
                                                  model1);
  hbn.emplace_shared<HybridGaussianConditional>(m, std::vector{c0, c1});
  hbn.push_back(mixing);
  return hbn;
}

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

/// Given \phi(m;z)\phi(m) use eliminate to obtain P(m|z).
DiscreteConditional SolveHFG(const HybridGaussianFactorGraph &hfg) {
  return *hfg.eliminateSequential()->at(0)->asDiscrete();
}

/// Given p(z,m) and z, convert to HFG and solve.
DiscreteConditional SolveHBN(const HybridBayesNet &hbn, double z) {
  VectorValues given{{Z(0), Vector1(z)}};
  return SolveHFG(hbn.toFactorGraph(given));
}

/*
 * Test a Gaussian Mixture Model P(m)p(z|m) with same sigma.
 * The posterior, as a function of z, should be a sigmoid function.
 */
TEST(GaussianMixture, GaussianMixtureModel) {
  double mu0 = 1.0, mu1 = 3.0;
  double sigma = 2.0;

  auto hbn = GaussianMixtureModel(mu0, mu1, sigma, sigma);

  // Check the number of keys matches what we expect
  auto hgc = hbn.at(0)->asHybrid();
  EXPECT_LONGS_EQUAL(2, hgc->keys().size());
  EXPECT_LONGS_EQUAL(1, hgc->continuousKeys().size());
  EXPECT_LONGS_EQUAL(1, hgc->discreteKeys().size());

  // At the halfway point between the means, we should get P(m|z)=0.5
  double midway = mu1 - mu0;
  auto pMid = SolveHBN(hbn, midway);
  EXPECT(assert_equal(DiscreteConditional(m, "60/40"), pMid));

  // Everywhere else, the result should be a sigmoid.
  for (const double shift : {-4, -2, 0, 2, 4}) {
    const double z = midway + shift;
    const double expected = prob_m_z(mu0, mu1, sigma, sigma, z);

    // Workflow 1: convert HBN to HFG and solve
    auto posterior1 = SolveHBN(hbn, z);
    EXPECT_DOUBLES_EQUAL(expected, posterior1(m1Assignment), 1e-8);

    // Workflow 2: directly specify HFG and solve
    HybridGaussianFactorGraph hfg1;
    hfg1.emplace_shared<DecisionTreeFactor>(
        m, std::vector{Gaussian(mu0, sigma, z), Gaussian(mu1, sigma, z)});
    hfg1.push_back(mixing);
    auto posterior2 = SolveHFG(hfg1);
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

  auto hbn = GaussianMixtureModel(mu0, mu1, sigma0, sigma1);

  // Check the number of keys matches what we expect
  auto hgc = hbn.at(0)->asHybrid();
  EXPECT_LONGS_EQUAL(2, hgc->keys().size());
  EXPECT_LONGS_EQUAL(1, hgc->continuousKeys().size());
  EXPECT_LONGS_EQUAL(1, hgc->discreteKeys().size());

  // We get zMax=3.1333 by finding the maximum value of the function, at which
  // point the mode m==1 is about twice as probable as m==0.
  double zMax = 3.133;
  auto pMax = SolveHBN(hbn, zMax);
  EXPECT(assert_equal(DiscreteConditional(m, "42/58"), pMax, 1e-4));

  // Everywhere else, the result should be a bell curve like function.
  for (const double shift : {-4, -2, 0, 2, 4}) {
    const double z = zMax + shift;
    const double expected = prob_m_z(mu0, mu1, sigma0, sigma1, z);

    // Workflow 1: convert HBN to HFG and solve
    auto posterior1 = SolveHBN(hbn, z);
    EXPECT_DOUBLES_EQUAL(expected, posterior1(m1Assignment), 1e-8);

    // Workflow 2: directly specify HFG and solve
    HybridGaussianFactorGraph hfg;
    hfg.emplace_shared<DecisionTreeFactor>(
        m, std::vector{Gaussian(mu0, sigma0, z), Gaussian(mu1, sigma1, z)});
    hfg.push_back(mixing);
    auto posterior2 = SolveHFG(hfg);
    EXPECT_DOUBLES_EQUAL(expected, posterior2(m1Assignment), 1e-8);
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
