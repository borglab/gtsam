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

/**
 * Closed form computation of P(m=1|z).
 * If sigma0 == sigma1, it simplifies to a sigmoid function.
 */
static double prob_m_z(double mu0, double mu1, double sigma0, double sigma1,
                       double z) {
  double x1 = ((z - mu0) / sigma0), x2 = ((z - mu1) / sigma1);
  double d = sigma1 / sigma0;
  double e = d * std::exp(-0.5 * (x1 * x1 - x2 * x2));
  return 1 / (1 + e);
};

// Define mode key and an assignment m==1
static const DiscreteKey m(M(0), 2);
static const DiscreteValues m1Assignment{{M(0), 1}};

/**
 * Create a simple Gaussian Mixture Model represented as p(z|m)P(m)
 * where m is a discrete variable and z is a continuous variable.
 * The "mode" m is binary and depending on m, we have 2 different means
 * μ1 and μ2 for the Gaussian density p(z|m).
 */
static HybridBayesNet GetGaussianMixtureModel(double mu0, double mu1,
                                              double sigma0, double sigma1) {
  auto model0 = noiseModel::Isotropic::Sigma(1, sigma0);
  auto model1 = noiseModel::Isotropic::Sigma(1, sigma1);

  auto c0 = make_shared<GaussianConditional>(Z(0), Vector1(mu0), I_1x1, model0),
       c1 = make_shared<GaussianConditional>(Z(0), Vector1(mu1), I_1x1, model1);

  HybridBayesNet hbn;
  hbn.emplace_shared<HybridGaussianConditional>(KeyVector{Z(0)}, KeyVector{}, m,
                                                std::vector{c0, c1});

  auto mixing = make_shared<DiscreteConditional>(m, "50/50");
  hbn.push_back(mixing);

  return hbn;
}

/// Given p(z,m) and z, use eliminate to obtain P(m|z).
static DiscreteConditional solveForMeasurement(const HybridBayesNet &hbn,
                                               double z) {
  VectorValues given;
  given.insert(Z(0), Vector1(z));

  HybridGaussianFactorGraph gfg = hbn.toFactorGraph(given);
  return *gfg.eliminateSequential()->at(0)->asDiscrete();
}

/*
 * Test a Gaussian Mixture Model P(m)p(z|m) with same sigma.
 * The posterior, as a function of z, should be a sigmoid function.
 */
TEST(HybridGaussianFactor, GaussianMixtureModel) {
  double mu0 = 1.0, mu1 = 3.0;
  double sigma = 2.0;

  auto hbn = GetGaussianMixtureModel(mu0, mu1, sigma, sigma);

  // At the halfway point between the means, we should get P(m|z)=0.5
  double midway = mu1 - mu0;
  auto pMid = solveForMeasurement(hbn, midway);
  EXPECT(assert_equal(DiscreteConditional(m, "50/50"), pMid));

  // Everywhere else, the result should be a sigmoid.
  for (const double shift : {-4, -2, 0, 2, 4}) {
    const double z = midway + shift;
    const double expected = prob_m_z(mu0, mu1, sigma, sigma, z);

    auto posterior = solveForMeasurement(hbn, z);
    EXPECT_DOUBLES_EQUAL(expected, posterior(m1Assignment), 1e-8);
  }
}

/*
 * Test a Gaussian Mixture Model P(m)p(z|m) with different sigmas.
 * The posterior, as a function of z, should be a unimodal function.
 */
TEST(HybridGaussianFactor, GaussianMixtureModel2) {
  double mu0 = 1.0, mu1 = 3.0;
  double sigma0 = 8.0, sigma1 = 4.0;

  auto hbn = GetGaussianMixtureModel(mu0, mu1, sigma0, sigma1);

  // We get zMax=3.1333 by finding the maximum value of the function, at which
  // point the mode m==1 is about twice as probable as m==0.
  double zMax = 3.133;
  auto pMax = solveForMeasurement(hbn, zMax);
  EXPECT(assert_equal(DiscreteConditional(m, "32.56/67.44"), pMax, 1e-5));

  // Everywhere else, the result should be a bell curve like function.
  for (const double shift : {-4, -2, 0, 2, 4}) {
    const double z = zMax + shift;
    const double expected = prob_m_z(mu0, mu1, sigma0, sigma1, z);

    auto posterior = solveForMeasurement(hbn, z);
    EXPECT_DOUBLES_EQUAL(expected, posterior(m1Assignment), 1e-8);
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
