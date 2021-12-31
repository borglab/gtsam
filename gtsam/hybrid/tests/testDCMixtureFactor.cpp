/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testDCMixtureFactor.cpp
 * @brief   Unit tests for DCMixtureFactor
 * @author  Kevin Doherty, kdoherty@mit.edu
 * @date    December 2021
 */

#include <gtsam/hybrid/DCMixtureFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/******************************************************************************/

/*
 * Test DCDiscreteFactor using a simple mixture.
 *
 * Construct a single factor (for a single variable) consisting of a
 * discrete-conditional mixture. Here we have a "null hypothesis" consisting
 of
 * a Gaussian with large variance and an "alternative hypothesis" consisting
 of
 * a Gaussian with smaller variance.
 */
TEST(TestSuite, dcdiscrete_mixture) {
  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  DiscreteKey dk(Symbol('d', 1), cardinality);

  // Make a symbol for a single continuous variable and add to KeyVector
  Symbol x1 = Symbol('x', 1);
  KeyVector keys;
  keys.push_back(x1);

  // Make a factor for non-null hypothesis
  const double loc = 0.0;
  const double sigma1 = 1.0;
  auto prior_noise1 = noiseModel::Isotropic::Sigma(1, sigma1);
  PriorFactor<double> f1(x1, loc, prior_noise1);

  // Make a factor for null hypothesis
  const double sigmaNullHypo = 8.0;
  auto prior_noiseNullHypo = noiseModel::Isotropic::Sigma(1, sigmaNullHypo);
  PriorFactor<double> fNullHypo(x1, loc, prior_noiseNullHypo);

  DCMixtureFactor<PriorFactor<double>> dcMixture(keys, dk, {f1, fNullHypo});

  // Check error.
  Values continuousVals;
  continuousVals.insert(x1, -2.5);
  DiscreteValues discreteVals;
  discreteVals[dk.first] = 0;

  // regression
  EXPECT_DOUBLES_EQUAL(2.2, dcMixture.error(continuousVals, discreteVals),
                       1e-1);
}

TEST(DCMixtureFactor, Error) {
  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  DiscreteKey dk(Symbol('d', 1), cardinality);
  DiscreteKeys dKeys;
  dKeys.push_back(dk);

  // Make a symbol for a single continuous variable and add to KeyVector
  Symbol x1 = Symbol('x', 1);
  KeyVector keys;
  keys.push_back(x1);

  // Make a factor for non-null hypothesis which has mu=1.0.
  auto prior_noise1 = noiseModel::Isotropic::Sigma(1, 1.0);
  PriorFactor<double> f1(x1, 1.0, prior_noise1);

  // Make a factor for null hypothesis with mu = 0.0.
  auto prior_noiseNullHypo = noiseModel::Isotropic::Sigma(1, 8.0);
  PriorFactor<double> fNullHypo(x1, 0.0, prior_noiseNullHypo);

  // Create the factor to test. We set normalize to true so that expected values
  // are easy to compute manually (they should be 0.0)
  DCMixtureFactor<PriorFactor<double>> factor(keys, dKeys, {fNullHypo, f1},
                                              true);

  Values continuousValues;
  DiscreteValues discreteValues;

  // Test the null hypothesis
  continuousValues.insert(x1, 0.0);
  discreteValues[dk.first] = 0;
  double error = factor.error(continuousValues, discreteValues);
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-9);

  // Test the alternate hypothesis
  continuousValues.update(x1, 1.0);
  discreteValues[dk.first] = 1;
  error = factor.error(continuousValues, discreteValues);
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-9);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
