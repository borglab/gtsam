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

#include "gtsam/hybrid/DCMixtureFactor.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/******************************************************************************/

/*
 * Test DCDiscreteFactor using a simple mixture.
 *
 * Construct a single factor (for a single variable) consisting of a
 * discrete-conditional mixture. Here we have a "null hypothesis" consisting of
 * a Gaussian with large variance and an "alternative hypothesis" consisting of
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
  EXPECT_DOUBLES_EQUAL(2.2, dcMixture.error(continuousVals, discreteVals), 1e-1);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
