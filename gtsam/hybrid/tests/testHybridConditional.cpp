/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridConditional.cpp
 * @brief   Unit tests for HybridConditional class
 * @date    January 2023
 */

#include <gtsam/hybrid/HybridConditional.h>

#include "TinyHybridExample.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

/* ****************************************************************************/
// Check invariants for all conditionals in a tiny Bayes net.
TEST(HybridConditional, Invariants) {
  // Create hybrid Bayes net p(z|x,m)p(x)P(m)
  auto bn = tiny::createHybridBayesNet();

  // Create values to check invariants.
  const VectorValues c{{X(0), Vector1(5.1)}, {Z(0), Vector1(4.9)}};
  const DiscreteValues d{{M(0), 1}};
  const HybridValues values{c, d};

  // Check invariants for p(z|x,m)
  auto hc1 = bn.at(0);
  CHECK(hc1->isHybrid());
  GTSAM_PRINT(*hc1);

  // Check invariants as a GaussianMixture.
  const auto mixture = hc1->asMixture();
  double probability = mixture->evaluate(values);
  CHECK(probability >= 0.0);
  EXPECT_DOUBLES_EQUAL(probability, (*mixture)(values), 1e-9);
  double logProb = mixture->logProbability(values);
  EXPECT_DOUBLES_EQUAL(probability, std::exp(logProb), 1e-9);
  double expected =
      mixture->logNormalizationConstant() - mixture->error(values);
  EXPECT_DOUBLES_EQUAL(logProb, expected, 1e-9);
  EXPECT(GaussianMixture::CheckInvariants(*mixture, values));

  // Check invariants as a HybridConditional.
  probability = hc1->evaluate(values);
  CHECK(probability >= 0.0);
  EXPECT_DOUBLES_EQUAL(probability, (*hc1)(values), 1e-9);
  logProb = hc1->logProbability(values);
  EXPECT_DOUBLES_EQUAL(probability, std::exp(logProb), 1e-9);
  expected = hc1->logNormalizationConstant() - hc1->error(values);
  EXPECT_DOUBLES_EQUAL(logProb, expected, 1e-9);
  EXPECT(HybridConditional::CheckInvariants(*hc1, values));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
