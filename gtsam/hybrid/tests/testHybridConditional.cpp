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

  GTSAM_PRINT(bn);

  // Check invariants for p(z|x,m)
  auto hc0 = bn.at(0);
  CHECK(hc0->isHybrid());

  // Check invariants as a HybridGaussianConditional.
  const auto conditional = hc0->asHybrid();
  EXPECT(HybridGaussianConditional::CheckInvariants(*conditional, values));

  // Check invariants as a HybridConditional.
  EXPECT(HybridConditional::CheckInvariants(*hc0, values));

  // Check invariants for p(x)
  auto hc1 = bn.at(1);
  CHECK(hc1->isContinuous());

  // Check invariants as a GaussianConditional.
  const auto gaussian = hc1->asGaussian();
  EXPECT(GaussianConditional::CheckInvariants(*gaussian, c));
  EXPECT(GaussianConditional::CheckInvariants(*gaussian, values));

  // Check invariants as a HybridConditional.
  EXPECT(HybridConditional::CheckInvariants(*hc1, values));

  // Check invariants for p(m)
  auto hc2 = bn.at(2);
  CHECK(hc2->isDiscrete());

  // Check invariants as a DiscreteConditional.
  const auto discrete = hc2->asDiscrete();
  EXPECT(DiscreteConditional::CheckInvariants(*discrete, d));
  EXPECT(DiscreteConditional::CheckInvariants(*discrete, values));

  // Check invariants as a HybridConditional.
  EXPECT(HybridConditional::CheckInvariants(*hc2, values));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
