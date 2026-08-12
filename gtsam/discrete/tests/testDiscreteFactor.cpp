/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDiscreteFactor.cpp
 *
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;
using symbol_shorthand::D;

/* ************************************************************************* */
TEST(DisreteKeys, Serialization) {
  DiscreteKeys keys;
  keys& DiscreteKey(0, 2);
  keys& DiscreteKey(1, 3);
  keys& DiscreteKey(2, 4);

  EXPECT(equalsObj<DiscreteKeys>(keys));
  EXPECT(equalsXML<DiscreteKeys>(keys));
  EXPECT(equalsBinary<DiscreteKeys>(keys));
}

/******************************************************************************/
/*
 * Test simple DiscretePriorFactor. We construct a DiscreteFactor factor
 * graph with a single binary variable d1 and a single custom
 * DiscretePriorFactor p(d1) specifying the distribution:
 * p(d1 = 0) = 0.1, p(d1 = 1) = 0.9.
 *
 * After constructing the factor graph, we validate that the marginals are
 * correct (i.e. they match the input p(d1)) and the most probable estimate
 * (computed by maximizing p(d1) over possible values of d1) is 1.
 */
TEST(DCSAM, DiscretePriorFactor) {
  // Make an empty discrete factor graph
  DiscreteFactorGraph dfg;

  // We'll make a variable with 2 possible assignments
  const size_t cardinality = 2;
  DiscreteKey dk(D(1), cardinality);
  const std::vector<double> probs{0.1, 0.9};

  // Make a discrete prior factor and add it to the graph
  DecisionTreeFactor dpf(dk, probs);
  dfg.push_back(dpf);

  // Solve
  DiscreteValues mostProbableEstimate = dfg.optimize();

  // Get the most probable estimate
  const size_t mpeD = mostProbableEstimate.at(dk.first);

  // Get the marginals
  DiscreteMarginals discreteMarginals(dfg);
  Vector margProbs = discreteMarginals.marginalProbabilities(dk);

  // Verify that each marginal probability is within `tol` of the true marginal
  for (size_t i = 0; i < dk.second; i++) {
    bool margWithinTol = (abs(margProbs[i] - probs[i]) < 1e-7);
    EXPECT(margWithinTol);
  }

  // Ensure that the most probable estimate is correct
  EXPECT_LONGS_EQUAL(mpeD, 1);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
