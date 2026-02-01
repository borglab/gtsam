/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDiscreteBayesNet.cpp
 *
 *  @date Feb 27, 2011
 *  @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/debug.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/inference/Symbol.h>

#include <iostream>
#include <string>
#include <vector>

#include "AsiaExample.h"

using namespace gtsam;

/* ************************************************************************* */
TEST(DiscreteBayesNet, bayesNet) {
  using ADT = AlgebraicDecisionTree<Key>;
  DiscreteBayesNet bayesNet;
  DiscreteKey Parent(0, 2), Child(1, 2);

  auto prior = std::make_shared<DiscreteConditional>(Parent % "6/4");
  CHECK(assert_equal(ADT({Parent}, "0.6 0.4"), (ADT)*prior));
  bayesNet.push_back(prior);

  auto conditional =
      std::make_shared<DiscreteConditional>(Child | Parent = "7/3 8/2");
  EXPECT_LONGS_EQUAL(1, *(conditional->beginFrontals()));
  ADT expected(Child & Parent, "0.7 0.8 0.3 0.2");
  CHECK(assert_equal(expected, (ADT)*conditional));
  bayesNet.push_back(conditional);

  DiscreteFactorGraph fg(bayesNet);
  LONGS_EQUAL(2, fg.back()->size());

  // Check the marginals
  const double expectedMarginal[2]{0.4, 0.6 * 0.3 + 0.4 * 0.2};
  DiscreteMarginals marginals(fg);
  for (size_t j = 0; j < 2; j++) {
    Vector FT = marginals.marginalProbabilities(DiscreteKey(j, 2));
    EXPECT_DOUBLES_EQUAL(expectedMarginal[j], FT[1], 1e-3);
    EXPECT_DOUBLES_EQUAL(FT[0], 1.0 - FT[1], 1e-9);
  }
}

/* ************************************************************************* */
TEST(DiscreteBayesNet, Asia) {
  using namespace asia_example;
  const DiscreteBayesNet asia = createAsiaExample();

  // Convert to factor graph
  DiscreteFactorGraph fg(asia);
  LONGS_EQUAL(1, fg.back()->size());

  // Check the marginals we know (of the parent-less nodes)
  DiscreteMarginals marginals(fg);
  Vector2 va(0.99, 0.01), vs(0.5, 0.5);
  EXPECT(assert_equal(va, marginals.marginalProbabilities(Asia)));
  EXPECT(assert_equal(vs, marginals.marginalProbabilities(Smoking)));

  // Create solver and eliminate
  const Ordering ordering{A, D, T, X, S, E, L, B};
  DiscreteBayesNet::shared_ptr chordal = fg.eliminateSequential(ordering);
  DiscreteConditional expected2(Bronchitis % "11/9");
  EXPECT(assert_equal(expected2, *chordal->back()));

  // Check evaluate and logProbability
  auto result = fg.optimize();
  EXPECT_DOUBLES_EQUAL(asia.logProbability(result),
                       std::log(asia.evaluate(result)), 1e-9);

  // add evidence, we were in Asia and we have dyspnea
  fg.add(Asia, "0 1");
  fg.add(Dyspnea, "0 1");

  // solve again, now with evidence
  DiscreteBayesNet::shared_ptr chordal2 = fg.eliminateSequential(ordering);
  EXPECT(assert_equal(expected2, *chordal->back()));

  // now sample from it
  DiscreteValues expectedSample{{Asia.first, 1},       {Dyspnea.first, 1},
                                {XRay.first, 0},       {Tuberculosis.first, 0},
                                {Smoking.first, 1},    {Either.first, 0},
                                {LungCancer.first, 0}, {Bronchitis.first, 1}};
  SETDEBUG("DiscreteConditional::sample", false);
  auto actualSample = chordal2->sample();
  EXPECT(assert_equal(expectedSample, actualSample));
}

/* ************************************************************************* */
TEST(DiscreteBayesNet, Sugar) {
  DiscreteKey T(0, 2), L(1, 2), E(2, 2), C(8, 3), S(7, 2);

  DiscreteBayesNet bn;

  // try logic
  bn.add((E | T, L) = "OR");
  bn.add((E | T, L) = "AND");

  // try multivalued
  bn.add(C % "1/1/2");
  bn.add(C | S = "1/1/2 5/2/3");
}

/* ************************************************************************* */
// Test that pruning a DiscreteBayesNet results in a conditional whose leaves sum to 1.
TEST(DiscreteBayesNet, PruneSumToOne) {
  using namespace asia_example;
  const DiscreteBayesNet asiaBn = createAsiaExample();

  // Test with various numbers of max leaves.
  // Asia network has 8 binary variables, so 2^8 = 256 possible assignments in the full joint.
  std::vector<size_t> maxLeavesToTest = { 1, 2, 5, 10, 50, 100, 256, 500 };

  for (size_t maxLeaves : maxLeavesToTest) {
    // We expect maxLeaves >= 1 for the sum-to-one property to hold meaningfully.

    DiscreteBayesNet prunedBn = asiaBn.prune(maxLeaves);

    // If the original BN was not empty and maxLeaves >= 1,
    // the prunedBN should contain one conditional (the pruned joint).
    EXPECT(prunedBn.size() > 0);
    if (prunedBn.size() > 0) {
      EXPECT_LONGS_EQUAL(1, prunedBn.size());

      DiscreteConditional::shared_ptr prunedConditional = prunedBn.front();
      CHECK(prunedConditional); // Ensure it's not null

      double sumOfProbs = prunedConditional->sum();
      EXPECT_DOUBLES_EQUAL(1.0, sumOfProbs, 1e-8);
    }
  }
}

/* ************************************************************************* */
TEST(DiscreteBayesNet, Dot) {
  using namespace asia_example;
  const DiscreteBayesNet fragment = createFragment();

  std::string expected =
      "digraph {\n"
      "  size=\"5,5\";\n"
      "\n"
      "  var4683743612465315848[label=\"A8\"];\n"
      "  var4971973988617027587[label=\"E3\"];\n"
      "  var5476377146882523141[label=\"L5\"];\n"
      "  var5980780305148018695[label=\"S7\"];\n"
      "  var6052837899185946630[label=\"T6\"];\n"
      "\n"
      "  var4683743612465315848->var6052837899185946630\n"
      "  var5980780305148018695->var5476377146882523141\n"
      "  var6052837899185946630->var4971973988617027587\n"
      "  var5476377146882523141->var4971973988617027587\n"
      "}";
  std::string actual = fragment.dot();
  EXPECT(actual.compare(expected) == 0);
}

/* ************************************************************************* */
// Check markdown representation looks as expected.
TEST(DiscreteBayesNet, markdown) {
  using namespace asia_example;
  DiscreteBayesNet priors = createPriors();

  std::string expected =
      "`DiscreteBayesNet` of size 2\n"
      "\n"
      " *P(Smoking):*\n\n"
      "|Smoking|value|\n"
      "|:-:|:-:|\n"
      "|0|0.5|\n"
      "|1|0.5|\n"
      "\n"
      " *P(Asia):*\n\n"
      "|Asia|value|\n"
      "|:-:|:-:|\n"
      "|0|0.99|\n"
      "|1|0.01|\n\n";
  auto formatter = [](Key key) { return key == A ? "Asia" : "Smoking"; };
  std::string actual = priors.markdown(formatter);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
