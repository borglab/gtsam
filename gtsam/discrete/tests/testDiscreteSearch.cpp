/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDiscreteSearch.cpp
 *
 *  @date January, 2025
 *  @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteSearch.h>

#include "AsiaExample.h"

using namespace gtsam;

// Create Asia Bayes net, FG, and Bayes tree once
namespace asia {
using namespace asia_example;
static const DiscreteBayesNet bayesNet = createAsiaExample();

// Create factor graph and optimize with max-product for MPE
static const DiscreteFactorGraph factorGraph(bayesNet);
static const DiscreteValues mpe = factorGraph.optimize();

// Create ordering
static const Ordering ordering{D, X, B, E, L, T, S, A};

// Create Bayes tree
static const DiscreteBayesTree bayesTree =
    *factorGraph.eliminateMultifrontal(ordering);
}  // namespace asia

/* ************************************************************************* */
TEST(DiscreteBayesNet, EmptyKBest) {
  DiscreteBayesNet net;  // no factors
  DiscreteSearch search(net);
  auto solutions = search.run(3);
  // Expect one solution with empty assignment, error=0
  EXPECT_LONGS_EQUAL(1, solutions.size());
  EXPECT_DOUBLES_EQUAL(0, std::fabs(solutions[0].error), 1e-9);
}

/* ************************************************************************* */
TEST(DiscreteBayesTree, EmptyTree) {
  DiscreteBayesTree bt;

  DiscreteSearch search(bt);
  auto solutions = search.run(3);

  // We expect exactly 1 solution with error = 0.0 (the empty assignment).
  EXPECT_LONGS_EQUAL(1, solutions.size());
  EXPECT_DOUBLES_EQUAL(0, std::fabs(solutions[0].error), 1e-9);
}

/* ************************************************************************* */
TEST(DiscreteBayesNet, AsiaKBest) {
  auto fromETree =
      DiscreteSearch::FromFactorGraph(asia::factorGraph, asia::ordering);
  auto fromJunctionTree =
      DiscreteSearch::FromFactorGraph(asia::factorGraph, asia::ordering, true);
  const DiscreteSearch fromBayesNet(asia::bayesNet);
  const DiscreteSearch fromBayesTree(asia::bayesTree);

  for (auto& search :
       {fromETree, fromJunctionTree, fromBayesNet, fromBayesTree}) {
    // Ask for the MPE
    auto mpe = search.run();

    // Regression on error lower bound
    EXPECT_DOUBLES_EQUAL(1.205536, search.lowerBound(), 1e-5);

    // Check that the cost-to-go heuristic decreases from there
    auto slots = search.slots();
    double previousHeuristic = search.lowerBound();
    for (auto&& slot : slots) {
      EXPECT(slot.heuristic <= previousHeuristic);
      previousHeuristic = slot.heuristic;
    }

    EXPECT_LONGS_EQUAL(1, mpe.size());
    // Regression test: check the MPE solution
    EXPECT_DOUBLES_EQUAL(1.236627, std::fabs(mpe[0].error), 1e-5);

    // Check it is equal to MPE via inference
    EXPECT(assert_equal(asia::mpe, mpe[0].assignment));

    // Ask for top 4 solutions
    auto solutions = search.run(4);

    EXPECT_LONGS_EQUAL(4, solutions.size());
    // Regression test: check the first and last solution
    EXPECT_DOUBLES_EQUAL(1.236627, std::fabs(solutions[0].error), 1e-5);
    EXPECT_DOUBLES_EQUAL(2.201708, std::fabs(solutions[3].error), 1e-5);
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
