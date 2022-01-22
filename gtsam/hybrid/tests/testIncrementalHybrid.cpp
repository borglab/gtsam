/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testDCFactorGraph.cpp
 * @brief   Unit tests for DCFactorGraph
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include "Switching.h"

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/hybrid/DCFactor.h>
#include <gtsam/hybrid/DCMixtureFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/IncrementalHybrid.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <numeric>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ****************************************************************************/
// Test if we can incrementally do the inference
TEST(DCGaussianElimination, Incremental_inference) {
  Switching switching(3);

  IncrementalHybrid incrementalHybrid;

  GaussianHybridFactorGraph graph1;

  graph1.push_back(switching.linearizedFactorGraph.dcGraph().at(0));
  graph1.push_back(switching.linearizedFactorGraph.gaussianGraph().at(0));
  graph1.push_back(switching.linearizedFactorGraph.gaussianGraph().at(1));
  graph1.push_back(switching.linearizedFactorGraph.gaussianGraph().at(2));

  // Create ordering.
  Ordering ordering;
  ordering += X(1);
  ordering += X(2);

  incrementalHybrid.update(graph1, ordering);

  auto hybridBayesNet = incrementalHybrid.hybridBayesNet_;
  CHECK(hybridBayesNet);
  EXPECT_LONGS_EQUAL(2, hybridBayesNet->size());
  EXPECT(hybridBayesNet->at(0)->frontals() == KeyVector{X(1)});
  EXPECT(hybridBayesNet->at(0)->parents() == KeyVector({X(2), M(1)}));
  EXPECT(hybridBayesNet->at(1)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet->at(1)->parents() == KeyVector({M(1)}));

  auto remainingFactorGraph = incrementalHybrid.remainingFactorGraph_;
  CHECK(remainingFactorGraph);
  EXPECT_LONGS_EQUAL(1, remainingFactorGraph->size());

  auto discreteFactor_m1 = *dynamic_pointer_cast<DecisionTreeFactor>(
      remainingFactorGraph->discreteGraph().at(0));
  EXPECT(discreteFactor_m1.keys() == KeyVector({M(1)}));

  GaussianHybridFactorGraph graph2;

  graph2.push_back(
      switching.linearizedFactorGraph.dcGraph().at(1));  // p(x3 | x2, m2)
  graph2.push_back(switching.linearizedFactorGraph.gaussianGraph().at(3));

  // Create ordering.
  Ordering ordering2;
  ordering2 += X(2);
  ordering2 += X(3);

  incrementalHybrid.update(graph2, ordering2);

  auto hybridBayesNet2 = incrementalHybrid.hybridBayesNet_;
  CHECK(hybridBayesNet2);
  EXPECT_LONGS_EQUAL(2, hybridBayesNet2->size());
  EXPECT(hybridBayesNet2->at(0)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet2->at(0)->parents() == KeyVector({X(3), M(2), M(1)}));
  EXPECT(hybridBayesNet2->at(1)->frontals() == KeyVector{X(3)});
  EXPECT(hybridBayesNet2->at(1)->parents() == KeyVector({M(2), M(1)}));

  auto remainingFactorGraph2 = incrementalHybrid.remainingFactorGraph_;
  CHECK(remainingFactorGraph2);
  EXPECT_LONGS_EQUAL(1, remainingFactorGraph2->size());

  auto discreteFactor = dynamic_pointer_cast<DecisionTreeFactor>(
      remainingFactorGraph2->discreteGraph().at(0));
  EXPECT(discreteFactor->keys() == KeyVector({M(2), M(1)}));

  ordering.clear();
  ordering += X(1);
  ordering += X(2);
  ordering += X(3);

  // Now we calculate the actual factors using full elimination
  HybridBayesNet::shared_ptr expectedHybridBayesNet;
  GaussianHybridFactorGraph::shared_ptr expectedRemainingGraph;
  std::tie(expectedHybridBayesNet, expectedRemainingGraph) =
      switching.linearizedFactorGraph.eliminatePartialSequential(ordering);

  // The densities on X(1) should be the same
  EXPECT(
      assert_equal(*(hybridBayesNet->at(0)), *(expectedHybridBayesNet->at(0))));

  // The densities on X(2) should be the same
  EXPECT(assert_equal(*(hybridBayesNet2->at(0)),
                      *(expectedHybridBayesNet->at(1))));

  // The densities on X(3) should be the same
  EXPECT(assert_equal(*(hybridBayesNet2->at(1)),
                      *(expectedHybridBayesNet->at(2))));

  // we only do the manual continuous elimination for 0,0
  // the other discrete probabilities on M(2) are calculated the same way
  auto m00_prob = [&]() {
    GaussianFactorGraph gf;
    gf.add(switching.linearizedFactorGraph.gaussianGraph().at(3));

    DiscreteValues m00;
    m00[M(1)] = 0, m00[M(2)] = 0;
    auto dcMixture =
        dynamic_pointer_cast<DCGaussianMixtureFactor>(graph2.dcGraph().at(0));
    gf.add(dcMixture->factors()(m00));
    auto x2_mixed = hybridBayesNet->at(1);
    gf.add(x2_mixed->factors()(m00));
    auto result_gf = gf.eliminateSequential();
    return gf.probPrime(result_gf->optimize());
  }();

  EXPECT(assert_equal(m00_prob, 0.60656, 1e-5));

  DiscreteValues assignment;
  assignment[M(1)] = 0;
  assignment[M(2)] = 0;
  EXPECT(assert_equal(m00_prob, (*discreteFactor)(assignment), 1e-5));
  assignment[M(1)] = 1;
  assignment[M(2)] = 0;
  EXPECT(assert_equal(0.612477, (*discreteFactor)(assignment), 1e-5));
  assignment[M(1)] = 0;
  assignment[M(2)] = 1;
  EXPECT(assert_equal(0.999952, (*discreteFactor)(assignment), 1e-5));
  assignment[M(1)] = 1;
  assignment[M(2)] = 1;
  EXPECT(assert_equal(1.0, (*discreteFactor)(assignment), 1e-5));

  DiscreteFactorGraph dfg;
  dfg.add(*discreteFactor);
  dfg.add(discreteFactor_m1);
  dfg.add_factors(switching.linearizedFactorGraph.discreteGraph());

  auto chordal = dfg.eliminateSequential();
  auto expectedChordal =
      expectedRemainingGraph->discreteGraph().eliminateSequential();

  EXPECT(assert_equal(*expectedChordal, *chordal, 1e-6));
}

/* ****************************************************************************/
// Test if we can approximately do the inference
TEST(DCGaussianElimination, Approx_inference) {
  Switching switching(5);

  IncrementalHybrid incrementalHybrid;

  GaussianHybridFactorGraph graph1;

  // Add the 3 DC factors, x1-x2, x2-x3, x3-x4
  for (size_t i = 0; i < 3; i++) {
    graph1.push_back(switching.linearizedFactorGraph.dcGraph().at(i));
  }

  // Add the Gaussian factors, 1 prior on X(1), 4 measurements
  for (size_t i = 0; i <= 4; i++) {
    graph1.push_back(switching.linearizedFactorGraph.gaussianGraph().at(i));
  }

  // Create ordering.
  Ordering ordering;
  for (size_t j = 1; j <= 4; j++) {
    ordering += X(j);
  }

  size_t maxComponents = 5;
  incrementalHybrid.update(graph1, ordering, maxComponents);

  /* A hybrid Bayes net
   * factor 0:  [x1 | x2 m1 ], 2 components
   * factor 1:  [x2 | x3 m2 m1 ], 4 components
   * factor 2:  [x3 | x4 m3 m2 m1 ], 8 components
   * factor 3:  [x4 | m3 m2 m1 ], 8 components
  */
  auto hybridBayesNet = incrementalHybrid.hybridBayesNet_;

  CHECK(hybridBayesNet);
  EXPECT_LONGS_EQUAL(4, hybridBayesNet->size());
  EXPECT_LONGS_EQUAL(2, hybridBayesNet->at(0)->nrComponents());
  EXPECT_LONGS_EQUAL(4, hybridBayesNet->at(1)->nrComponents());
  EXPECT_LONGS_EQUAL(8, hybridBayesNet->at(2)->nrComponents());
  EXPECT_LONGS_EQUAL(5, hybridBayesNet->at(3)->nrComponents());

  GTSAM_PRINT(*hybridBayesNet);

  /*
   unpruned factor is:
       Choice(m3)
       0 Choice(m2)
       0 0 Choice(m1)
       0 0 0 Leaf 0.2248 -
       0 0 1 Leaf 0.3715 -
       0 1 Choice(m1)
       0 1 0 Leaf 0.3742 *
       0 1 1 Leaf 0.6125 *
       1 Choice(m2)
       1 0 Choice(m1)
       1 0 0 Leaf 0.3706 -
       1 0 1 Leaf 0.6124 *
       1 1 Choice(m1)
       1 1 0 Leaf 0.611 *
       1 1 1 Leaf    1 *
   */
  auto remainingFactorGraph = incrementalHybrid.remainingFactorGraph_;
  CHECK(remainingFactorGraph);
  EXPECT_LONGS_EQUAL(1, remainingFactorGraph->size());

  GTSAM_PRINT(*remainingFactorGraph);

  auto discreteFactor_m1 = *dynamic_pointer_cast<DecisionTreeFactor>(
      remainingFactorGraph->discreteGraph().at(0));
  EXPECT(discreteFactor_m1.keys() == KeyVector({M(1)}));
  EXPECT_LONGS_EQUAL(5, discreteFactor_m1.nrLeaves());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
