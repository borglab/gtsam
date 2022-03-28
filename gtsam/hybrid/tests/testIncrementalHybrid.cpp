/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testIncrementalHybrid.cpp
 * @brief   Unit tests for incremental inference
 * @author  Fan Jiang, Varun Agrawal, Frank Dellaert
 * @date    Jan 2021
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/DCFactor.h>
#include <gtsam/hybrid/DCMixtureFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/IncrementalHybrid.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <numeric>

#include "Switching.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::L;
using symbol_shorthand::M;
using symbol_shorthand::W;
using symbol_shorthand::X;
using symbol_shorthand::Y;
using symbol_shorthand::Z;

/* ****************************************************************************/
// Test if we can incrementally do the inference
TEST(DCGaussianElimination, Incremental_inference) {
  Switching switching(3);
  IncrementalHybrid incrementalHybrid;
  GaussianHybridFactorGraph graph1;

  // Create initial factor graph
  // *- X1 -*-  X2 -*-X3
  //     \*-M1-*/
  graph1.push_back(switching.linearizedFactorGraph.dcGraph().at(0));
  graph1.push_back(switching.linearizedFactorGraph.gaussianGraph().at(0));
  graph1.push_back(switching.linearizedFactorGraph.gaussianGraph().at(1));
  graph1.push_back(switching.linearizedFactorGraph.gaussianGraph().at(2));

  // Create ordering.
  Ordering ordering;
  ordering += X(1);
  ordering += X(2);

  // Run update step
  incrementalHybrid.update(graph1, ordering);

  // Check that after update we have 2 hybrid Bayes net nodes:
  // P(X1|X2, M1) and P(X2|M1)
  auto hybridBayesNet = incrementalHybrid.hybridBayesNet();
  EXPECT_LONGS_EQUAL(2, hybridBayesNet.size());
  EXPECT(hybridBayesNet.at(0)->frontals() == KeyVector{X(1)});
  EXPECT(hybridBayesNet.at(0)->parents() == KeyVector({X(2), M(1)}));
  EXPECT(hybridBayesNet.at(1)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet.at(1)->parents() == KeyVector({M(1)}));

  // Check that the remaining factor graph has
  // a single decision tree factor on M1
  auto remainingFactorGraph = incrementalHybrid.remainingFactorGraph();
  EXPECT_LONGS_EQUAL(1, remainingFactorGraph.size());

  auto discreteFactor_m1 = *dynamic_pointer_cast<DecisionTreeFactor>(
      remainingFactorGraph.discreteGraph().at(0));
  EXPECT(discreteFactor_m1.keys() == KeyVector({M(1)}));

  /********************************************************/
  // New factor graph for incremental update.
  GaussianHybridFactorGraph graph2;

  graph2.push_back(
      switching.linearizedFactorGraph.dcGraph().at(1));  // p(x3 | x2, m2)
  graph2.push_back(switching.linearizedFactorGraph.gaussianGraph().at(3));

  // Create ordering.
  Ordering ordering2;
  ordering2 += X(2);
  ordering2 += X(3);

  incrementalHybrid.update(graph2, ordering2);

  // Check that after the second update we have
  // 2 additional hybrid Bayes net nodes:
  // P(X2|X3, M2, M1), and P(X3|M2, M1)
  auto hybridBayesNet2 = incrementalHybrid.hybridBayesNet();
  EXPECT_LONGS_EQUAL(3, hybridBayesNet2.size());
  EXPECT(hybridBayesNet2.at(1)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet2.at(1)->parents() == KeyVector({X(3), M(2), M(1)}));
  EXPECT(hybridBayesNet2.at(2)->frontals() == KeyVector{X(3)});
  EXPECT(hybridBayesNet2.at(2)->parents() == KeyVector({M(2), M(1)}));

  // Check that the remaining factor graph has a single decision tree factor on
  // M2 and M1.
  auto remainingFactorGraph2 = incrementalHybrid.remainingFactorGraph();
  EXPECT_LONGS_EQUAL(1, remainingFactorGraph2.size());
  auto discreteFactor = dynamic_pointer_cast<DecisionTreeFactor>(
      remainingFactorGraph2.discreteGraph().at(0));
  EXPECT(discreteFactor->keys() == KeyVector({M(2), M(1)}));

  /********************************************************/
  // Run batch elimination so we can compare results.
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
  EXPECT(assert_equal(*(hybridBayesNet.atGaussian(0)),
                      *(expectedHybridBayesNet->atGaussian(0))));

  // The densities on X(2) should be the same
  EXPECT(assert_equal(*(hybridBayesNet2.atGaussian(1)),
                      *(expectedHybridBayesNet->atGaussian(1))));

  // The densities on X(3) should be the same
  EXPECT(assert_equal(*(hybridBayesNet2.atGaussian(2)),
                      *(expectedHybridBayesNet->atGaussian(2))));

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
    auto x2_mixed =
        boost::dynamic_pointer_cast<GaussianMixture>(hybridBayesNet.at(1));
    gf.add(x2_mixed->factors()(m00));
    auto result_gf = gf.eliminateSequential();
    return gf.probPrime(result_gf->optimize());
  }();

  /// Test if the probability values are as expected with regression tests.
  DiscreteValues assignment;
  EXPECT(assert_equal(m00_prob, 0.60656, 1e-5));
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

  // Check if the chordal graph generated from incremental elimination matches
  // that of batch elimination.
  auto chordal = dfg.eliminateSequential();
  auto expectedChordal =
      expectedRemainingGraph->discreteGraph().eliminateSequential();

  EXPECT(assert_equal(*expectedChordal, *chordal, 1e-6));
}

/* ****************************************************************************/
// Test if we can approximately do the inference
TEST(DCGaussianElimination, Approx_inference) {
  Switching switching(4);
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

  // Now we calculate the actual factors using full elimination
  HybridBayesNet::shared_ptr unprunedHybridBayesNet;
  GaussianHybridFactorGraph::shared_ptr unprunedRemainingGraph;
  std::tie(unprunedHybridBayesNet, unprunedRemainingGraph) =
      switching.linearizedFactorGraph.eliminatePartialSequential(ordering);

  size_t maxComponents = 5;
  incrementalHybrid.update(graph1, ordering, maxComponents);

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

  pruned factors is:
      Choice(m3)
      0 Choice(m2)
      0 0 Leaf    0
      0 1 Choice(m1)
      0 1 0 Leaf 0.3742
      0 1 1 Leaf 0.6125
      1 Choice(m2)
      1 0 Choice(m1)
      1 0 0 Leaf    0
      1 0 1 Leaf 0.6124
      1 1 Choice(m1)
      1 1 0 Leaf 0.611
      1 1 1 Leaf    1
   */

  // Test that the remaining factor graph has one
  // DecisionTreeFactor on {M3, M2, M1}.
  auto remainingFactorGraph = incrementalHybrid.remainingFactorGraph();
  EXPECT_LONGS_EQUAL(1, remainingFactorGraph.size());

  auto discreteFactor_m1 = *dynamic_pointer_cast<DecisionTreeFactor>(
      incrementalHybrid.remainingDiscreteGraph().at(0));
  EXPECT(discreteFactor_m1.keys() == KeyVector({M(3), M(2), M(1)}));

  // Get the number of elements which are equal to 0.
  auto count = [](const double &value, int count) {
    return value > 0 ? count + 1 : count;
  };
  // Check that the number of leaves after pruning is 5.
  EXPECT_LONGS_EQUAL(5, discreteFactor_m1.fold(count, 0));

  /* Expected hybrid Bayes net
   * factor 0:  [x1 | x2 m1 ], 2 components
   * factor 1:  [x2 | x3 m2 m1 ], 4 components
   * factor 2:  [x3 | x4 m3 m2 m1 ], 8 components
   * factor 3:  [x4 | m3 m2 m1 ], 8 components
   */
  auto hybridBayesNet = incrementalHybrid.hybridBayesNet();

  // Check if we have a bayes net with 4 hybrid nodes,
  // each with 2, 4, 5 (pruned), and 5 (pruned) leaves respetively.
  EXPECT_LONGS_EQUAL(4, hybridBayesNet.size());
  EXPECT_LONGS_EQUAL(2, hybridBayesNet.atGaussian(0)->nrComponents());
  EXPECT_LONGS_EQUAL(4, hybridBayesNet.atGaussian(1)->nrComponents());
  EXPECT_LONGS_EQUAL(5, hybridBayesNet.atGaussian(2)->nrComponents());
  EXPECT_LONGS_EQUAL(5, hybridBayesNet.atGaussian(3)->nrComponents());

  // Check that the hybrid nodes of the bayes net match those of the bayes net
  // before pruning, at the same positions.
  auto &lastDensity = *(hybridBayesNet.atGaussian(3));
  auto &unprunedLastDensity = *(unprunedHybridBayesNet->atGaussian(3));
  std::vector<std::pair<DiscreteValues, double>> assignments =
      discreteFactor_m1.enumerate();
  // Loop over all assignments and check the pruned components
  for (auto &&av : assignments) {
    const DiscreteValues &assignment = av.first;
    const double value = av.second;

    if (value == 0.0) {
      EXPECT(lastDensity(assignment) == nullptr);
    } else {
      CHECK(lastDensity(assignment));
      EXPECT(assert_equal(*unprunedLastDensity(assignment),
                          *lastDensity(assignment)));
    }
  }
}

/* ****************************************************************************/
// Test approximate inference with an additional pruning step.
TEST(DCGaussianElimination, Incremental_approximate) {
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

  // Run update with pruning
  size_t maxComponents = 5;
  incrementalHybrid.update(graph1, ordering, maxComponents);

  // Check if we have a bayes net with 4 hybrid nodes,
  // each with 2, 4, 8, and 5 (pruned) leaves respetively.
  auto actualBayesNet1 = incrementalHybrid.hybridBayesNet();
  CHECK_EQUAL(4, actualBayesNet1.size());
  EXPECT_LONGS_EQUAL(2, actualBayesNet1.atGaussian(0)->nrComponents());
  EXPECT_LONGS_EQUAL(4, actualBayesNet1.atGaussian(1)->nrComponents());
  EXPECT_LONGS_EQUAL(8, actualBayesNet1.atGaussian(2)->nrComponents());
  EXPECT_LONGS_EQUAL(5, actualBayesNet1.atGaussian(3)->nrComponents());

  /***** Run Round 2 *****/
  GaussianHybridFactorGraph graph2;
  graph2.push_back(switching.linearizedFactorGraph.dcGraph().at(3));
  graph2.push_back(switching.linearizedFactorGraph.gaussianGraph().at(5));

  Ordering ordering2;
  ordering2 += X(4);
  ordering2 += X(5);

  // Run update with pruning a second time.
  incrementalHybrid.update(graph2, ordering2, maxComponents);

  // Check if we have a bayes net with 2 hybrid nodes,
  // each with 10 (pruned), and 5 (pruned) leaves respetively.
  auto actualBayesNet = incrementalHybrid.hybridBayesNet();
  CHECK_EQUAL(2, actualBayesNet.size());
  EXPECT_LONGS_EQUAL(10, actualBayesNet.atGaussian(0)->nrComponents());
  EXPECT_LONGS_EQUAL(5, actualBayesNet.atGaussian(1)->nrComponents());
}

/* ************************************************************************/
// Test for figuring out the optimal ordering to ensure we get
// a discrete graph after elimination.
TEST(IncrementalHybrid, NonTrivial) {
  // This is a GTSAM-only test for running inference on a single legged robot.
  // The leg links are represented by the chain X-Y-Z-W, where X is the base and
  // W is the foot.
  // We use BetweenFactor<Pose2> as constraints between each of the poses.

  /*************** Run Round 1 ***************/
  NonlinearHybridFactorGraph fg;

  // Add a prior on pose x1 at the origin.
  // A prior factor consists of a mean  and
  // a noise model (covariance matrix)
  Pose2 prior(0.0, 0.0, 0.0);  // prior mean is at origin
  auto priorNoise = noiseModel::Diagonal::Sigmas(
      Vector3(0.3, 0.3, 0.1));  // 30cm std on x,y, 0.1 rad on theta
  fg.emplace_nonlinear<PriorFactor<Pose2>>(X(0), prior, priorNoise);

  // create a noise model for the landmark measurements
  auto poseNoise = noiseModel::Isotropic::Sigma(3, 0.1);

  // We model a robot's single leg as X - Y - Z - W
  // where X is the base link and W is the foot link.

  // Add connecting poses similar to PoseFactors in GTD
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(X(0), Y(0), Pose2(0, 1.0, 0),
                                             poseNoise);
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(Y(0), Z(0), Pose2(0, 1.0, 0),
                                             poseNoise);
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(Z(0), W(0), Pose2(0, 1.0, 0),
                                             poseNoise);

  // Create initial estimate
  Values initial;
  initial.insert(X(0), Pose2(0.0, 0.0, 0.0));
  initial.insert(Y(0), Pose2(0.0, 1.0, 0.0));
  initial.insert(Z(0), Pose2(0.0, 2.0, 0.0));
  initial.insert(W(0), Pose2(0.0, 3.0, 0.0));

  GaussianHybridFactorGraph gfg = fg.linearize(initial);
  fg = NonlinearHybridFactorGraph();

  IncrementalHybrid inc;

  // Regular ordering going up the chain.
  Ordering ordering;
  ordering += W(0);
  ordering += Z(0);
  ordering += Y(0);
  ordering += X(0);

  // Update without pruning
  // The result is a HybridBayesNet with no discrete variables
  // (equivalent to a GaussianBayesNet).
  // Factorization is:
  // `P(X | measurements) = P(W0|Z0) P(Z0|Y0) P(Y0|X0) P(X0)`
  inc.update(gfg, ordering);

  /*************** Run Round 2 ***************/
  using PlanarMotionModel = BetweenFactor<Pose2>;

  // Add odometry factor with discrete modes.
  Pose2 odometry(1.0, 0.0, 0.0);
  KeyVector contKeys = {W(0), W(1)};
  auto noise_model = noiseModel::Isotropic::Sigma(3, 1.0);
  auto still = boost::make_shared<PlanarMotionModel>(W(0), W(1), Pose2(0, 0, 0),
                                                     noise_model),
       moving = boost::make_shared<PlanarMotionModel>(W(0), W(1), odometry,
                                                      noise_model);
  std::vector<PlanarMotionModel::shared_ptr> components = {moving, still};
  auto dcFactor = boost::make_shared<DCMixtureFactor<PlanarMotionModel>>(
      contKeys, DiscreteKeys{gtsam::DiscreteKey(M(1), 2)}, components);
  fg.push_back(dcFactor);

  // Add equivalent of ImuFactor
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(X(0), X(1), Pose2(1.0, 0.0, 0),
                                             poseNoise);
  // PoseFactors-like at k=1
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(X(1), Y(1), Pose2(0, 1, 0),
                                             poseNoise);
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(Y(1), Z(1), Pose2(0, 1, 0),
                                             poseNoise);
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(Z(1), W(1), Pose2(-1, 1, 0),
                                             poseNoise);

  initial.insert(X(1), Pose2(1.0, 0.0, 0.0));
  initial.insert(Y(1), Pose2(1.0, 1.0, 0.0));
  initial.insert(Z(1), Pose2(1.0, 2.0, 0.0));
  // The leg link did not move so we set the expected pose accordingly.
  initial.insert(W(1), Pose2(0.0, 3.0, 0.0));

  // Ordering for k=1.
  // This ordering follows the intuition that we eliminate the previous
  // timestep, and then the current timestep.
  ordering = Ordering();
  ordering += W(0);
  ordering += Z(0);
  ordering += Y(0);
  ordering += X(0);
  ordering += W(1);
  ordering += Z(1);
  ordering += Y(1);
  ordering += X(1);

  gfg = fg.linearize(initial);
  fg = NonlinearHybridFactorGraph();

  // Update without pruning
  // The result is a HybridBayesNet with 1 discrete variable M(1).
  // P(X | measurements) = P(W0|Z0, W1, M1) P(Z0|Y0, W1, M1) P(Y0|X0, W1, M1)
  //                       P(X0 | X1, W1, M1) P(W1|Z1, X1, M1) P(Z1|Y1, X1, M1)
  //                       P(Y1 | X1, M1)P(X1 | M1)P(M1)
  // The MHS tree is a 2 level tree for time indices (0, 1) with 2 leaves.
  inc.update(gfg, ordering);

  /*************** Run Round 3 ***************/
  // Add odometry factor with discrete modes.
  contKeys = {W(1), W(2)};
  still = boost::make_shared<PlanarMotionModel>(W(1), W(2), Pose2(0, 0, 0),
                                                noise_model);
  moving =
      boost::make_shared<PlanarMotionModel>(W(1), W(2), odometry, noise_model);
  components = {moving, still};
  dcFactor = boost::make_shared<DCMixtureFactor<PlanarMotionModel>>(
      contKeys, DiscreteKeys{gtsam::DiscreteKey(M(2), 2)}, components);
  fg.push_back(dcFactor);

  // Add equivalent of ImuFactor
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(X(1), X(2), Pose2(1.0, 0.0, 0),
                                             poseNoise);
  // PoseFactors-like at k=1
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(X(2), Y(2), Pose2(0, 1, 0),
                                             poseNoise);
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(Y(2), Z(2), Pose2(0, 1, 0),
                                             poseNoise);
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(Z(2), W(2), Pose2(-2, 1, 0),
                                             poseNoise);

  initial.insert(X(2), Pose2(2.0, 0.0, 0.0));
  initial.insert(Y(2), Pose2(2.0, 1.0, 0.0));
  initial.insert(Z(2), Pose2(2.0, 2.0, 0.0));
  initial.insert(W(2), Pose2(0.0, 3.0, 0.0));

  // Ordering at k=2. Same intuition as before.
  ordering = Ordering();
  ordering += W(1);
  ordering += Z(1);
  ordering += Y(1);
  ordering += X(1);
  ordering += W(2);
  ordering += Z(2);
  ordering += Y(2);
  ordering += X(2);

  gfg = fg.linearize(initial);
  fg = NonlinearHybridFactorGraph();

  // Now we prune!
  // P(X | measurements) = P(W0|Z0, W1, M1) P(Z0|Y0, W1, M1) P(Y0|X0, W1, M1) P(X0 | X1, W1, M1)
  //                       P(W1|W2, Z1, X1, M1, M2) P(Z1| W2, Y1, X1, M1, M2) P(Y1 | W2, X1, M1, M2)
  //                       P(X1 | W2, X2, M1, M2) P(W2|Z2, X2, M1, M2) P(Z2|Y2, X2, M1, M2)
  //                       P(Y2 | X2, M1, M2)P(X2 | M1, M2) P(M1, M2)
  // The MHS at this point should be a 3 level tree on (0, 1, 2).
  // 0 has 2 choices, 1 has 4 choices and 2 has 4 choices.
  inc.update(gfg, ordering, 2);

  /*************** Run Round 4 ***************/
  // Add odometry factor with discrete modes.
  contKeys = {W(2), W(3)};
  still = boost::make_shared<PlanarMotionModel>(W(2), W(3), Pose2(0, 0, 0),
                                                noise_model);
  moving =
      boost::make_shared<PlanarMotionModel>(W(2), W(3), odometry, noise_model);
  components = {moving, still};
  dcFactor = boost::make_shared<DCMixtureFactor<PlanarMotionModel>>(
      contKeys, DiscreteKeys{gtsam::DiscreteKey(M(3), 2)}, components);
  fg.push_back(dcFactor);

  // Add equivalent of ImuFactor
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(X(2), X(3), Pose2(1.0, 0.0, 0),
                                             poseNoise);
  // PoseFactors-like at k=3
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(X(3), Y(3), Pose2(0, 1, 0),
                                             poseNoise);
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(Y(3), Z(3), Pose2(0, 1, 0),
                                             poseNoise);
  fg.emplace_nonlinear<BetweenFactor<Pose2>>(Z(3), W(3), Pose2(-3, 1, 0),
                                             poseNoise);

  initial.insert(X(3), Pose2(3.0, 0.0, 0.0));
  initial.insert(Y(3), Pose2(3.0, 1.0, 0.0));
  initial.insert(Z(3), Pose2(3.0, 2.0, 0.0));
  initial.insert(W(3), Pose2(0.0, 3.0, 0.0));

  // Ordering at k=3. Same intuition as before.
  ordering = Ordering();
  ordering += W(2);
  ordering += Z(2);
  ordering += Y(2);
  ordering += X(2);
  ordering += W(3);
  ordering += Z(3);
  ordering += Y(3);
  ordering += X(3);

  gfg = fg.linearize(initial);
  fg = NonlinearHybridFactorGraph();

  // Keep pruning!
  inc.update(gfg, ordering, 3);

  // The final discrete graph should not be empty since we have eliminated
  // all continuous variables.
  EXPECT(!inc.remainingDiscreteGraph().empty());

  // Test if the optimal discrete mode assignment is (1, 1, 1).
  DiscreteValues optimal_assignment = inc.remainingDiscreteGraph().optimize();
  DiscreteValues expected_assignment;
  expected_assignment[M(1)] = 1;
  expected_assignment[M(2)] = 1;
  expected_assignment[M(3)] = 1;
  EXPECT(assert_equal(expected_assignment, optimal_assignment));

  // Test if pruning worked correctly by checking that we only have 3 leaves in
  // the last node.
  auto lastConditional = boost::dynamic_pointer_cast<GaussianMixture>(
      inc.hybridBayesNet().at(inc.hybridBayesNet().size() - 1));
  EXPECT_LONGS_EQUAL(3, lastConditional->nrComponents());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
