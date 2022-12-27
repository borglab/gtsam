/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridNonlinearFactorGraph.cpp
 * @brief   Unit tests for HybridNonlinearFactorGraph
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/MixtureFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <numeric>

#include "Switching.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::L;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ****************************************************************************
 * Test that any linearizedFactorGraph gaussian factors are appended to the
 * existing gaussian factor graph in the hybrid factor graph.
 */
TEST(HybridFactorGraph, GaussianFactorGraph) {
  HybridNonlinearFactorGraph fg;

  // Add a simple prior factor to the nonlinear factor graph
  fg.emplace_nonlinear<PriorFactor<double>>(X(0), 0, Isotropic::Sigma(1, 0.1));

  // Linearization point
  Values linearizationPoint;
  linearizationPoint.insert<double>(X(0), 0);

  HybridGaussianFactorGraph ghfg = *fg.linearize(linearizationPoint);

  // Add a factor to the GaussianFactorGraph
  ghfg.add(JacobianFactor(X(0), I_1x1, Vector1(5)));

  EXPECT_LONGS_EQUAL(2, ghfg.size());
}

/***************************************************************************
 * Test equality for Hybrid Nonlinear Factor Graph.
 */
TEST(HybridNonlinearFactorGraph, Equals) {
  HybridNonlinearFactorGraph graph1;
  HybridNonlinearFactorGraph graph2;

  // Test empty factor graphs
  EXPECT(assert_equal(graph1, graph2));

  auto f0 = boost::make_shared<PriorFactor<Pose2>>(
      1, Pose2(), noiseModel::Isotropic::Sigma(3, 0.001));
  graph1.push_back(f0);
  graph2.push_back(f0);

  auto f1 = boost::make_shared<BetweenFactor<Pose2>>(
      1, 2, Pose2(), noiseModel::Isotropic::Sigma(3, 0.1));
  graph1.push_back(f1);
  graph2.push_back(f1);

  // Test non-empty graphs
  EXPECT(assert_equal(graph1, graph2));
}

/***************************************************************************
 * Test that the resize method works correctly for a HybridNonlinearFactorGraph.
 */
TEST(HybridNonlinearFactorGraph, Resize) {
  HybridNonlinearFactorGraph fg;
  auto nonlinearFactor = boost::make_shared<BetweenFactor<double>>();
  fg.push_back(nonlinearFactor);

  auto discreteFactor = boost::make_shared<DecisionTreeFactor>();
  fg.push_back(discreteFactor);

  auto dcFactor = boost::make_shared<MixtureFactor>();
  fg.push_back(dcFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 3);

  fg.resize(0);
  EXPECT_LONGS_EQUAL(fg.size(), 0);
}

/***************************************************************************
 * Test that the resize method works correctly for a
 * HybridGaussianFactorGraph.
 */
TEST(HybridGaussianFactorGraph, Resize) {
  HybridNonlinearFactorGraph nhfg;
  auto nonlinearFactor = boost::make_shared<BetweenFactor<double>>(
      X(0), X(1), 0.0, Isotropic::Sigma(1, 0.1));
  nhfg.push_back(nonlinearFactor);
  auto discreteFactor = boost::make_shared<DecisionTreeFactor>();
  nhfg.push_back(discreteFactor);

  KeyVector contKeys = {X(0), X(1)};
  auto noise_model = noiseModel::Isotropic::Sigma(1, 1.0);
  auto still = boost::make_shared<MotionModel>(X(0), X(1), 0.0, noise_model),
       moving = boost::make_shared<MotionModel>(X(0), X(1), 1.0, noise_model);

  std::vector<MotionModel::shared_ptr> components = {still, moving};
  auto dcFactor = boost::make_shared<MixtureFactor>(
      contKeys, DiscreteKeys{gtsam::DiscreteKey(M(1), 2)}, components);
  nhfg.push_back(dcFactor);

  Values linearizationPoint;
  linearizationPoint.insert<double>(X(0), 0);
  linearizationPoint.insert<double>(X(1), 1);

  // Generate `HybridGaussianFactorGraph` by linearizing
  HybridGaussianFactorGraph gfg = *nhfg.linearize(linearizationPoint);

  EXPECT_LONGS_EQUAL(gfg.size(), 3);

  gfg.resize(0);
  EXPECT_LONGS_EQUAL(gfg.size(), 0);
}

/***************************************************************************
 * Test that the MixtureFactor reports correctly if the number of continuous
 * keys provided do not match the keys in the factors.
 */
TEST(HybridGaussianFactorGraph, MixtureFactor) {
  auto nonlinearFactor = boost::make_shared<BetweenFactor<double>>(
      X(0), X(1), 0.0, Isotropic::Sigma(1, 0.1));
  auto discreteFactor = boost::make_shared<DecisionTreeFactor>();

  auto noise_model = noiseModel::Isotropic::Sigma(1, 1.0);
  auto still = boost::make_shared<MotionModel>(X(0), X(1), 0.0, noise_model),
       moving = boost::make_shared<MotionModel>(X(0), X(1), 1.0, noise_model);

  std::vector<MotionModel::shared_ptr> components = {still, moving};

  // Check for exception when number of continuous keys are under-specified.
  KeyVector contKeys = {X(0)};
  THROWS_EXCEPTION(boost::make_shared<MixtureFactor>(
      contKeys, DiscreteKeys{gtsam::DiscreteKey(M(1), 2)}, components));

  // Check for exception when number of continuous keys are too many.
  contKeys = {X(0), X(1), X(2)};
  THROWS_EXCEPTION(boost::make_shared<MixtureFactor>(
      contKeys, DiscreteKeys{gtsam::DiscreteKey(M(1), 2)}, components));
}

/*****************************************************************************
 * Test push_back on HFG makes the correct distinction.
 */
TEST(HybridFactorGraph, PushBack) {
  HybridNonlinearFactorGraph fg;

  auto nonlinearFactor = boost::make_shared<BetweenFactor<double>>();
  fg.push_back(nonlinearFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 1);

  fg = HybridNonlinearFactorGraph();

  auto discreteFactor = boost::make_shared<DecisionTreeFactor>();
  fg.push_back(discreteFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 1);

  fg = HybridNonlinearFactorGraph();

  auto dcFactor = boost::make_shared<MixtureFactor>();
  fg.push_back(dcFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 1);

  // Now do the same with HybridGaussianFactorGraph
  HybridGaussianFactorGraph ghfg;

  auto gaussianFactor = boost::make_shared<JacobianFactor>();
  ghfg.push_back(gaussianFactor);

  EXPECT_LONGS_EQUAL(ghfg.size(), 1);

  ghfg = HybridGaussianFactorGraph();
  ghfg.push_back(discreteFactor);

  EXPECT_LONGS_EQUAL(ghfg.size(), 1);

  ghfg = HybridGaussianFactorGraph();
  ghfg.push_back(dcFactor);

  HybridGaussianFactorGraph hgfg2;
  hgfg2.push_back(ghfg.begin(), ghfg.end());

  EXPECT_LONGS_EQUAL(ghfg.size(), 1);

  HybridNonlinearFactorGraph hnfg;
  NonlinearFactorGraph factors;
  auto noise = noiseModel::Isotropic::Sigma(3, 1.0);
  factors.emplace_shared<PriorFactor<Pose2>>(0, Pose2(0, 0, 0), noise);
  factors.emplace_shared<PriorFactor<Pose2>>(1, Pose2(1, 0, 0), noise);
  factors.emplace_shared<PriorFactor<Pose2>>(2, Pose2(2, 0, 0), noise);
  // TODO(Varun) This does not currently work. It should work once HybridFactor
  // becomes a base class of NonlinearFactor.
  // hnfg.push_back(factors.begin(), factors.end());

  // EXPECT_LONGS_EQUAL(3, hnfg.size());
}

/****************************************************************************
 * Test construction of switching-like hybrid factor graph.
 */
TEST(HybridFactorGraph, Switching) {
  Switching self(3);

  EXPECT_LONGS_EQUAL(7, self.nonlinearFactorGraph.size());
  EXPECT_LONGS_EQUAL(7, self.linearizedFactorGraph.size());
}

/****************************************************************************
 * Test linearization on a switching-like hybrid factor graph.
 */
TEST(HybridFactorGraph, Linearization) {
  Switching self(3);

  // Linearize here:
  HybridGaussianFactorGraph actualLinearized =
      *self.nonlinearFactorGraph.linearize(self.linearizationPoint);

  EXPECT_LONGS_EQUAL(7, actualLinearized.size());
}

/****************************************************************************
 * Test elimination tree construction
 */
TEST(HybridFactorGraph, EliminationTree) {
  Switching self(3);

  // Create ordering.
  Ordering ordering;
  for (size_t k = 0; k < self.K; k++) ordering += X(k);

  // Create elimination tree.
  HybridEliminationTree etree(self.linearizedFactorGraph, ordering);
  EXPECT_LONGS_EQUAL(1, etree.roots().size())
}

/****************************************************************************
 *Test elimination function by eliminating x0 in *-x0-*-x1 graph.
 */
TEST(GaussianElimination, Eliminate_x0) {
  Switching self(3);

  // Gather factors on x1, has a simple Gaussian and a mixture factor.
  HybridGaussianFactorGraph factors;
  // Add gaussian prior
  factors.push_back(self.linearizedFactorGraph[0]);
  // Add first hybrid factor
  factors.push_back(self.linearizedFactorGraph[1]);

  // Eliminate x0
  Ordering ordering;
  ordering += X(0);

  auto result = EliminateHybrid(factors, ordering);
  CHECK(result.first);
  EXPECT_LONGS_EQUAL(1, result.first->nrFrontals());
  CHECK(result.second);
  // Has two keys, x2 and m1
  EXPECT_LONGS_EQUAL(2, result.second->size());
}

/****************************************************************************
 * Test elimination function by eliminating x1 in x0-*-x1-*-x2 chain.
 *                                                m0/      \m1
 */
TEST(HybridsGaussianElimination, Eliminate_x1) {
  Switching self(3);

  // Gather factors on x1, will be two mixture factors (with x0 and x2, resp.).
  HybridGaussianFactorGraph factors;
  factors.push_back(self.linearizedFactorGraph[1]);  // involves m0
  factors.push_back(self.linearizedFactorGraph[2]);  // involves m1

  // Eliminate x1
  Ordering ordering;
  ordering += X(1);

  std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr> result =
      EliminateHybrid(factors, ordering);
  CHECK(result.first);
  EXPECT_LONGS_EQUAL(1, result.first->nrFrontals());
  CHECK(result.second);
  // Note: separator keys should include m1, m2.
  EXPECT_LONGS_EQUAL(4, result.second->size());
}

/****************************************************************************
 * Helper method to generate gaussian factor graphs with a specific mode.
 */
GaussianFactorGraph::shared_ptr batchGFG(double between,
                                         Values linearizationPoint) {
  NonlinearFactorGraph graph;
  graph.addPrior<double>(X(0), 0, Isotropic::Sigma(1, 0.1));

  auto between_x0_x1 = boost::make_shared<MotionModel>(
      X(0), X(1), between, Isotropic::Sigma(1, 1.0));

  graph.push_back(between_x0_x1);

  return graph.linearize(linearizationPoint);
}

/****************************************************************************
 * Test elimination function by eliminating x0 and x1 in graph.
 */
TEST(HybridGaussianElimination, EliminateHybrid_2_Variable) {
  Switching self(2, 1.0, 0.1);

  auto factors = self.linearizedFactorGraph;

  // Eliminate x0
  Ordering ordering;
  ordering += X(0);
  ordering += X(1);

  HybridConditional::shared_ptr hybridConditionalMixture;
  HybridFactor::shared_ptr factorOnModes;

  std::tie(hybridConditionalMixture, factorOnModes) =
      EliminateHybrid(factors, ordering);

  auto gaussianConditionalMixture =
      dynamic_pointer_cast<GaussianMixture>(hybridConditionalMixture->inner());

  CHECK(gaussianConditionalMixture);
  // Frontals = [x0, x1]
  EXPECT_LONGS_EQUAL(2, gaussianConditionalMixture->nrFrontals());
  // 1 parent, which is the mode
  EXPECT_LONGS_EQUAL(1, gaussianConditionalMixture->nrParents());

  // This is now a HybridDiscreteFactor
  auto hybridDiscreteFactor =
      dynamic_pointer_cast<HybridDiscreteFactor>(factorOnModes);
  // Access the type-erased inner object and convert to DecisionTreeFactor
  auto discreteFactor =
      dynamic_pointer_cast<DecisionTreeFactor>(hybridDiscreteFactor->inner());
  CHECK(discreteFactor);
  EXPECT_LONGS_EQUAL(1, discreteFactor->discreteKeys().size());
  EXPECT(discreteFactor->root_->isLeaf() == false);

  // TODO(Varun) Test emplace_discrete
}

/****************************************************************************
 * Test partial elimination
 */
TEST(HybridFactorGraph, Partial_Elimination) {
  Switching self(3);

  auto linearizedFactorGraph = self.linearizedFactorGraph;

  // Create ordering of only continuous variables.
  Ordering ordering;
  for (size_t k = 0; k < self.K; k++) ordering += X(k);

  // Eliminate partially i.e. only continuous part.
  HybridBayesNet::shared_ptr hybridBayesNet;
  HybridGaussianFactorGraph::shared_ptr remainingFactorGraph;
  std::tie(hybridBayesNet, remainingFactorGraph) =
      linearizedFactorGraph.eliminatePartialSequential(ordering);

  CHECK(hybridBayesNet);
  EXPECT_LONGS_EQUAL(3, hybridBayesNet->size());
  EXPECT(hybridBayesNet->at(0)->frontals() == KeyVector{X(0)});
  EXPECT(hybridBayesNet->at(0)->parents() == KeyVector({X(1), M(0)}));
  EXPECT(hybridBayesNet->at(1)->frontals() == KeyVector{X(1)});
  EXPECT(hybridBayesNet->at(1)->parents() == KeyVector({X(2), M(0), M(1)}));
  EXPECT(hybridBayesNet->at(2)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet->at(2)->parents() == KeyVector({M(0), M(1)}));

  CHECK(remainingFactorGraph);
  EXPECT_LONGS_EQUAL(3, remainingFactorGraph->size());
  EXPECT(remainingFactorGraph->at(0)->keys() == KeyVector({M(0)}));
  EXPECT(remainingFactorGraph->at(1)->keys() == KeyVector({M(1), M(0)}));
  EXPECT(remainingFactorGraph->at(2)->keys() == KeyVector({M(0), M(1)}));
}

/****************************************************************************
 * Test full elimination
 */
TEST(HybridFactorGraph, Full_Elimination) {
  Switching self(3);

  auto linearizedFactorGraph = self.linearizedFactorGraph;

  // We first do a partial elimination
  HybridBayesNet::shared_ptr hybridBayesNet_partial;
  HybridGaussianFactorGraph::shared_ptr remainingFactorGraph_partial;
  DiscreteBayesNet discreteBayesNet;

  {
    // Create ordering.
    Ordering ordering;
    for (size_t k = 0; k < self.K; k++) ordering += X(k);

    // Eliminate partially.
    std::tie(hybridBayesNet_partial, remainingFactorGraph_partial) =
        linearizedFactorGraph.eliminatePartialSequential(ordering);

    DiscreteFactorGraph discrete_fg;
    // TODO(Varun) Make this a function of HybridGaussianFactorGraph?
    for (HybridFactor::shared_ptr& factor : (*remainingFactorGraph_partial)) {
      auto df = dynamic_pointer_cast<HybridDiscreteFactor>(factor);
      discrete_fg.push_back(df->inner());
    }

    ordering.clear();
    for (size_t k = 0; k < self.K - 1; k++) ordering += M(k);
    discreteBayesNet =
        *discrete_fg.eliminateSequential(ordering, EliminateForMPE);
  }

  // Create ordering.
  Ordering ordering;
  for (size_t k = 0; k < self.K; k++) ordering += X(k);
  for (size_t k = 0; k < self.K - 1; k++) ordering += M(k);

  // Eliminate partially.
  HybridBayesNet::shared_ptr hybridBayesNet =
      linearizedFactorGraph.eliminateSequential(ordering);

  CHECK(hybridBayesNet);
  EXPECT_LONGS_EQUAL(5, hybridBayesNet->size());
  // p(x0 | x1, m0)
  EXPECT(hybridBayesNet->at(0)->frontals() == KeyVector{X(0)});
  EXPECT(hybridBayesNet->at(0)->parents() == KeyVector({X(1), M(0)}));
  // p(x1 | x2, m0, m1)
  EXPECT(hybridBayesNet->at(1)->frontals() == KeyVector{X(1)});
  EXPECT(hybridBayesNet->at(1)->parents() == KeyVector({X(2), M(0), M(1)}));
  // p(x2 | m0, m1)
  EXPECT(hybridBayesNet->at(2)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet->at(2)->parents() == KeyVector({M(0), M(1)}));
  // P(m0 | m1)
  EXPECT(hybridBayesNet->at(3)->frontals() == KeyVector{M(0)});
  EXPECT(hybridBayesNet->at(3)->parents() == KeyVector({M(1)}));
  EXPECT(
      dynamic_pointer_cast<DiscreteConditional>(hybridBayesNet->at(3)->inner())
          ->equals(*discreteBayesNet.at(0)));
  // P(m1)
  EXPECT(hybridBayesNet->at(4)->frontals() == KeyVector{M(1)});
  EXPECT_LONGS_EQUAL(0, hybridBayesNet->at(4)->nrParents());
  EXPECT(
      dynamic_pointer_cast<DiscreteConditional>(hybridBayesNet->at(4)->inner())
          ->equals(*discreteBayesNet.at(1)));
}

/****************************************************************************
 * Test printing
 */
TEST(HybridFactorGraph, Printing) {
  Switching self(3);

  auto linearizedFactorGraph = self.linearizedFactorGraph;

  // Create ordering.
  Ordering ordering;
  for (size_t k = 0; k < self.K; k++) ordering += X(k);

  // Eliminate partially.
  HybridBayesNet::shared_ptr hybridBayesNet;
  HybridGaussianFactorGraph::shared_ptr remainingFactorGraph;
  std::tie(hybridBayesNet, remainingFactorGraph) =
      linearizedFactorGraph.eliminatePartialSequential(ordering);

  string expected_hybridFactorGraph = R"(
size: 7
factor 0: Continuous [x0]

  A[x0] = [
	10
]
  b = [ -10 ]
  No noise model
factor 1: Hybrid [x0 x1; m0]{
 Choice(m0) 
 0 Leaf :
  A[x0] = [
	-1
]
  A[x1] = [
	1
]
  b = [ -1 ]
  No noise model

 1 Leaf :
  A[x0] = [
	-1
]
  A[x1] = [
	1
]
  b = [ -0 ]
  No noise model

}
factor 2: Hybrid [x1 x2; m1]{
 Choice(m1) 
 0 Leaf :
  A[x1] = [
	-1
]
  A[x2] = [
	1
]
  b = [ -1 ]
  No noise model

 1 Leaf :
  A[x1] = [
	-1
]
  A[x2] = [
	1
]
  b = [ -0 ]
  No noise model

}
factor 3: Continuous [x1]

  A[x1] = [
	10
]
  b = [ -10 ]
  No noise model
factor 4: Continuous [x2]

  A[x2] = [
	10
]
  b = [ -10 ]
  No noise model
factor 5: Discrete [m0]
 P( m0 ):
 Leaf  0.5

factor 6: Discrete [m1 m0]
 P( m1 | m0 ):
 Choice(m1) 
 0 Choice(m0) 
 0 0 Leaf 0.33333333
 0 1 Leaf  0.6
 1 Choice(m0) 
 1 0 Leaf 0.66666667
 1 1 Leaf  0.4

)";
  EXPECT(assert_print_equal(expected_hybridFactorGraph, linearizedFactorGraph));

  // Expected output for hybridBayesNet.
  string expected_hybridBayesNet = R"(
size: 3
factor 0: Hybrid  P( x0 | x1 m0)
 Discrete Keys = (m0, 2), 
 Choice(m0) 
 0 Leaf  p(x0 | x1)
  R = [ 10.0499 ]
  S[x1] = [ -0.0995037 ]
  d = [ -9.85087 ]
  No noise model

 1 Leaf  p(x0 | x1)
  R = [ 10.0499 ]
  S[x1] = [ -0.0995037 ]
  d = [ -9.95037 ]
  No noise model

factor 1: Hybrid  P( x1 | x2 m0 m1)
 Discrete Keys = (m0, 2), (m1, 2), 
 Choice(m1) 
 0 Choice(m0) 
 0 0 Leaf  p(x1 | x2)
  R = [ 10.2048 ]
  S[x2] = [ -0.0979927 ]
  d = [ -10.3164 ]
  No noise model

 0 1 Leaf  p(x1 | x2)
  R = [ 10.2048 ]
  S[x2] = [ -0.0979927 ]
  d = [ -10.0089 ]
  No noise model

 1 Choice(m0) 
 1 0 Leaf  p(x1 | x2)
  R = [ 10.2048 ]
  S[x2] = [ -0.0979927 ]
  d = [ -10.4144 ]
  No noise model

 1 1 Leaf  p(x1 | x2)
  R = [ 10.2048 ]
  S[x2] = [ -0.0979927 ]
  d = [ -10.1068 ]
  No noise model

factor 2: Hybrid  P( x2 | m0 m1)
 Discrete Keys = (m0, 2), (m1, 2), 
 Choice(m1) 
 0 Choice(m0) 
 0 0 Leaf  p(x2)
  R = [ 10.157 ]
  d = [ -10.4779 ]
  No noise model

 0 1 Leaf  p(x2)
  R = [ 10.157 ]
  d = [ -10.4685 ]
  No noise model

 1 Choice(m0) 
 1 0 Leaf  p(x2)
  R = [ 10.157 ]
  d = [ -10.1664 ]
  No noise model

 1 1 Leaf  p(x2)
  R = [ 10.157 ]
  d = [ -10.157 ]
  No noise model

)";
  EXPECT(assert_print_equal(expected_hybridBayesNet, *hybridBayesNet));
}

/****************************************************************************
 * Simple PlanarSLAM example test with 2 poses and 2 landmarks (each pose
 * connects to 1 landmark) to expose issue with default decision tree creation
 * in hybrid elimination. The hybrid factor is between the poses X0 and X1.
 * The issue arises if we eliminate a landmark variable first since it is not
 * connected to a HybridFactor.
 */
TEST(HybridFactorGraph, DefaultDecisionTree) {
  HybridNonlinearFactorGraph fg;

  // Add a prior on pose x0 at the origin.
  // A prior factor consists of a mean and a noise model (covariance matrix)
  Pose2 prior(0.0, 0.0, 0.0);  // prior mean is at origin
  auto priorNoise = noiseModel::Diagonal::Sigmas(
      Vector3(0.3, 0.3, 0.1));  // 30cm std on x,y, 0.1 rad on theta
  fg.emplace_nonlinear<PriorFactor<Pose2>>(X(0), prior, priorNoise);

  using PlanarMotionModel = BetweenFactor<Pose2>;

  // Add odometry factor
  Pose2 odometry(2.0, 0.0, 0.0);
  KeyVector contKeys = {X(0), X(1)};
  auto noise_model = noiseModel::Isotropic::Sigma(3, 1.0);
  auto still = boost::make_shared<PlanarMotionModel>(X(0), X(1), Pose2(0, 0, 0),
                                                     noise_model),
       moving = boost::make_shared<PlanarMotionModel>(X(0), X(1), odometry,
                                                      noise_model);
  std::vector<PlanarMotionModel::shared_ptr> motion_models = {still, moving};
  fg.emplace_hybrid<MixtureFactor>(
      contKeys, DiscreteKeys{gtsam::DiscreteKey(M(1), 2)}, motion_models);

  // Add Range-Bearing measurements to from X0 to L0 and X1 to L1.
  // create a noise model for the landmark measurements
  auto measurementNoise = noiseModel::Diagonal::Sigmas(
      Vector2(0.1, 0.2));  // 0.1 rad std on bearing, 20cm on range
  // create the measurement values - indices are (pose id, landmark id)
  Rot2 bearing11 = Rot2::fromDegrees(45), bearing22 = Rot2::fromDegrees(90);
  double range11 = std::sqrt(4.0 + 4.0), range22 = 2.0;

  // Add Bearing-Range factors
  fg.emplace_nonlinear<BearingRangeFactor<Pose2, Point2>>(
      X(0), L(0), bearing11, range11, measurementNoise);
  fg.emplace_nonlinear<BearingRangeFactor<Pose2, Point2>>(
      X(1), L(1), bearing22, range22, measurementNoise);

  // Create (deliberately inaccurate) initial estimate
  Values initialEstimate;
  initialEstimate.insert(X(0), Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(X(1), Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(L(0), Point2(1.8, 2.1));
  initialEstimate.insert(L(1), Point2(4.1, 1.8));

  // We want to eliminate variables not connected to DCFactors first.
  Ordering ordering;
  ordering += L(0);
  ordering += L(1);
  ordering += X(0);
  ordering += X(1);

  HybridGaussianFactorGraph linearized = *fg.linearize(initialEstimate);
  gtsam::HybridBayesNet::shared_ptr hybridBayesNet;
  gtsam::HybridGaussianFactorGraph::shared_ptr remainingFactorGraph;

  // This should NOT fail
  std::tie(hybridBayesNet, remainingFactorGraph) =
      linearized.eliminatePartialSequential(ordering);
  EXPECT_LONGS_EQUAL(4, hybridBayesNet->size());
  EXPECT_LONGS_EQUAL(1, remainingFactorGraph->size());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
