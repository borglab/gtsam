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

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridISAM.h>
#include <gtsam/hybrid/MixtureFactor.h>
#include <gtsam/hybrid/NonlinearHybridFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
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
  NonlinearHybridFactorGraph fg;

  // Add a simple prior factor to the nonlinear factor graph
  fg.emplace_nonlinear<PriorFactor<double>>(X(0), 0, Isotropic::Sigma(1, 0.1));

  // Linearization point
  Values linearizationPoint;
  linearizationPoint.insert<double>(X(0), 0);

  GaussianHybridFactorGraph ghfg = fg.linearize(linearizationPoint);

  // Add a factor to the GaussianFactorGraph
  ghfg.add(JacobianFactor(X(0), I_1x1, Vector1(5)));

  EXPECT_LONGS_EQUAL(2, ghfg.size());
}

/* **************************************************************************
 */
/// Test that the resize method works correctly for a
/// NonlinearHybridFactorGraph.
TEST(NonlinearHybridFactorGraph, Resize) {
  NonlinearHybridFactorGraph fg;
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

/* **************************************************************************
 */
/// Test that the resize method works correctly for a
/// GaussianHybridFactorGraph.
TEST(GaussianHybridFactorGraph, Resize) {
  NonlinearHybridFactorGraph nhfg;
  auto nonlinearFactor = boost::make_shared<BetweenFactor<double>>(
      X(0), X(1), 0.0, Isotropic::Sigma(1, 0.1));
  nhfg.push_back(nonlinearFactor);
  auto discreteFactor = boost::make_shared<DecisionTreeFactor>();
  nhfg.push_back(discreteFactor);

  KeyVector contKeys = {X(0), X(1)};
  auto noise_model = noiseModel::Isotropic::Sigma(1, 1.0);
  auto still = boost::make_shared<MotionModel>(X(0), X(1), 0.0, noise_model),
       moving = boost::make_shared<MotionModel>(X(0), X(1), 1.0, noise_model);

  // TODO(Varun) This is declared as NonlinearFactor instead of MotionModel, aka
  // not clear!!
  std::vector<NonlinearFactor::shared_ptr> components = {still, moving};
  auto dcFactor = boost::make_shared<MixtureFactor>(
      contKeys, DiscreteKeys{gtsam::DiscreteKey(M(1), 2)}, components);
  nhfg.push_back(dcFactor);

  Values linearizationPoint;
  linearizationPoint.insert<double>(X(0), 0);
  linearizationPoint.insert<double>(X(1), 1);

  // Generate `GaussianHybridFactorGraph` by linearizing
  GaussianHybridFactorGraph gfg = nhfg.linearize(linearizationPoint);

  EXPECT_LONGS_EQUAL(gfg.size(), 3);

  gfg.resize(0);
  EXPECT_LONGS_EQUAL(gfg.size(), 0);
}

/*
****************************************************************************
* Test push_back on HFG makes the correct distinction.
*/
TEST(HybridFactorGraph, PushBack) {
  NonlinearHybridFactorGraph fg;

  auto nonlinearFactor = boost::make_shared<BetweenFactor<double>>();
  fg.push_back(nonlinearFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 1);

  fg = NonlinearHybridFactorGraph();

  auto discreteFactor = boost::make_shared<DecisionTreeFactor>();
  fg.push_back(discreteFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 1);

  fg = NonlinearHybridFactorGraph();

  auto dcFactor = boost::make_shared<MixtureFactor>();
  fg.push_back(dcFactor);

  EXPECT_LONGS_EQUAL(fg.size(), 1);

  // Now do the same with GaussianHybridFactorGraph
  GaussianHybridFactorGraph ghfg;

  auto gaussianFactor = boost::make_shared<JacobianFactor>();
  ghfg.push_back(gaussianFactor);

  EXPECT_LONGS_EQUAL(ghfg.size(), 1);

  ghfg = GaussianHybridFactorGraph();
  ghfg.push_back(discreteFactor);

  EXPECT_LONGS_EQUAL(ghfg.size(), 1);

  ghfg = GaussianHybridFactorGraph();
  ghfg.push_back(dcFactor);

  EXPECT_LONGS_EQUAL(ghfg.size(), 1);
}

// /*
// ****************************************************************************/
// // Test construction of switching-like hybrid factor graph.
// TEST(HybridFactorGraph, Switching) {
//   Switching self(3);

//   EXPECT_LONGS_EQUAL(8, self.nonlinearFactorGraph.size());
//   EXPECT_LONGS_EQUAL(4, self.nonlinearFactorGraph.nonlinearGraph().size());
//   EXPECT_LONGS_EQUAL(2, self.nonlinearFactorGraph.discreteGraph().size());
//   EXPECT_LONGS_EQUAL(2, self.nonlinearFactorGraph.dcGraph().size());

//   EXPECT_LONGS_EQUAL(8, self.linearizedFactorGraph.size());
//   EXPECT_LONGS_EQUAL(2, self.linearizedFactorGraph.discreteGraph().size());
//   EXPECT_LONGS_EQUAL(2, self.linearizedFactorGraph.dcGraph().size());
//   EXPECT_LONGS_EQUAL(4, self.linearizedFactorGraph.gaussianGraph().size());
// }

// /*
// ****************************************************************************/
// // Test linearization on a switching-like hybrid factor graph.
// TEST(HybridFactorGraph, Linearization) {
//   Switching self(3);

//   // Linearize here:
//   GaussianHybridFactorGraph actualLinearized =
//       self.nonlinearFactorGraph.linearize(self.linearizationPoint);

//   EXPECT_LONGS_EQUAL(8, actualLinearized.size());
//   EXPECT_LONGS_EQUAL(2, actualLinearized.discreteGraph().size());
//   EXPECT_LONGS_EQUAL(2, actualLinearized.dcGraph().size());
//   EXPECT_LONGS_EQUAL(4, actualLinearized.gaussianGraph().size());
// }

// /*
// ****************************************************************************/
// // Test elimination tree construction
// TEST(HybridFactorGraph, EliminationTree) {
//   Switching self(3);

//   // Create ordering.
//   Ordering ordering;
//   for (size_t k = 1; k <= self.K; k++) ordering += X(k);

//   // Create elimination tree.
//   HybridEliminationTree etree(self.linearizedFactorGraph, ordering);
//   EXPECT_LONGS_EQUAL(1, etree.roots().size())
// }

// /*
// ****************************************************************************/
// // Test elimination function by eliminating x1 in *-x1-*-x2 graph.
// TEST(DCGaussianElimination, Eliminate_x1) {
//   Switching self(3);

//   // Gather factors on x1, has a simple Gaussian and a mixture factor.
//   GaussianHybridFactorGraph factors;
//   factors.push_gaussian(self.linearizedFactorGraph.gaussianGraph()[0]);
//   factors.push_dc(self.linearizedFactorGraph.dcGraph()[0]);

//   // Check that sum works:
//   auto sum = factors.sum();
//   Assignment<Key> mode;
//   mode[M(1)] = 1;
//   auto actual = sum(mode);               // Selects one of 2 modes.
//   EXPECT_LONGS_EQUAL(2, actual.size());  // Prior and motion model.

//   // Eliminate x1
//   Ordering ordering;
//   ordering += X(1);

//   auto result = EliminateHybrid(factors, ordering);
//   CHECK(result.first);
//   EXPECT_LONGS_EQUAL(1, result.first->nrFrontals());
//   CHECK(result.second);
//   // Has two keys, x2 and m1
//   EXPECT_LONGS_EQUAL(2, result.second->size());
// }

// /*
// ****************************************************************************/
// // Test elimination function by eliminating x2 in x1-*-x2-*-x3 chain.
// //                                                m1/      \m2
// TEST(DCGaussianElimination, Eliminate_x2) {
//   Switching self(3);

//   // Gather factors on x2, will be two mixture factors (with x1 and x3,
//   resp.). GaussianHybridFactorGraph factors;
//   factors.push_dc(self.linearizedFactorGraph.dcGraph()[0]);  // involves m1
//   factors.push_dc(self.linearizedFactorGraph.dcGraph()[1]);  // involves m2

//   // Check that sum works:
//   auto sum = factors.sum();
//   Assignment<Key> mode;
//   mode[M(1)] = 0;
//   mode[M(2)] = 1;
//   auto actual = sum(mode);               // Selects one of 4 mode
//   combinations. EXPECT_LONGS_EQUAL(2, actual.size());  // 2 motion models.

//   // Eliminate x2
//   Ordering ordering;
//   ordering += X(2);

//   std::pair<AbstractConditional::shared_ptr, boost::shared_ptr<Factor>>
//   result =
//       EliminateHybrid(factors, ordering);
//   CHECK(result.first);
//   EXPECT_LONGS_EQUAL(1, result.first->nrFrontals());
//   CHECK(result.second);
//   // Note: separator keys should include m1, m2.
//   EXPECT_LONGS_EQUAL(4, result.second->size());
// }

// /*
// ****************************************************************************/
// // Helper method to generate gaussian factor graphs with a specific mode.
// GaussianFactorGraph::shared_ptr batchGFG(double between,
//                                          Values linearizationPoint) {
//   NonlinearFactorGraph graph;
//   graph.addPrior<double>(X(1), 0, Isotropic::Sigma(1, 0.1));

//   auto between_x1_x2 = boost::make_shared<MotionModel>(
//       X(1), X(2), between, Isotropic::Sigma(1, 1.0));

//   graph.push_back(between_x1_x2);

//   return graph.linearize(linearizationPoint);
// }

// /*
// ****************************************************************************/
// // Test elimination function by eliminating x1 and x2 in graph.
// TEST(DCGaussianElimination, EliminateHybrid_2_Variable) {
//   Switching self(2, 1.0, 0.1);

//   auto factors = self.linearizedFactorGraph;

//   // Check that sum works:
//   auto sum = factors.sum();
//   Assignment<Key> mode;
//   mode[M(1)] = 1;
//   auto actual = sum(mode);  // Selects one of 2 modes.
//   EXPECT_LONGS_EQUAL(4,
//                      actual.size());  // Prior, 1 motion models, 2
//                      measurements.

//   // Eliminate x1
//   Ordering ordering;
//   ordering += X(1);
//   ordering += X(2);

//   AbstractConditional::shared_ptr abstractConditionalMixture;
//   boost::shared_ptr<Factor> factorOnModes;
//   std::tie(abstractConditionalMixture, factorOnModes) =
//       EliminateHybrid(factors, ordering);

//   auto gaussianConditionalMixture =
//       dynamic_pointer_cast<GaussianMixture>(abstractConditionalMixture);

//   CHECK(gaussianConditionalMixture);
//   EXPECT_LONGS_EQUAL(
//       2,
//       gaussianConditionalMixture->nrFrontals());  // Frontals = [x1, x2]
//   EXPECT_LONGS_EQUAL(
//       1,
//       gaussianConditionalMixture->nrParents());  // 1 parent, which is the
//       mode

//   auto discreteFactor =
//   dynamic_pointer_cast<DecisionTreeFactor>(factorOnModes);
//   CHECK(discreteFactor);
//   EXPECT_LONGS_EQUAL(1, discreteFactor->discreteKeys().size());
//   EXPECT(discreteFactor->root_->isLeaf() == false);
// }

// /*
// ****************************************************************************/
// /// Test the toDecisionTreeFactor method
// TEST(HybridFactorGraph, ToDecisionTreeFactor) {
//   size_t K = 3;

//   // Provide tight sigma values so that the errors are visibly different.
//   double between_sigma = 5e-8, prior_sigma = 1e-7;

//   Switching self(K, between_sigma, prior_sigma);

//   // Clear out discrete factors since sum() cannot hanldle those
//   GaussianHybridFactorGraph linearizedFactorGraph(
//       self.linearizedFactorGraph.gaussianGraph(), DiscreteFactorGraph(),
//       self.linearizedFactorGraph.dcGraph());

//   auto decisionTreeFactor = linearizedFactorGraph.toDecisionTreeFactor();

//   auto allAssignments =
//       DiscreteValues::CartesianProduct(linearizedFactorGraph.discreteKeys());

//   // Get the error of the discrete assignment m1=0, m2=1.
//   double actual = (*decisionTreeFactor)(allAssignments[1]);

//   /********************************************/
//   // Create equivalent factor graph for m1=0, m2=1
//   GaussianFactorGraph graph = linearizedFactorGraph.gaussianGraph();

//   for (auto &p : linearizedFactorGraph.dcGraph()) {
//     if (auto mixture =
//             boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(p)) {
//       graph.add((*mixture)(allAssignments[1]));
//     }
//   }

//   VectorValues values = graph.optimize();
//   double expected = graph.probPrime(values);
//   /********************************************/
//   EXPECT_DOUBLES_EQUAL(expected, actual, 1e-12);
//   // REGRESSION:
//   EXPECT_DOUBLES_EQUAL(0.6125, actual, 1e-4);
// }

// /*
// ****************************************************************************/
// // Test partial elimination
// TEST_UNSAFE(HybridFactorGraph, Partial_Elimination) {
//   Switching self(3);

//   auto linearizedFactorGraph = self.linearizedFactorGraph;

//   // Create ordering.
//   Ordering ordering;
//   for (size_t k = 1; k <= self.K; k++) ordering += X(k);

//   // Eliminate partially.
//   HybridBayesNet::shared_ptr hybridBayesNet;
//   GaussianHybridFactorGraph::shared_ptr remainingFactorGraph;
//   std::tie(hybridBayesNet, remainingFactorGraph) =
//       linearizedFactorGraph.eliminatePartialSequential(ordering);

//   CHECK(hybridBayesNet);
//   //  GTSAM_PRINT(*hybridBayesNet);  // HybridBayesNet
//   EXPECT_LONGS_EQUAL(3, hybridBayesNet->size());
//   EXPECT(hybridBayesNet->at(0)->frontals() == KeyVector{X(1)});
//   EXPECT(hybridBayesNet->at(0)->parents() == KeyVector({X(2), M(1)}));
//   EXPECT(hybridBayesNet->at(1)->frontals() == KeyVector{X(2)});
//   EXPECT(hybridBayesNet->at(1)->parents() == KeyVector({X(3), M(2), M(1)}));
//   EXPECT(hybridBayesNet->at(2)->frontals() == KeyVector{X(3)});
//   EXPECT(hybridBayesNet->at(2)->parents() == KeyVector({M(2), M(1)}));

//   CHECK(remainingFactorGraph);
//   //  GTSAM_PRINT(*remainingFactorGraph);  // HybridFactorGraph
//   EXPECT_LONGS_EQUAL(3, remainingFactorGraph->size());
//   EXPECT(remainingFactorGraph->discreteGraph().at(0)->keys() ==
//          KeyVector({M(1)}));
//   EXPECT(remainingFactorGraph->discreteGraph().at(1)->keys() ==
//          KeyVector({M(2), M(1)}));
//   EXPECT(remainingFactorGraph->discreteGraph().at(2)->keys() ==
//          KeyVector({M(2), M(1)}));
// }

// /*
// ****************************************************************************/
// // Test full elimination
// TEST_UNSAFE(HybridFactorGraph, Full_Elimination) {
//   Switching self(3);

//   auto linearizedFactorGraph = self.linearizedFactorGraph;

//   // We first do a partial elimination
//   HybridBayesNet::shared_ptr hybridBayesNet_partial;
//   GaussianHybridFactorGraph::shared_ptr remainingFactorGraph_partial;
//   DiscreteBayesNet discreteBayesNet;

//   {
//     // Create ordering.
//     Ordering ordering;
//     for (size_t k = 1; k <= self.K; k++) ordering += X(k);

//     // Eliminate partially.
//     std::tie(hybridBayesNet_partial, remainingFactorGraph_partial) =
//         linearizedFactorGraph.eliminatePartialSequential(ordering);

//     DiscreteFactorGraph dfg;
//     dfg.push_back(remainingFactorGraph_partial->discreteGraph());
//     ordering.clear();
//     for (size_t k = 1; k < self.K; k++) ordering += M(k);
//     discreteBayesNet = *dfg.eliminateSequential(ordering, EliminateForMPE);
//   }

//   // Create ordering.
//   Ordering ordering;
//   for (size_t k = 1; k <= self.K; k++) ordering += X(k);
//   for (size_t k = 1; k < self.K; k++) ordering += M(k);

//   // Eliminate partially.
//   HybridBayesNet::shared_ptr hybridBayesNet =
//       linearizedFactorGraph.eliminateSequential(ordering);

//   CHECK(hybridBayesNet);
//   EXPECT_LONGS_EQUAL(5, hybridBayesNet->size());
//   // p(x1 | x2, m1)
//   EXPECT(hybridBayesNet->at(0)->frontals() == KeyVector{X(1)});
//   EXPECT(hybridBayesNet->at(0)->parents() == KeyVector({X(2), M(1)}));
//   // p(x2 | x3, m1, m2)
//   EXPECT(hybridBayesNet->at(1)->frontals() == KeyVector{X(2)});
//   EXPECT(hybridBayesNet->at(1)->parents() == KeyVector({X(3), M(2), M(1)}));
//   // p(x3 | m1, m2)
//   EXPECT(hybridBayesNet->at(2)->frontals() == KeyVector{X(3)});
//   EXPECT(hybridBayesNet->at(2)->parents() == KeyVector({M(2), M(1)}));
//   // P(m1 | m2)
//   EXPECT(hybridBayesNet->at(3)->frontals() == KeyVector{M(1)});
//   EXPECT(hybridBayesNet->at(3)->parents() == KeyVector({M(2)}));
//   EXPECT(dynamic_pointer_cast<DiscreteConditional>(hybridBayesNet->at(3))
//              ->equals(*discreteBayesNet.at(0)));
//   // P(m2)
//   EXPECT(hybridBayesNet->at(4)->frontals() == KeyVector{M(2)});
//   EXPECT_LONGS_EQUAL(0, hybridBayesNet->at(4)->nrParents());
//   EXPECT(dynamic_pointer_cast<DiscreteConditional>(hybridBayesNet->at(4))
//              ->equals(*discreteBayesNet.at(1)));
// }

// /*
// ****************************************************************************/
// // Test printing
// TEST(HybridFactorGraph, Printing) {
//   Switching self(3);

//   auto linearizedFactorGraph = self.linearizedFactorGraph;

//   // Create ordering.
//   Ordering ordering;
//   for (size_t k = 1; k <= self.K; k++) ordering += X(k);

//   // Eliminate partially.
//   HybridBayesNet::shared_ptr hybridBayesNet;
//   GaussianHybridFactorGraph::shared_ptr remainingFactorGraph;
//   std::tie(hybridBayesNet, remainingFactorGraph) =
//       linearizedFactorGraph.eliminatePartialSequential(ordering);

//   string expected_hybridFactorGraph = R"(
// size: 8
// DiscreteFactorGraph
// size: 2
// factor 0:  P( m1 ):
//  Leaf  0.5
// factor 1:  P( m2 | m1 ):
//  Choice(m2)
//  0 Choice(m1)
//  0 0 Leaf 0.33333333
//  0 1 Leaf  0.6
//  1 Choice(m1)
//  1 0 Leaf 0.66666667
//  1 1 Leaf  0.4
// DCFactorGraph
// size: 2
// factor 0:  [ x1 x2; m1 ]{
//  Choice(m1)
//  0 Leaf Jacobian factor on 2 keys:
//   A[x1] = [
// 	-1
// ]
//   A[x2] = [
// 	1
// ]
//   b = [ -1 ]
//   No noise model
//  1 Leaf Jacobian factor on 2 keys:
//   A[x1] = [
// 	-1
// ]
//   A[x2] = [
// 	1
// ]
//   b = [ -0 ]
//   No noise model
// }
// factor 1:  [ x2 x3; m2 ]{
//  Choice(m2)
//  0 Leaf Jacobian factor on 2 keys:
//   A[x2] = [
// 	-1
// ]
//   A[x3] = [
// 	1
// ]
//   b = [ -1 ]
//   No noise model
//  1 Leaf Jacobian factor on 2 keys:
//   A[x2] = [
// 	-1
// ]
//   A[x3] = [
// 	1
// ]
//   b = [ -0 ]
//   No noise model
// }
// GaussianGraph
// size: 4
// factor 0:
//   A[x1] = [
// 	10
// ]
//   b = [ -10 ]
//   No noise model
// factor 1:
//   A[x1] = [
// 	10
// ]
//   b = [ -10 ]
//   No noise model
// factor 2:
//   A[x2] = [
// 	10
// ]
//   b = [ -10 ]
//   No noise model
// factor 3:
//   A[x3] = [
// 	10
// ]
//   b = [ -10 ]
//   No noise model
// )";
//   EXPECT(assert_print_equal(expected_hybridFactorGraph,
//   linearizedFactorGraph));

//   // Expected output for hybridBayesNet.
//   string expected_hybridBayesNet = R"(
// size: 3
// factor 0:  GaussianMixture [ x1 | x2 m1 ]{
//  Choice(m1)
//  0 Leaf Jacobian factor on 2 keys:
//  p(x1 | x2)
//   R = [ 14.1774 ]
//   S[x2] = [ -0.0705346 ]
//   d = [ -14.0364 ]
//   No noise model
//  1 Leaf Jacobian factor on 2 keys:
//  p(x1 | x2)
//   R = [ 14.1774 ]
//   S[x2] = [ -0.0705346 ]
//   d = [ -14.1069 ]
//   No noise model
// }
// factor 1:  GaussianMixture [ x2 | x3 m2 m1 ]{
//  Choice(m2)
//  0 Choice(m1)
//  0 0 Leaf Jacobian factor on 2 keys:
//  p(x2 | x3)
//   R = [ 10.0993 ]
//   S[x3] = [ -0.0990172 ]
//   d = [ -9.99975 ]
//   No noise model
//  0 1 Leaf Jacobian factor on 2 keys:
//  p(x2 | x3)
//   R = [ 10.0993 ]
//   S[x3] = [ -0.0990172 ]
//   d = [ -9.90122 ]
//   No noise model
//  1 Choice(m1)
//  1 0 Leaf Jacobian factor on 2 keys:
//  p(x2 | x3)
//   R = [ 10.0993 ]
//   S[x3] = [ -0.0990172 ]
//   d = [ -10.0988 ]
//   No noise model
//  1 1 Leaf Jacobian factor on 2 keys:
//  p(x2 | x3)
//   R = [ 10.0993 ]
//   S[x3] = [ -0.0990172 ]
//   d = [ -10.0002 ]
//   No noise model
// }
// factor 2:  GaussianMixture [ x3 | m2 m1 ]{
//  Choice(m2)
//  0 Choice(m1)
//  0 0 Leaf Jacobian factor on 1 keys:
//  p(x3)
//   R = [ 10.0494 ]
//   d = [ -10.1489 ]
//   No noise model
//  0 1 Leaf Jacobian factor on 1 keys:
//  p(x3)
//   R = [ 10.0494 ]
//   d = [ -10.1479 ]
//   No noise model
//  1 Choice(m1)
//  1 0 Leaf Jacobian factor on 1 keys:
//  p(x3)
//   R = [ 10.0494 ]
//   d = [ -10.0504 ]
//   No noise model
//  1 1 Leaf Jacobian factor on 1 keys:
//  p(x3)
//   R = [ 10.0494 ]
//   d = [ -10.0494 ]
//   No noise model
// }
// )";
//   EXPECT(assert_print_equal(expected_hybridBayesNet, *hybridBayesNet));
// }

// /* *************************************************************************
// */
// // Simple PlanarSLAM example test with 2 poses and 2 landmarks (each pose
// // connects to 1 landmark) to expose issue with default decision tree
// creation
// // in hybrid elimination. The hybrid factor is between the poses X0 and X1.
// The
// // issue arises if we eliminate a landmark variable first since it is not
// // connected to a DCFactor.
// TEST(HybridFactorGraph, DefaultDecisionTree) {
//   NonlinearHybridFactorGraph fg;

//   // Add a prior on pose x1 at the origin. A prior factor consists of a mean
//   and
//   // a noise model (covariance matrix)
//   Pose2 prior(0.0, 0.0, 0.0);  // prior mean is at origin
//   auto priorNoise = noiseModel::Diagonal::Sigmas(
//       Vector3(0.3, 0.3, 0.1));  // 30cm std on x,y, 0.1 rad on theta
//   fg.emplace_nonlinear<PriorFactor<Pose2>>(X(0), prior, priorNoise);

//   using PlanarMotionModel = BetweenFactor<Pose2>;

//   // Add odometry factor
//   Pose2 odometry(2.0, 0.0, 0.0);
//   KeyVector contKeys = {X(0), X(1)};
//   auto noise_model = noiseModel::Isotropic::Sigma(3, 1.0);
//   auto still = boost::make_shared<PlanarMotionModel>(X(0), X(1), Pose2(0, 0,
//   0),
//                                                      noise_model),
//        moving = boost::make_shared<PlanarMotionModel>(X(0), X(1), odometry,
//                                                       noise_model);
//   std::vector<PlanarMotionModel::shared_ptr> components = {still, moving};
//   auto dcFactor = boost::make_shared<DCMixtureFactor<PlanarMotionModel>>(
//       contKeys, DiscreteKeys{gtsam::DiscreteKey(M(1), 2)}, components);
//   fg.push_back(dcFactor);

//   // Add Range-Bearing measurements to from X0 to L0 and X1 to L1.
//   // create a noise model for the landmark measurements
//   auto measurementNoise = noiseModel::Diagonal::Sigmas(
//       Vector2(0.1, 0.2));  // 0.1 rad std on bearing, 20cm on range
//   // create the measurement values - indices are (pose id, landmark id)
//   Rot2 bearing11 = Rot2::fromDegrees(45), bearing22 = Rot2::fromDegrees(90);
//   double range11 = std::sqrt(4.0 + 4.0), range22 = 2.0;

//   // Add Bearing-Range factors
//   fg.emplace_nonlinear<BearingRangeFactor<Pose2, Point2>>(
//       X(0), L(0), bearing11, range11, measurementNoise);
//   fg.emplace_nonlinear<BearingRangeFactor<Pose2, Point2>>(
//       X(1), L(1), bearing22, range22, measurementNoise);

//   // Create (deliberately inaccurate) initial estimate
//   Values initialEstimate;
//   initialEstimate.insert(X(0), Pose2(0.5, 0.0, 0.2));
//   initialEstimate.insert(X(1), Pose2(2.3, 0.1, -0.2));
//   initialEstimate.insert(L(0), Point2(1.8, 2.1));
//   initialEstimate.insert(L(1), Point2(4.1, 1.8));

//   // We want to eliminate variables not connected to DCFactors first.
//   Ordering ordering;
//   ordering += L(0);
//   ordering += L(1);
//   ordering += X(0);
//   ordering += X(1);

//   GaussianHybridFactorGraph linearized = fg.linearize(initialEstimate);
//   gtsam::HybridBayesNet::shared_ptr hybridBayesNet;
//   gtsam::GaussianHybridFactorGraph::shared_ptr remainingFactorGraph;

//   // This should NOT fail
//   std::tie(hybridBayesNet, remainingFactorGraph) =
//       linearized.eliminatePartialSequential(ordering);
//   EXPECT_LONGS_EQUAL(4, hybridBayesNet->size());
//   EXPECT_LONGS_EQUAL(1, remainingFactorGraph->size());
// }

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */