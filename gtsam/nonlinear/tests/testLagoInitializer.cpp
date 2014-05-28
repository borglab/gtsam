/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testPlanarSLAMExample_lago.cpp
 *  @brief Unit tests for planar SLAM example using the initialization technique
 *  LAGO (Linear Approximation for Graph Optimization)
 *
 *  @author Luca Carlone
 *  @author Frank Dellaert
 *  @date   May 14, 2014
 */

#include <gtsam/geometry/Pose2.h>

#include <gtsam/inference/Symbol.h>

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LagoInitializer.h>

#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/math/constants/constants.hpp>
#include <cmath>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

Symbol x0('x', 0), x1('x', 1), x2('x', 2), x3('x', 3);
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(3, 0.1));

namespace simple {
// We consider a small graph:
//                            symbolic FG
//               x2               0  1
//             / | \              1  2
//            /  |  \             2  3
//          x3   |   x1           2  0
//           \   |   /            0  3
//            \  |  /
//               x0
//

Pose2 pose0 = Pose2(0.000000, 0.000000, 0.000000);
Pose2 pose1 = Pose2(1.000000, 1.000000, 1.570796);
Pose2 pose2 = Pose2(0.000000, 2.000000, 3.141593);
Pose2 pose3 = Pose2(-1.000000, 1.000000, 4.712389);

NonlinearFactorGraph graph() {
  NonlinearFactorGraph g;
  g.add(BetweenFactor<Pose2>(x0, x1, pose0.between(pose1), model));
  g.add(BetweenFactor<Pose2>(x1, x2, pose1.between(pose2), model));
  g.add(BetweenFactor<Pose2>(x2, x3, pose2.between(pose3), model));
  g.add(BetweenFactor<Pose2>(x2, x0, pose2.between(pose0), model));
  g.add(BetweenFactor<Pose2>(x0, x3, pose0.between(pose3), model));
  g.add(PriorFactor<Pose2>(x0, pose0, model));
  return g;
}
}

/* *************************************************************************** */
TEST( Lago, checkSTandChords ) {
  NonlinearFactorGraph g = simple::graph();
  PredecessorMap<Key> tree = findMinimumSpanningTree<NonlinearFactorGraph, Key,
      BetweenFactor<Pose2> >(g);

  key2doubleMap deltaThetaMap;
  vector<size_t> spanningTreeIds; // ids of between factors forming the spanning tree T
  vector<size_t> chordsIds; // ids of between factors corresponding to chordsIds wrt T
  getSymbolicGraph(spanningTreeIds, chordsIds, deltaThetaMap, tree, g);

  DOUBLES_EQUAL(spanningTreeIds[0], 0, 1e-6); // factor 0 is the first in the ST (0->1)
  DOUBLES_EQUAL(spanningTreeIds[1], 3, 1e-6); // factor 3 is the second in the ST(2->0)
  DOUBLES_EQUAL(spanningTreeIds[2], 4, 1e-6); // factor 4 is the third in the  ST(0->3)

}

/* *************************************************************************** */
TEST( Lago, orientationsOverSpanningTree ) {
  NonlinearFactorGraph g = simple::graph();
  PredecessorMap<Key> tree = findMinimumSpanningTree<NonlinearFactorGraph, Key,
      BetweenFactor<Pose2> >(g);

  // check the tree structure
  EXPECT_LONGS_EQUAL(tree[x0], x0);
  EXPECT_LONGS_EQUAL(tree[x1], x0);
  EXPECT_LONGS_EQUAL(tree[x2], x0);
  EXPECT_LONGS_EQUAL(tree[x3], x0);

  key2doubleMap expected;
  expected[x0]=  0;
  expected[x1]=  M_PI/2; // edge x0->x1 (consistent with edge (x0,x1))
  expected[x2]= -M_PI; // edge x0->x2 (traversed backwards wrt edge (x2,x0))
  expected[x3]= -M_PI/2;  // edge x0->x3 (consistent with edge (x0,x3))

  key2doubleMap deltaThetaMap;
  vector<size_t> spanningTreeIds; // ids of between factors forming the spanning tree T
  vector<size_t> chordsIds; // ids of between factors corresponding to chordsIds wrt T
  getSymbolicGraph(spanningTreeIds, chordsIds, deltaThetaMap, tree, g);

  key2doubleMap actual;
  actual = computeThetasToRoot(deltaThetaMap, tree);
  DOUBLES_EQUAL(expected[x0], actual[x0], 1e-6);
  DOUBLES_EQUAL(expected[x1], actual[x1], 1e-6);
  DOUBLES_EQUAL(expected[x2], actual[x2], 1e-6);
  DOUBLES_EQUAL(expected[x3], actual[x3], 1e-6);
}

/* *************************************************************************** */
TEST( Lago, regularizedMeasurements ) {
  NonlinearFactorGraph g = simple::graph();
  PredecessorMap<Key> tree = findMinimumSpanningTree<NonlinearFactorGraph, Key,
      BetweenFactor<Pose2> >(g);

  key2doubleMap deltaThetaMap;
  vector<size_t> spanningTreeIds; // ids of between factors forming the spanning tree T
  vector<size_t> chordsIds; // ids of between factors corresponding to chordsIds wrt T
  getSymbolicGraph(spanningTreeIds, chordsIds, deltaThetaMap, tree, g);

  key2doubleMap orientationsToRoot = computeThetasToRoot(deltaThetaMap, tree);

  GaussianFactorGraph lagoGraph = buildLinearOrientationGraph(spanningTreeIds, chordsIds, g, orientationsToRoot, tree);
  std::pair<Matrix,Vector> actualAb = lagoGraph.jacobian();
  // jacobian corresponding to the orientation measurements (last entry is the prior on the anchor and is disregarded)
  Vector actual = (Vector(5) <<  actualAb.second(0),actualAb.second(1),actualAb.second(2),actualAb.second(3),actualAb.second(4));
  // this is the whitened error, so we multiply by the std to unwhiten
  actual = 0.1 * actual;
  // Expected regularized measurements (same for the spanning tree, corrected for the chordsIds)
  Vector expected = (Vector(5) << M_PI/2, M_PI, -M_PI/2, M_PI/2 - 2*M_PI , M_PI/2);

  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* *************************************************************************** */
TEST( Lago, smallGraphVectorValues ) {

  VectorValues initialGuessLago = initializeOrientationsLago(simple::graph());

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal((Vector(1) << 0.0), initialGuessLago.at(x0), 1e-6));
  EXPECT(assert_equal((Vector(1) << 0.5 * M_PI), initialGuessLago.at(x1), 1e-6));
  EXPECT(assert_equal((Vector(1) << M_PI - 2*M_PI), initialGuessLago.at(x2), 1e-6));
  EXPECT(assert_equal((Vector(1) << 1.5 * M_PI - 2*M_PI), initialGuessLago.at(x3), 1e-6));
}

/* *************************************************************************** */
TEST( Lago, multiplePosePriors ) {
  NonlinearFactorGraph g = simple::graph();
  g.add(PriorFactor<Pose2>(x1, simple::pose1, model));
  VectorValues initialGuessLago = initializeOrientationsLago(g);

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal((Vector(1) << 0.0), initialGuessLago.at(x0), 1e-6));
  EXPECT(assert_equal((Vector(1) << 0.5 * M_PI), initialGuessLago.at(x1), 1e-6));
  EXPECT(assert_equal((Vector(1) << M_PI - 2*M_PI), initialGuessLago.at(x2), 1e-6));
  EXPECT(assert_equal((Vector(1) << 1.5 * M_PI - 2*M_PI), initialGuessLago.at(x3), 1e-6));
}

/* *************************************************************************** */
TEST( Lago, multiplePoseAndRotPriors ) {
  NonlinearFactorGraph g = simple::graph();
  g.add(PriorFactor<Rot2>(x1, simple::pose1.theta(), model));
  VectorValues initialGuessLago = initializeOrientationsLago(g);

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal((Vector(1) << 0.0), initialGuessLago.at(x0), 1e-6));
  EXPECT(assert_equal((Vector(1) << 0.5 * M_PI), initialGuessLago.at(x1), 1e-6));
  EXPECT(assert_equal((Vector(1) << M_PI - 2*M_PI), initialGuessLago.at(x2), 1e-6));
  EXPECT(assert_equal((Vector(1) << 1.5 * M_PI - 2*M_PI), initialGuessLago.at(x3), 1e-6));
}

/* *************************************************************************** */
TEST( Lago, smallGraphValues ) {

  // we set the orientations in the initial guess to zero
  Values initialGuess;
  initialGuess.insert(x0,Pose2(simple::pose0.x(),simple::pose0.y(),0.0));
  initialGuess.insert(x1,Pose2(simple::pose1.x(),simple::pose1.y(),0.0));
  initialGuess.insert(x2,Pose2(simple::pose2.x(),simple::pose2.y(),0.0));
  initialGuess.insert(x3,Pose2(simple::pose3.x(),simple::pose3.y(),0.0));

  // lago does not touch the Cartesian part and only fixed the orientations
  Values actual = initializeLago(simple::graph(), initialGuess);

  // we are in a noiseless case
  Values expected;
  expected.insert(x0,simple::pose0);
  expected.insert(x1,simple::pose1);
  expected.insert(x2,simple::pose2);
  expected.insert(x3,simple::pose3);

  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* *************************************************************************** */
TEST( Lago, smallGraph2 ) {

  // lago does not touch the Cartesian part and only fixed the orientations
  Values actual = initializeLago(simple::graph());

  // we are in a noiseless case
  Values expected;
  expected.insert(x0,simple::pose0);
  expected.insert(x1,simple::pose1);
  expected.insert(x2,simple::pose2);
  expected.insert(x3,simple::pose3);

  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* *************************************************************************** */
TEST( Lago, smallGraphNoisy_orientations ) {

  NonlinearFactorGraph g;
  Values initial;
  readG2o("/home/aspn/Desktop/noisyToyGraph.txt", g, initial);

  // Add prior on the pose having index (key) = 0
  NonlinearFactorGraph graphWithPrior = g;
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances((Vector(3) << 1e-2, 1e-2, 1e-4));
  graphWithPrior.add(PriorFactor<Pose2>(0, Pose2(), priorModel));

  VectorValues initialGuessLago = initializeOrientationsLago(graphWithPrior);

  // Results from Matlab
  //  VERTEX_SE2 0 0.000000 0.000000 0.000000
  //  VERTEX_SE2 1 0.000000 0.000000 1.568774
  //  VERTEX_SE2 2 0.000000 0.000000 3.142238
  //  VERTEX_SE2 3 0.000000 0.000000 4.702950

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal((Vector(1) << 0.0), initialGuessLago.at(0), 1e-5));
  EXPECT(assert_equal((Vector(1) << 1.568774), initialGuessLago.at(1), 1e-5));
  EXPECT(assert_equal((Vector(1) << 3.14223 - 2*M_PI), initialGuessLago.at(2), 1e-5));
  EXPECT(assert_equal((Vector(1) << 4.702950 - 2*M_PI), initialGuessLago.at(3), 1e-5));
}

/* *************************************************************************** */
TEST( Lago, smallGraphNoisy ) {

  NonlinearFactorGraph g;
  Values initial;
  readG2o("/home/aspn/Desktop/noisyToyGraph.txt", g, initial);

  // Add prior on the pose having index (key) = 0
  NonlinearFactorGraph graphWithPrior = g;
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances((Vector(3) << 1e-2, 1e-2, 1e-4));
  graphWithPrior.add(PriorFactor<Pose2>(0, Pose2(), priorModel));

  // lago does not touch the Cartesian part and only fixed the orientations
  Values actual = initializeLago(graphWithPrior);

  // Optimized results from matlab
  //  VERTEX_SE2 0 0.000000 0.000000 0.000000
  //  VERTEX_SE2 1 1.141931 0.980395 1.569023
  //  VERTEX_SE2 2 0.124207 2.140972 -3.140451
  //  VERTEX_SE2 3 -0.958080 1.070072 -1.577699

  Values expected;
  expected.insert(x0,Pose2(0.000000, 0.000000, 0.000000));
  expected.insert(x1,Pose2(1.141931, 0.980395, 1.569023));
  expected.insert(x2,Pose2(0.124207, 2.140972, -3.140451));
  expected.insert(x3,Pose2(-0.958080, 1.070072, -1.577699));

  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

