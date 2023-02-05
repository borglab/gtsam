/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testLago.cpp
 *  @brief Unit tests for planar SLAM example using the initialization technique
 *  LAGO (Linear Approximation for Graph Optimization)
 *
 *  @author Luca Carlone
 *  @author Frank Dellaert
 *  @date   May 14, 2014
 */

#include <gtsam/slam/lago.h>
#include <gtsam/slam/InitializePose.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>

#include <cmath>

using namespace std;
using namespace gtsam;

static Symbol x0('x', 0), x1('x', 1), x2('x', 2), x3('x', 3);
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(3, 0.1));

namespace simpleLago {
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

static Pose2 pose0 = Pose2(0.000000, 0.000000, 0.000000);
static Pose2 pose1 = Pose2(1.000000, 1.000000, 1.570796);
static Pose2 pose2 = Pose2(0.000000, 2.000000, 3.141593);
static Pose2 pose3 = Pose2(-1.000000, 1.000000, 4.712389);

NonlinearFactorGraph graph() {
  NonlinearFactorGraph g;
  g.add(BetweenFactor<Pose2>(x0, x1, pose0.between(pose1), model));
  g.add(BetweenFactor<Pose2>(x1, x2, pose1.between(pose2), model));
  g.add(BetweenFactor<Pose2>(x2, x3, pose2.between(pose3), model));
  g.add(BetweenFactor<Pose2>(x2, x0, pose2.between(pose0), model));
  g.add(BetweenFactor<Pose2>(x0, x3, pose0.between(pose3), model));
  g.addPrior(x0, pose0, model);
  return g;
}
}

/*******************************************************************************/
TEST(Lago, findMinimumSpanningTree) {
  NonlinearFactorGraph g = simpleLago::graph();
  auto gPlus = initialize::buildPoseGraph<Pose2>(g);
  lago::PredecessorMap tree = lago::findMinimumSpanningTree(gPlus);

  // We should recover the following spanning tree:
  //
  //              x2
  //             /  \               
  //            /    \              
  //          x3     x1
  //                 /
  //                /
  //              x0
  //               |
  //               a
  using initialize::kAnchorKey;
  EXPECT_LONGS_EQUAL(kAnchorKey, tree[kAnchorKey]);
  EXPECT_LONGS_EQUAL(kAnchorKey, tree[x0]);
  EXPECT_LONGS_EQUAL(x0, tree[x1]);
  EXPECT_LONGS_EQUAL(x1, tree[x2]);
  EXPECT_LONGS_EQUAL(x2, tree[x3]);
}

/* *************************************************************************** */
TEST( Lago, checkSTandChords ) {
  NonlinearFactorGraph g = simpleLago::graph();
  auto gPlus = initialize::buildPoseGraph<Pose2>(g);
  lago::PredecessorMap tree = lago::findMinimumSpanningTree(gPlus);

  lago::key2doubleMap deltaThetaMap;
  vector<size_t> spanningTreeIds; // ids of between factors forming the spanning tree T
  vector<size_t> chordsIds; // ids of between factors corresponding to chordsIds wrt T
  lago::getSymbolicGraph(spanningTreeIds, chordsIds, deltaThetaMap, tree, g);

  EXPECT_LONGS_EQUAL(0, spanningTreeIds[0]); // factor 0 is the first in the ST(0->1)
  EXPECT_LONGS_EQUAL(1, spanningTreeIds[1]); // factor 1 is the second in the ST(1->2)
  EXPECT_LONGS_EQUAL(2, spanningTreeIds[2]); // factor 2 is the third in the  ST(2->3)

}

/* *************************************************************************** */
TEST(Lago, orientationsOverSpanningTree) {
  NonlinearFactorGraph g = simpleLago::graph();
  auto gPlus = initialize::buildPoseGraph<Pose2>(g);
  lago::PredecessorMap tree = lago::findMinimumSpanningTree(gPlus);

  // check the tree structure
  using initialize::kAnchorKey;

  EXPECT_LONGS_EQUAL(kAnchorKey, tree[x0]);
  EXPECT_LONGS_EQUAL(x0, tree[x1]);
  EXPECT_LONGS_EQUAL(x1, tree[x2]);
  EXPECT_LONGS_EQUAL(x2, tree[x3]);

  lago::key2doubleMap expected;
  expected[x0] = 0;
  expected[x1] = M_PI / 2;      // edges traversed: x0->x1
  expected[x2] = M_PI;          // edges traversed: x0->x1->x2
  expected[x3] = 3 * M_PI / 2;  // edges traversed: x0->x1->x2->x3

  lago::key2doubleMap deltaThetaMap;
  vector<size_t>
      spanningTreeIds;  // ids of between factors forming the spanning tree T
  vector<size_t>
      chordsIds;  // ids of between factors corresponding to chordsIds wrt T
  lago::getSymbolicGraph(spanningTreeIds, chordsIds, deltaThetaMap, tree,
                         gPlus);

  lago::key2doubleMap actual;
  actual = lago::computeThetasToRoot(deltaThetaMap, tree);

  DOUBLES_EQUAL(expected[x0], actual[x0], 1e-6);
  DOUBLES_EQUAL(expected[x1], actual[x1], 1e-6);
  DOUBLES_EQUAL(expected[x2], actual[x2], 1e-6);
  DOUBLES_EQUAL(expected[x3], actual[x3], 1e-6);
}

/* *************************************************************************** */
TEST( Lago, regularizedMeasurements ) {
  NonlinearFactorGraph g = simpleLago::graph();
  auto gPlus = initialize::buildPoseGraph<Pose2>(g);
  lago::PredecessorMap tree = lago::findMinimumSpanningTree(gPlus);

  lago::key2doubleMap deltaThetaMap;
  vector<size_t> spanningTreeIds; // ids of between factors forming the spanning tree T
  vector<size_t> chordsIds; // ids of between factors corresponding to chordsIds wrt T
  lago::getSymbolicGraph(spanningTreeIds, chordsIds, deltaThetaMap, tree, gPlus);

  lago::key2doubleMap orientationsToRoot = lago::computeThetasToRoot(deltaThetaMap, tree);

  GaussianFactorGraph lagoGraph = lago::buildLinearOrientationGraph(spanningTreeIds, chordsIds, gPlus, orientationsToRoot, tree);
  std::pair<Matrix,Vector> actualAb = lagoGraph.jacobian();
  // jacobian corresponding to the orientation measurements (last entry is the prior on the anchor and is disregarded)
  Vector actual = (Vector(5) <<  actualAb.second(0),actualAb.second(1),actualAb.second(2),actualAb.second(3),actualAb.second(4)).finished();
  // this is the whitened error, so we multiply by the std to unwhiten
  actual = 0.1 * actual;
  // Expected regularized measurements (same for the spanning tree, corrected for the chordsIds)
  Vector expected = (Vector(5) << M_PI/2, M_PI/2, M_PI/2, 0 , -M_PI).finished();

  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* *************************************************************************** */
TEST( Lago, smallGraphVectorValues ) {
  bool useOdometricPath = false;
  VectorValues initial = lago::initializeOrientations(simpleLago::graph(), useOdometricPath);

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal((Vector(1) << 0.0).finished(), initial.at(x0), 1e-6));
  EXPECT(assert_equal((Vector(1) << 0.5 * M_PI).finished(), initial.at(x1), 1e-6));
  EXPECT(assert_equal((Vector(1) << M_PI).finished(), initial.at(x2), 1e-6));
  EXPECT(assert_equal((Vector(1) << 1.5 * M_PI).finished(), initial.at(x3), 1e-6));
}

/* *************************************************************************** */
TEST( Lago, smallGraphVectorValuesSP ) {

  VectorValues initial = lago::initializeOrientations(simpleLago::graph());

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal((Vector(1) << 0.0).finished(), initial.at(x0), 1e-6));
  EXPECT(assert_equal((Vector(1) << 0.5 * M_PI).finished(), initial.at(x1), 1e-6));
  EXPECT(assert_equal((Vector(1) << M_PI ).finished(), initial.at(x2), 1e-6));
  EXPECT(assert_equal((Vector(1) << 1.5 * M_PI ).finished(), initial.at(x3), 1e-6));
}

/* *************************************************************************** */
TEST( Lago, multiplePosePriors ) {
  bool useOdometricPath = false;
  NonlinearFactorGraph g = simpleLago::graph();
  g.addPrior(x1, simpleLago::pose1, model);
  VectorValues initial = lago::initializeOrientations(g, useOdometricPath);

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal((Vector(1) << 0.0).finished(), initial.at(x0), 1e-6));
  EXPECT(assert_equal((Vector(1) << 0.5 * M_PI).finished(), initial.at(x1), 1e-6));
  EXPECT(assert_equal((Vector(1) << M_PI).finished(), initial.at(x2), 1e-6));
  EXPECT(assert_equal((Vector(1) << 1.5 * M_PI).finished(), initial.at(x3), 1e-6));
}

/* *************************************************************************** */
TEST( Lago, multiplePosePriorsSP ) {
  NonlinearFactorGraph g = simpleLago::graph();
  g.addPrior(x1, simpleLago::pose1, model);
  VectorValues initial = lago::initializeOrientations(g);

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal((Vector(1) << 0.0).finished(), initial.at(x0), 1e-6));
  EXPECT(assert_equal((Vector(1) << 0.5 * M_PI).finished(), initial.at(x1), 1e-6));
  EXPECT(assert_equal((Vector(1) << M_PI ).finished(), initial.at(x2), 1e-6));
  EXPECT(assert_equal((Vector(1) << 1.5 * M_PI ).finished(), initial.at(x3), 1e-6));
}

/* *************************************************************************** */
TEST( Lago, multiplePoseAndRotPriors ) {
  bool useOdometricPath = false;
  NonlinearFactorGraph g = simpleLago::graph();
  g.addPrior(x1, simpleLago::pose1.theta(), model);
  VectorValues initial = lago::initializeOrientations(g, useOdometricPath);
  
  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal((Vector(1) << 0.0).finished(), initial.at(x0), 1e-6));
  EXPECT(assert_equal((Vector(1) << 0.5 * M_PI).finished(), initial.at(x1), 1e-6));
  EXPECT(assert_equal((Vector(1) << M_PI).finished(), initial.at(x2), 1e-6));
  EXPECT(assert_equal((Vector(1) << 1.5 * M_PI).finished(), initial.at(x3), 1e-6));
}

/* *************************************************************************** */
TEST( Lago, multiplePoseAndRotPriorsSP ) {
  NonlinearFactorGraph g = simpleLago::graph();
  g.addPrior(x1, simpleLago::pose1.theta(), model);
  VectorValues initial = lago::initializeOrientations(g);

  // comparison is up to M_PI, that's why we add some multiples of 2*M_PI
  EXPECT(assert_equal((Vector(1) << 0.0).finished(), initial.at(x0), 1e-6));
  EXPECT(assert_equal((Vector(1) << 0.5 * M_PI).finished(), initial.at(x1), 1e-6));
  EXPECT(assert_equal((Vector(1) << M_PI ).finished(), initial.at(x2), 1e-6));
  EXPECT(assert_equal((Vector(1) << 1.5 * M_PI ).finished(), initial.at(x3), 1e-6));
}

/* *************************************************************************** */
TEST( Lago, smallGraphValues ) {

  // we set the orientations in the initial guess to zero
  Values initialGuess;
  initialGuess.insert(x0,Pose2(simpleLago::pose0.x(),simpleLago::pose0.y(),0.0));
  initialGuess.insert(x1,Pose2(simpleLago::pose1.x(),simpleLago::pose1.y(),0.0));
  initialGuess.insert(x2,Pose2(simpleLago::pose2.x(),simpleLago::pose2.y(),0.0));
  initialGuess.insert(x3,Pose2(simpleLago::pose3.x(),simpleLago::pose3.y(),0.0));

  // lago does not touch the Cartesian part and only fixed the orientations
  Values actual = lago::initialize(simpleLago::graph(), initialGuess);

  // we are in a noiseless case
  Values expected;
  expected.insert(x0,simpleLago::pose0);
  expected.insert(x1,simpleLago::pose1);
  expected.insert(x2,simpleLago::pose2);
  expected.insert(x3,simpleLago::pose3);

  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* *************************************************************************** */
TEST( Lago, smallGraph2 ) {

  // lago does not touch the Cartesian part and only fixed the orientations
  Values actual = lago::initialize(simpleLago::graph());

  // we are in a noiseless case
  Values expected;
  expected.insert(x0,simpleLago::pose0);
  expected.insert(x1,simpleLago::pose1);
  expected.insert(x2,simpleLago::pose2);
  expected.insert(x3,simpleLago::pose3);

  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* *************************************************************************** */
TEST( Lago, largeGraphNoisy_orientations ) {

  string inputFile = findExampleDataFile("noisyToyGraph");
  NonlinearFactorGraph::shared_ptr g;
  Values::shared_ptr initial;
  std::tie(g, initial) = readG2o(inputFile);

  // Add prior on the pose having index (key) = 0
  NonlinearFactorGraph graphWithPrior = *g;
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances(Vector3(1e-2, 1e-2, 1e-4));
  graphWithPrior.addPrior(0, Pose2(), priorModel);

  VectorValues actualVV = lago::initializeOrientations(graphWithPrior);
  Values actual;
  Key keyAnc = symbol('Z',9999999);
  for(VectorValues::const_iterator it = actualVV.begin(); it != actualVV.end(); ++it ){
    Key key = it->first;
    if (key != keyAnc){
      Vector orientation = actualVV.at(key);
      Pose2 poseLago = Pose2(0.0,0.0,orientation(0));
      actual.insert(key, poseLago);
    }
  }
  string matlabFile = findExampleDataFile("orientationsNoisyToyGraph");
  NonlinearFactorGraph::shared_ptr gmatlab;
  Values::shared_ptr expected;
  std::tie(gmatlab, expected) = readG2o(matlabFile);

  for(const auto& key_pose: expected->extract<Pose2>()){
    const Key& k = key_pose.first;
    const Pose2& pose = key_pose.second;
    EXPECT(assert_equal(pose, actual.at<Pose2>(k), 1e-5));
  }
}

/* *************************************************************************** */
TEST( Lago, largeGraphNoisy ) {

  string inputFile = findExampleDataFile("noisyToyGraph");
  NonlinearFactorGraph::shared_ptr g;
  Values::shared_ptr initial;
  std::tie(g, initial) = readG2o(inputFile);

  // Add prior on the pose having index (key) = 0
  NonlinearFactorGraph graphWithPrior = *g;
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances(Vector3(1e-2, 1e-2, 1e-4));
  graphWithPrior.addPrior(0, Pose2(), priorModel);

  Values actual = lago::initialize(graphWithPrior);

  string matlabFile = findExampleDataFile("optimizedNoisyToyGraph");
  NonlinearFactorGraph::shared_ptr gmatlab;
  Values::shared_ptr expected;
  std::tie(gmatlab, expected) = readG2o(matlabFile);

  for(const auto& key_pose: expected->extract<Pose2>()){
    const Key& k = key_pose.first;
    const Pose2& pose = key_pose.second;
    EXPECT(assert_equal(pose, actual.at<Pose2>(k), 1e-2));
  }
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

