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

// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta) to represent
// the robot positions and Point2 variables (x, y) to represent the landmark coordinates.
#include <gtsam/geometry/Pose2.h>

#include <gtsam/linear/GaussianFactorGraph.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use a RangeBearing factor for the range-bearing measurements to identified
// landmarks, and Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/math/constants/constants.hpp>
#include <cmath>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

Symbol x0('x', 0), x1('x', 1), x2('x', 2), x3('x', 3);
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(3, 0.1));
static const double PI = boost::math::constants::pi<double>();

/**
 *  @brief Initialization technique for planar pose SLAM using
 *  LAGO (Linear Approximation for Graph Optimization). see papers:
 *
 *  L. Carlone, R. Aragues, J. Castellanos, and B. Bona, A fast and accurate
 *  approximation for planar pose graph optimization, IJRR, 2014.
 *
 *  L. Carlone, R. Aragues, J.A. Castellanos, and B. Bona, A linear approximation
 *  for graph-based simultaneous localization and mapping, RSS, 2011.
 *
 *  @param graph: nonlinear factor graph including between (Pose2) measurements
 *  @return Values: initial guess including orientation estimate from LAGO
 */

/* ************************************************************************* */
//
#include <gtsam/inference/graph.h>
Values initializeLago(const NonlinearFactorGraph& graph) {
  // Find a minimum spanning tree
  PredecessorMap<Key> tree = findMinimumSpanningTree<NonlinearFactorGraph, Key,
      BetweenFactor<Pose2> >(graph);

  // Order measurements: ordered spanning path first, loop closure later

  // Extract angles in so2 from relative rotations in SO2

  // Correct orientations along loops

  // Create a linear factor graph (LFG) of scalars

  // Solve the LFG

  // Store solution of the LFG in values
  Values estimateLago;
  return estimateLago;
}

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
  return g;
}
}

/*
 *  This function computes the cumulative orientation (without wrapping)
 *  from each node to the root (root has zero orientation)
 */
double computeThetaToRoot(const Key nodeKey, PredecessorMap<Key>& tree,
    map<Key, double>& deltaThetaMap, map<Key, double>& thetaFromRootMap) {

    double nodeTheta = 0;
    Key key_child = nodeKey; // the node
    Key key_parent = 0; // the initialization does not matter
    while(1){
      // We check if we reached the root
      if(tree[key_child]==key_child) // if we reached the root
        break;

      // we sum the delta theta corresponding to the edge parent->child
      nodeTheta += deltaThetaMap[key_child];

      // we get the parent
      key_parent = tree[key_child]; // the parent

      // we check if we connected to some part of the tree we know
      if(thetaFromRootMap.find(key_parent)!=thetaFromRootMap.end()){
        nodeTheta += thetaFromRootMap[key_parent];
        break;
      }
      key_child = key_parent; // we move upwards in the tree
    }
    return nodeTheta;
}

void getSymbolicSubgraph(vector<Key>& keysInBinary, vector<size_t>& spanningTree,
    vector<size_t>& chords, map<Key, double>& deltaThetaMap, PredecessorMap<Key>& tree, const NonlinearFactorGraph& g){

  // Get keys for which you want the orientation
  size_t id=0;

    // Loop over the factors
    BOOST_FOREACH(const boost::shared_ptr<NonlinearFactor>& factor, g){
      if (factor->keys().size() == 2){
        Key key1 = factor->keys()[0];
        Key key2 = factor->keys()[1];
        if(std::find(keysInBinary.begin(), keysInBinary.end(), key1)==keysInBinary.end()) // did not find key1, we add it
          keysInBinary.push_back(key1);
        if(std::find(keysInBinary.begin(), keysInBinary.end(), key2)==keysInBinary.end()) // did not find key2, we add it
          keysInBinary.push_back(key2);

        // recast to a between
        boost::shared_ptr< BetweenFactor<Pose2> > pose2Between = boost::dynamic_pointer_cast< BetweenFactor<Pose2> >(factor);
        if (!pose2Between) continue;

        // get the orientation - measured().theta();
        double deltaTheta = pose2Between->measured().theta();

        bool inTree=false;
        if(tree[key1]==key2){
          deltaThetaMap.insert(std::pair<Key, double>(key1, -deltaTheta));
          inTree = true;
        }
        if(tree[key2]==key1){
          deltaThetaMap.insert(std::pair<Key, double>(key2,  deltaTheta));
          inTree = true;
        }
        if(inTree == true)
          spanningTree.push_back(id);
        else // it's a chord!
          chords.push_back(id);
      }
      id++;
    }
}

/*
 *  This function computes the cumulative orientation (without wrapping)
 *  from each node to the root (root has zero orientation)
 */
map<Key, double> computeThetasToRoot(vector<Key>& keysInBinary, map<Key, double>& deltaThetaMap, PredecessorMap<Key>& tree){

  map<Key, double> thetaToRootMap;
  BOOST_FOREACH(const Key& nodeKey, keysInBinary){
    double nodeTheta = computeThetaToRoot(nodeKey, tree, deltaThetaMap, thetaToRootMap);
    thetaToRootMap.insert(std::pair<Key, double>(nodeKey, nodeTheta));
  }
  return thetaToRootMap;
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

  map<Key, double> expected;
  expected[x0]=  0;
  expected[x1]=  PI/2; // edge x0->x1 (consistent with edge (x0,x1))
  expected[x2]= -PI; // edge x0->x2 (traversed backwards wrt edge (x2,x0))
  expected[x3]= -PI/2;  // edge x0->x3 (consistent with edge (x0,x3))

  vector<Key> keysInBinary;
  map<Key, double> deltaThetaMap;
  vector<size_t> spanningTree; // ids of between factors forming the spanning tree T
  vector<size_t> chords; // ids of between factors corresponding to chords wrt T
  getSymbolicSubgraph(keysInBinary, spanningTree, chords, deltaThetaMap, tree, g);

  map<Key, double> actual;
  actual = computeThetasToRoot(keysInBinary, deltaThetaMap, tree);
  DOUBLES_EQUAL(expected[x0], actual[x0], 1e-6);
  DOUBLES_EQUAL(expected[x1], actual[x1], 1e-6);
  DOUBLES_EQUAL(expected[x2], actual[x2], 1e-6);
  DOUBLES_EQUAL(expected[x3], actual[x3], 1e-6);
}

/*
 *  Linear factor graph with regularized orientation measurements
 */
GaussianFactorGraph buildOrientationGraph(const vector<size_t>& spanningTree, const vector<size_t>& chords,
    const NonlinearFactorGraph& g, map<Key, double> orientationsToRoot){
  GaussianFactorGraph lagoGraph;

  BOOST_FOREACH(const size_t& factorId, spanningTree){ // put original measurements in the spanning tree
    Key key1 = g[factorId]->keys()[0];
    Key key2 = g[factorId]->keys()[1];
    boost::shared_ptr< BetweenFactor<Pose2> > pose2Between = boost::dynamic_pointer_cast< BetweenFactor<Pose2> >(g[factorId]);
    if (!pose2Between) throw std::invalid_argument("buildOrientationGraph: invalid between factor!");;
    double deltaTheta = pose2Between->measured().theta();
    //SharedNoiseModel model = g[factorId]->get_noiseModel()
    //lagoGraph.add(JacobianFactor(key1, -1, key2, 1, deltaTheta, model));
  }

  return lagoGraph;
}

/* *************************************************************************** */
TEST( Lago, regularizedMeasurements ) {
  NonlinearFactorGraph g = simple::graph();
  PredecessorMap<Key> tree = findMinimumSpanningTree<NonlinearFactorGraph, Key,
        BetweenFactor<Pose2> >(g);

  vector<Key> keysInBinary;
  map<Key, double> deltaThetaMap;
  vector<size_t> spanningTree; // ids of between factors forming the spanning tree T
  vector<size_t> chords; // ids of between factors corresponding to chords wrt T
  getSymbolicSubgraph(keysInBinary, spanningTree, chords, deltaThetaMap, tree, g);

  map<Key, double> orientationsToRoot = computeThetasToRoot(keysInBinary, deltaThetaMap, tree);

  GaussianFactorGraph lagoGraph = buildOrientationGraph(spanningTree, chords, g, orientationsToRoot);
  std::pair<Matrix,Vector> actualAb = lagoGraph.jacobian();
  Vector actual = actualAb.second;

  // This respects the order of the factors in the original graph
  Vector expected = (Vector(5) <<  1.570796326794897, 1.570796326794897, 1.570796326794897, -3.141592653589793, 4.712388980384690 );
  // This arranges the spanning tree first and chords later
  Vector orderedExpected = (Vector(5) <<  expected[spanningTree[0]], expected[spanningTree[1]], expected[spanningTree[2]],
      expected[chords[0]], expected[chords[1]] );

  EXPECT(assert_equal(orderedExpected, actual, 1e-6));
}

/* *************************************************************************** */
//TEST( Lago, smallGraph_GTmeasurements ) {
//
//  Values initialGuessLago = initializeLago(simple::graph());
//
//  DOUBLES_EQUAL(0.0, (initialGuessLago.at<Pose2>(x0)).theta(), 1e-6);
//  DOUBLES_EQUAL(0.5 * PI, (initialGuessLago.at<Pose2>(x1)).theta(), 1e-6);
//  DOUBLES_EQUAL(PI, (initialGuessLago.at<Pose2>(x2)).theta(), 1e-6);
//  DOUBLES_EQUAL(1.5 * PI, (initialGuessLago.at<Pose2>(x3)).theta(), 1e-6);
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

