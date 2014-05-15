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

map<Key, double> misteryFunction(const PredecessorMap<Key>& tree, const NonlinearFactorGraph&){

}

/* *************************************************************************** */
TEST( Lago, sumOverLoops ) {
  NonlinearFactorGraph g = simple::graph();
  PredecessorMap<Key> tree = findMinimumSpanningTree<NonlinearFactorGraph, Key,
        BetweenFactor<Pose2> >(g);

  // check the tree structure
  EXPECT_LONGS_EQUAL(tree[x0], x0);
  EXPECT_LONGS_EQUAL(tree[x1], x0);
  EXPECT_LONGS_EQUAL(tree[x2], x0);
  EXPECT_LONGS_EQUAL(tree[x3], x0);

  g.print("");

  map<Key, double> expected;
  expected[x0]=  0;
  expected[x1]=  1.570796; // edge x0->x1 (consistent with edge (x0,x1))
  expected[x2]= -3.141593; // edge x0->x2 (traversed backwards wrt edge (x2,x0))
  expected[x3]=  4.712389; // edge x0->x3 (consistent with edge (x0,x3))

  map<Key, double> actual;
  actual = misteryFunction(tree, g);
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

