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
 *  LAGO (Linear Approximation for Graph Optimization) proposed in:
 *
 *  L. Carlone, R. Aragues, J. Castellanos, and B. Bona, A fast and accurate
 *  approximation for planar pose graph optimization, IJRR, 2014.
 *
 *  L. Carlone, R. Aragues, J.A. Castellanos, and B. Bona, A linear approximation
 *  for graph-based simultaneous localization and mapping, RSS, 2011.
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

Symbol x0('x', 0), x1('x', 1), x2('x', 2), x3('x',3);
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(3, 0.1));
static const double PI = boost::math::constants::pi<double>();

/* ************************************************************************* */
Values initializeLago(const NonlinearFactorGraph& graph) {
  // Order measurements: ordered spanning path first, loop closure later

  // Extract angles in so2 from relative rotations in SO2

  // Correct orientations along loops

  // Create a linear factor graph (LFG) of scalars

  // Solve the LFG

  // Store solution of the LFG in values
  Values estimateLago;
  return estimateLago;
}


/* *************************************************************************** */
TEST( Lago, smallGraph_GTmeasurements ) {
// We consider a small graph:
//                            symbolic FG
//               x2               0  1
//             / | \              1  2
//            /  |  \             2  3
//          x3   |   x4           2  1
//           \   |   /            1  3
//            \  |  /
//               x0

  Pose2 pose0 = Pose2( 0.000000, 0.000000, 0.000000);
  Pose2 pose1 = Pose2( 1.000000, 1.000000, 1.570796);
  Pose2 pose2 = Pose2( 0.000000, 2.000000, 3.141593);
  Pose2 pose3 = Pose2(-1.000000, 1.000000, 4.712389);

  NonlinearFactorGraph graph;

  BetweenFactor<Pose2> factor01(x0, x1, pose0.between(pose1), model);
  graph.add(factor01);

  BetweenFactor<Pose2> factor12(x1, x2, pose1.between(pose2), model);
  graph.add(factor12);

  BetweenFactor<Pose2> factor23(x2, x3, pose2.between(pose3), model);
  graph.add(factor23);

  BetweenFactor<Pose2> factor20(x2, x0, pose2.between(pose0), model);
  graph.add(factor20);

  BetweenFactor<Pose2> factor03(x0, x3, pose0.between(pose3), model);
  graph.add(factor03);

  // graph.print("graph");

  Values initialGuessLago = initializeLago(graph);

  Vector expectedOrientations = (Vector(4) << 0.0, 0.5*PI, PI, 1.5*PI);
  Vector actualOrientations(4);
  actualOrientations(0) = (initialGuessLago.at<Pose2>(x0)).theta();
  actualOrientations(1) = (initialGuessLago.at<Pose2>(x1)).theta();
  actualOrientations(2) = (initialGuessLago.at<Pose2>(x2)).theta();
  actualOrientations(3) = (initialGuessLago.at<Pose2>(x3)).theta();

  EXPECT(assert_equal(expectedOrientations, actualOrientations, 1e-6));
  //DOUBLES_EQUAL(expected, actual, 1e-6);
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

