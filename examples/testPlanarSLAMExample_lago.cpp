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


#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

Symbol x0('x', 0), l1('x', 1), x2('x', 2), x3('x',3);
static SharedNoiseModel noiseModel(noiseModel::Isotropic::Sigma(3, 0.1));

/* ************************************************************************* */
Values initializeLago(const NonlinearFactorGraph& graph) {
  // Order measurements: ordered spanning path first, loop closure later

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

  Pose2 pose0 = Pose2( 0.000000 0.000000 0.000000);
  Pose2 pose1 = Pose2( 1.000000 1.000000 1.570796);
  Pose2 pose2 = Pose2( 0.000000 2.000000 3.141593);
  Pose2 pose3 = Pose2(-1.000000 1.000000 4.712389);

  NonlinearFactorGraph = graph;

  BetweenFactor<Pose2> factor01(x0, x1, pose0.between(pose1), noiseModel);
  graph.add(factor01);

//  BetweenFactor<Pose2> factor12(x1, x2, pose1.between(pose2), noiseModel);
//  graph.add(factor);
//
//  BetweenFactor<Pose2> factor01(x0, x1, pose0.between(pose1), noiseModel);
//  graph.add(factor);
//
//  BetweenFactor<Pose2> factor01(x0, x1, pose0.between(pose1), noiseModel);
//  graph.add(factor);
//
//  BetweenFactor<Pose2> factor01(x0, x1, pose0.between(pose1), noiseModel);
//  graph.add(factor);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

