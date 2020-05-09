/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file METISOrdering.cpp
 * @brief Simple robot motion example, with prior and two odometry measurements,
 * using a METIS ordering
 * @author Frank Dellaert
 * @author Andrew Melim
 */

/**
 * Example of a simple 2D localization example optimized using METIS ordering
 * - For more details on the full optimization pipeline, see OdometryExample.cpp
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  NonlinearFactorGraph graph;

  Pose2 priorMean(0.0, 0.0, 0.0);  // prior at origin
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.addPrior(1, priorMean, priorNoise);

  Pose2 odometry(2.0, 0.0, 0.0);
  auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, odometry, odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, odometry, odometryNoise);
  graph.print("\nFactor Graph:\n");  // print

  Values initial;
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));
  initial.print("\nInitial Estimate:\n");  // print

  // optimize using Levenberg-Marquardt optimization
  LevenbergMarquardtParams params;
  // In order to specify the ordering type, we need to se the "orderingType". By
  // default this parameter is set to OrderingType::COLAMD
  params.orderingType = Ordering::METIS;
  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  return 0;
}
