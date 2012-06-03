/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMwSPCG.cpp
 * @brief A 2D Pose SLAM example using the SimpleSPCGSolver.
 * @author Yong-Dian Jian
 * @date June 2, 2012
 */

#include <gtsam/linear/IterativeOptimizationParameters.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/pose2SLAM.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(void) {

  // 1. Create graph container and add factors to it
  pose2SLAM::Graph graph ;

  // 2a. Add Gaussian prior
  Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
  SharedDiagonal priorNoise(Vector_(3, 0.3, 0.3, 0.1));
  graph.addPrior(1, priorMean, priorNoise);

  // 2b. Add odometry factors
  SharedDiagonal odometryNoise(Vector_(3, 0.2, 0.2, 0.1));
  graph.addOdometry(1, 2, Pose2(2.0, 0.0, 0.0   ), odometryNoise);
  graph.addOdometry(2, 3, Pose2(2.0, 0.0, M_PI_2), odometryNoise);
  graph.addOdometry(3, 4, Pose2(2.0, 0.0, M_PI_2), odometryNoise);
  graph.addOdometry(4, 5, Pose2(2.0, 0.0, M_PI_2), odometryNoise);

  // 2c. Add pose constraint
  SharedDiagonal constraintUncertainty(Vector_(3, 0.2, 0.2, 0.1));
  graph.addConstraint(5, 2, Pose2(2.0, 0.0, M_PI_2), constraintUncertainty);

  // print
  graph.print("\nFactor graph:\n");

  // 3. Create the data structure to hold the initialEstimate estinmate to the solution
  pose2SLAM::Values initialEstimate;
  Pose2 x1(0.5, 0.0, 0.2   ); initialEstimate.insertPose(1, x1);
  Pose2 x2(2.3, 0.1,-0.2   ); initialEstimate.insertPose(2, x2);
  Pose2 x3(4.1, 0.1, M_PI_2); initialEstimate.insertPose(3, x3);
  Pose2 x4(4.0, 2.0, M_PI  ); initialEstimate.insertPose(4, x4);
  Pose2 x5(2.1, 2.1,-M_PI_2); initialEstimate.insertPose(5, x5);
  initialEstimate.print("\nInitial estimate:\n  ");
  cout << "initial error = " << graph.error(initialEstimate) << endl ;

  // 4. Single Step Optimization using Levenberg-Marquardt
  // Note: Although there are many options in IterativeOptimizationParameters,
  // the SimpleSPCGSolver doesn't actually use all of them at this moment.
  // More detail in the next release.
  LevenbergMarquardtParams param;
  param.linearSolverType = SuccessiveLinearizationParams::CG;
  param.iterativeParams = boost::make_shared<IterativeOptimizationParameters>();

  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, param);
  Values result = optimizer.optimize();
  cout << "final error = " << graph.error(result) << endl;

  return 0 ;
}
