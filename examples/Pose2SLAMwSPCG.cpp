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

#include <gtsam/linear/SimpleSPCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/pose2SLAM.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using namespace std;
using namespace gtsam;
using namespace gtsam::noiseModel;

/* ************************************************************************* */
int main(void) {

  // 1. Create graph container and add factors to it
  pose2SLAM::Graph graph ;

  // 2a. Add Gaussian prior
  Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
  SharedDiagonal priorNoise  = Diagonal::Sigmas(Vector_(3, 0.3, 0.3, 0.1));
  graph.addPosePrior(1, priorMean, priorNoise);

  // 2b. Add odometry factors
  SharedDiagonal odometryNoise = Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
  graph.addRelativePose(1, 2, Pose2(2.0, 0.0, 0.0   ), odometryNoise);
  graph.addRelativePose(2, 3, Pose2(2.0, 0.0, M_PI_2), odometryNoise);
  graph.addRelativePose(3, 4, Pose2(2.0, 0.0, M_PI_2), odometryNoise);
  graph.addRelativePose(4, 5, Pose2(2.0, 0.0, M_PI_2), odometryNoise);

  // 2c. Add pose constraint
  SharedDiagonal constraintUncertainty = Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
  graph.addRelativePose(5, 2, Pose2(2.0, 0.0, M_PI_2), constraintUncertainty);

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
  LevenbergMarquardtParams param;
  param.verbosity = NonlinearOptimizerParams::ERROR;
  param.verbosityLM = LevenbergMarquardtParams::LAMBDA;
  param.linearSolverType = SuccessiveLinearizationParams::CG;

  {
    param.iterativeParams = boost::make_shared<SimpleSPCGSolverParameters>();
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, param);
    Values result = optimizer.optimize();
    result.print("\nFinal result:\n");
    cout << "simple spcg solver final error = " << graph.error(result) << endl;
  }

  {
    param.iterativeParams = boost::make_shared<SubgraphSolverParameters>();
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, param);
    Values result = optimizer.optimize();
    result.print("\nFinal result:\n");
    cout << "subgraph solver final error = " << graph.error(result) << endl;
  }

  {
  	Values result = graph.optimizeSPCG(initialEstimate);
  	result.print("\nFinal result:\n");
  }

  return 0 ;
}
