/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample_easy.cpp
 * @brief A 2D Pose SLAM example using the predefined typedefs in gtsam/slam/pose2SLAM.h
 * @date Oct 21, 2010
 * @author Yong Dian Jian
 */

// pull in the Pose2 SLAM domain with all typedefs and helper functions defined
#include <cmath>
#include <gtsam/slam/pose2SLAM.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

	// 1. Create graph container and add factors to it
	pose2SLAM::Graph graph ;

	// 2a. Add Gaussian prior
	Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
	SharedDiagonal priorNoise(Vector_(3, 0.3, 0.3, 0.1));
	graph.addPrior(1, priorMean, priorNoise);

	// 2b. Add odometry factors
	SharedDiagonal odometryNoise(Vector_(3, 0.2, 0.2, 0.1));
	graph.addOdometry(1, 2, Pose2(2.0, 0.0, 0.0   ), odometryNoise);
	graph.addOdometry(2, 3, Pose2(2.0, 0.0, M_PI/2.0), odometryNoise);
	graph.addOdometry(3, 4, Pose2(2.0, 0.0, M_PI/2.0), odometryNoise);
	graph.addOdometry(4, 5, Pose2(2.0, 0.0, M_PI/2.0), odometryNoise);

	// 2c. Add pose constraint
	SharedDiagonal constraintUncertainty(Vector_(3, 0.2, 0.2, 0.1));
	graph.addConstraint(5, 2, Pose2(2.0, 0.0, M_PI/2.0), constraintUncertainty);

	// print
	graph.print("\nFactor graph:\n");

  // 3. Create the data structure to hold the initialEstimate estinmate to the solution
	pose2SLAM::Values initialEstimate;
	Pose2 x1(0.5, 0.0, 0.2   ); initialEstimate.insertPose(1, x1);
	Pose2 x2(2.3, 0.1,-0.2   ); initialEstimate.insertPose(2, x2);
	Pose2 x3(4.1, 0.1, M_PI/2.0); initialEstimate.insertPose(3, x3);
	Pose2 x4(4.0, 2.0, M_PI  ); initialEstimate.insertPose(4, x4);
	Pose2 x5(2.1, 2.1,-M_PI/2.0); initialEstimate.insertPose(5, x5);
	initialEstimate.print("\nInitial estimate:\n  ");

	// 4. Single Step Optimization using Levenberg-Marquardt
	pose2SLAM::Values result = graph.optimize(initialEstimate);
	result.print("\nFinal result:\n  ");

	return 0;
}
