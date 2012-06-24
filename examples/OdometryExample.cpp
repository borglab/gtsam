/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file OdometryExample.cpp
 * @brief Simple robot motion example, with prior and two odometry measurements
 * @author Frank Dellaert
 */

// pull in the 2D PoseSLAM domain with all typedefs and helper functions defined
#include <gtsam/slam/pose2SLAM.h>

// include this for marginals
#include <gtsam/nonlinear/Marginals.h>

#include <iomanip>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gtsam::noiseModel;

/**
 * Example of a simple 2D localization example
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 */
int main(int argc, char** argv) {

	// create the graph (defined in pose2SLAM.h, derived from NonlinearFactorGraph)
	pose2SLAM::Graph graph;

	// add a Gaussian prior on pose x_1
	Pose2 priorMean(0.0, 0.0, 0.0); // prior mean is at origin
	SharedDiagonal priorNoise = Diagonal::Sigmas(Vector_(3, 0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
	graph.addPosePrior(1, priorMean, priorNoise); // add directly to graph

	// add two odometry factors
	Pose2 odometry(2.0, 0.0, 0.0); // create a measurement for both factors (the same in this case)
	SharedDiagonal odometryNoise = Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta
	graph.addRelativePose(1, 2, odometry, odometryNoise);
	graph.addRelativePose(2, 3, odometry, odometryNoise);

	// print
	graph.print("\nFactor graph:\n");

	// create (deliberatly inaccurate) initial estimate
	pose2SLAM::Values initialEstimate;
	initialEstimate.insertPose(1, Pose2(0.5, 0.0, 0.2));
	initialEstimate.insertPose(2, Pose2(2.3, 0.1, -0.2));
	initialEstimate.insertPose(3, Pose2(4.1, 0.1, 0.1));
	initialEstimate.print("\nInitial estimate:\n  ");

	// optimize using Levenberg-Marquardt optimization with an ordering from colamd
	pose2SLAM::Values result = graph.optimize(initialEstimate);
	result.print("\nFinal result:\n  ");

	// Query the marginals
	cout.precision(2);
	Marginals marginals = graph.marginals(result);
	cout << "\nP1:\n" << marginals.marginalCovariance(1) << endl;
	cout << "\nP2:\n" << marginals.marginalCovariance(2) << endl;
	cout << "\nP3:\n" << marginals.marginalCovariance(3) << endl;

	return 0;
}

