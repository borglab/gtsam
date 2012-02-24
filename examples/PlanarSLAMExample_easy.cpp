/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PlanarSLAMExample.cpp
 * @brief Simple robotics example using the pre-built planar SLAM domain
 * @author Alex Cunningham
 */

#include <cmath>
#include <iostream>

// pull in the planar SLAM domain with all typedefs and helper functions defined
#include <gtsam/slam/planarSLAM.h>
#include <gtsam/nonlinear/NonlinearOptimization-inl.h>

using namespace std;
using namespace gtsam;
using namespace planarSLAM;

/**
 * In this version of the system we make the following assumptions:
 *  - All values are axis aligned
 *  - Robot poses are facing along the X axis (horizontal, to the right in images)
 *  - We have bearing and range information for measurements
 *  - We have full odometry for measurements
 *  - The robot and landmarks are on a grid, moving 2 meters each step
 *  - Landmarks are 2 meters away from the robot trajectory
 */
int main(int argc, char** argv) {

  // create graph container and add factors to it
	Graph graph;

	/* add prior  */
	// gaussian for prior
	SharedDiagonal prior_model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.3, 0.3, 0.1));
	Pose2 prior_measurement(0.0, 0.0, 0.0); // prior at origin
	graph.addPrior(1, prior_measurement, prior_model); // add directly to graph

	/* add odometry */
	// general noisemodel for odometry
	SharedDiagonal odom_model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
	Pose2 odom_measurement(2.0, 0.0, 0.0); // create a measurement for both factors (the same in this case)
	graph.addOdometry(1, 2, odom_measurement, odom_model);
	graph.addOdometry(2, 3, odom_measurement, odom_model);

	/* add measurements */
	// general noisemodel for measurements
	SharedDiagonal meas_model = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.2));

	// create the measurement values - indices are (pose id, landmark id)
	Rot2 bearing11 = Rot2::fromDegrees(45),
		 bearing21 = Rot2::fromDegrees(90),
		 bearing32 = Rot2::fromDegrees(90);
	double range11 = sqrt(4+4),
		   range21 = 2.0,
		   range32 = 2.0;

	// create bearing/range factors and add them
	graph.addBearingRange(1, 1, bearing11, range11, meas_model);
	graph.addBearingRange(2, 1, bearing21, range21, meas_model);
	graph.addBearingRange(3, 2, bearing32, range32, meas_model);

	graph.print("full graph");

	// initialize to noisy points
	planarSLAM::Values initialEstimate;
	initialEstimate.insertPose(1, Pose2(0.5, 0.0, 0.2));
	initialEstimate.insertPose(2, Pose2(2.3, 0.1,-0.2));
	initialEstimate.insertPose(3, Pose2(4.1, 0.1, 0.1));
	initialEstimate.insertPoint(1, Point2(1.8, 2.1));
	initialEstimate.insertPoint(2, Point2(4.1, 1.8));

	initialEstimate.print("initial estimate");

	// optimize using Levenberg-Marquardt optimization with an ordering from colamd
	planarSLAM::Values result = optimize(graph, initialEstimate);
	result.print("final result");

	return 0;
}

