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

// pull in the planar SLAM domain with all typedefs and helper functions defined
#include <gtsam/slam/planarSLAM.h>

// we will use Symbol keys
#include <gtsam/nonlinear/Symbol.h>

using namespace std;
using namespace gtsam;

/**
 * Example of a simple 2D planar slam example with landmarls
 *  - The robot and landmarks are on a 2 meter grid
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 *  - We have bearing and range information for measurements
 *  - Landmarks are 2 meters away from the robot trajectory
 */
int main(int argc, char** argv) {

  // create the graph (defined in planarSlam.h, derived from NonlinearFactorGraph)
	planarSLAM::Graph graph;

	// Create some keys
	static Symbol i1('x',1), i2('x',2), i3('x',3);
	static Symbol j1('l',1), j2('l',2);

	// add a Gaussian prior on pose x_1
	Pose2 priorMean(0.0, 0.0, 0.0); // prior mean is at origin
	SharedDiagonal priorNoise(Vector_(3, 0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
	graph.addPrior(i1, priorMean, priorNoise); // add directly to graph

	// add two odometry factors
	Pose2 odometry(2.0, 0.0, 0.0); // create a measurement for both factors (the same in this case)
	SharedDiagonal odometryNoise(Vector_(3, 0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta
	graph.addOdometry(i1, i2, odometry, odometryNoise);
	graph.addOdometry(i2, i3, odometry, odometryNoise);

	// create a noise model for the landmark measurements
	SharedDiagonal measurementNoise(Vector_(2, 0.1, 0.2)); // 0.1 rad std on bearing, 20cm on range

	// create the measurement values - indices are (pose id, landmark id)
	Rot2 bearing11 = Rot2::fromDegrees(45),
		   bearing21 = Rot2::fromDegrees(90),
		   bearing32 = Rot2::fromDegrees(90);
	double range11 = sqrt(4+4),
		     range21 = 2.0,
		     range32 = 2.0;

	// add bearing/range factors (created by "addBearingRange")
	graph.addBearingRange(i1, j1, bearing11, range11, measurementNoise);
	graph.addBearingRange(i2, j1, bearing21, range21, measurementNoise);
	graph.addBearingRange(i3, j2, bearing32, range32, measurementNoise);

	// print
	graph.print("Factor graph");

	// create (deliberatly inaccurate) initial estimate
	planarSLAM::Values initialEstimate;
	initialEstimate.insertPose(i1, Pose2(0.5, 0.0, 0.2));
	initialEstimate.insertPose(i2, Pose2(2.3, 0.1,-0.2));
	initialEstimate.insertPose(i3, Pose2(4.1, 0.1, 0.1));
	initialEstimate.insertPoint(j1, Point2(1.8, 2.1));
	initialEstimate.insertPoint(j2, Point2(4.1, 1.8));

	initialEstimate.print("Initial estimate:\n  ");

	// optimize using Levenberg-Marquardt optimization with an ordering from colamd
	planarSLAM::Values result = graph.optimize(initialEstimate);
	result.print("Final result:\n  ");

	return 0;
}

