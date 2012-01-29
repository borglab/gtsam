/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample_easy.cpp
 *
 * A 2D Pose SLAM example using the predefined typedefs in gtsam/slam/pose2SLAM.h
 *
 * @date Oct 21, 2010
 * @author ydjian
 */

#include <cmath>
#include <iostream>
#include <boost/shared_ptr.hpp>

// pull in the Pose2 SLAM domain with all typedefs and helper functions defined
#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/nonlinear/NonlinearOptimization.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::pose2SLAM;

int main(int argc, char** argv) {
	// create keys for robot positions
	Key x1(1), x2(2), x3(3);

	/* 1. create graph container and add factors to it */
	Graph graph ;

	/* 2.a add prior  */
	// gaussian for prior
	SharedDiagonal prior_model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.3, 0.3, 0.1));
	Pose2 prior_measurement(0.0, 0.0, 0.0); // prior at origin
	graph.addPrior(x1, prior_measurement, prior_model); // add directly to graph

	/* 2.b add odometry */
	// general noisemodel for odometry
	SharedDiagonal odom_model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));

	/* Pose2 measurements take (x,y,theta), where theta is taken from the positive x-axis*/
	Pose2 odom_measurement(2.0, 0.0, 0.0); // create a measurement for both factors (the same in this case)
	graph.addConstraint(x1, x2, odom_measurement, odom_model);
	graph.addConstraint(x2, x3, odom_measurement, odom_model);
	graph.print("full graph");

    /* 3. Create the data structure to hold the initial estinmate to the solution
     * initialize to noisy points */
	DynamicValues initial;
	initial.insert(x1, Pose2(0.5, 0.0, 0.2));
	initial.insert(x2, Pose2(2.3, 0.1,-0.2));
	initial.insert(x3, Pose2(4.1, 0.1, 0.1));
	initial.print("initial estimate");

	/* 4 Single Step Optimization
	* optimize using Levenberg-Marquardt optimization with an ordering from colamd */
	DynamicValues result = optimize<Graph>(graph, initial);
	result.print("final result");


	return 0;
}
