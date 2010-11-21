/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample.cpp
 * @brief Simple Pose2SLAM Example using
 * pre-built pose2SLAM domain
 * @author Chris Beall
 */

#include <cmath>
#include <iostream>
#include <boost/shared_ptr.hpp>

// pull in the Pose2 SLAM domain with all typedefs and helper functions defined
#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/nonlinear/NonlinearOptimization-inl.h>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

using namespace std;
using namespace gtsam;
using namespace boost;
using namespace gtsam::pose2SLAM;

int main(int argc, char** argv) {
	// create keys for robot positions
	Key x1(1), x2(2), x3(3);

	/* 1. create graph container and add factors to it */
	shared_ptr<Graph> graph(new Graph);

	/* 2.a add prior  */
	// gaussian for prior
	SharedDiagonal prior_model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.3, 0.3, 0.1));
	Pose2 prior_measurement(0.0, 0.0, 0.0); // prior at origin
	graph->addPrior(x1, prior_measurement, prior_model); // add directly to graph

	/* 2.b add odometry */
	// general noisemodel for odometry
	SharedDiagonal odom_model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));

	/* Pose2 measurements take (x,y,theta), where theta is taken from the positive x-axis*/
	Pose2 odom_measurement(2.0, 0.0, 0.0); // create a measurement for both factors (the same in this case)
	graph->addConstraint(x1, x2, odom_measurement, odom_model);
	graph->addConstraint(x2, x3, odom_measurement, odom_model);
	graph->print("full graph");

  /* 3. Create the data structure to hold the initial estimate to the solution
   * initialize to noisy points */
	shared_ptr<Values> initial(new Values);
	initial->insert(x1, Pose2(0.5, 0.0, 0.2));
	initial->insert(x2, Pose2(2.3, 0.1,-0.2));
	initial->insert(x3, Pose2(4.1, 0.1, 0.1));
	initial->print("initial estimate");


	/* 4.2.1 Alternatively, you can go through the process step by step
	 * Choose an ordering */
	Ordering::shared_ptr ordering = graph->orderingCOLAMD(*initial);

	/* 4.2.2 set up solver and optimize */
  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newDrecreaseThresholds(1e-15, 1e-15);
	Optimizer optimizer(graph, initial, ordering, params);
	Optimizer optimizer_result = optimizer.levenbergMarquardt();

	Values result = *optimizer_result.values();
	result.print("final result");

	/* Get covariances */
	Matrix covariance1  = optimizer_result.marginalCovariance(x1).second;
	Matrix covariance2  = optimizer_result.marginalCovariance(x2).second;

	print(covariance1, "Covariance1");
	print(covariance2, "Covariance2");

	return 0;
}

