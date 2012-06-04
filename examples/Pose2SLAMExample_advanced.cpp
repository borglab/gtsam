/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample_advanced.cpp
 * @brief Simple Pose2SLAM Example using
 * pre-built pose2SLAM domain
 * @author Chris Beall
 */

// pull in the Pose2 SLAM domain with all typedefs and helper functions defined
#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <iostream>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
	/* 1. create graph container and add factors to it */
	pose2SLAM::Graph graph;

	/* 2.a add prior  */
	Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
	SharedDiagonal priorNoise(Vector_(3, 0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
	graph.addPrior(1, priorMean, priorNoise); // add directly to graph

	/* 2.b add odometry */
	SharedDiagonal odometryNoise(Vector_(3, 0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta
	Pose2 odometry(2.0, 0.0, 0.0); // create a measurement for both factors (the same in this case)
	graph.addOdometry(1, 2, odometry, odometryNoise);
	graph.addOdometry(2, 3, odometry, odometryNoise);
	graph.print("full graph");

	/* 3. Create the data structure to hold the initial estimate to the solution
	 * initialize to noisy points */
	pose2SLAM::Values initial;
	initial.insertPose(1, Pose2(0.5, 0.0, 0.2));
	initial.insertPose(2, Pose2(2.3, 0.1, -0.2));
	initial.insertPose(3, Pose2(4.1, 0.1, 0.1));
	initial.print("initial estimate");

	/* 4.2.1 Alternatively, you can go through the process step by step
	 * Choose an ordering */
	Ordering ordering = *graph.orderingCOLAMD(initial);

	/* 4.2.2 set up solver and optimize */
	LevenbergMarquardtParams params;
	params.absoluteErrorTol = 1e-15;
	params.relativeErrorTol = 1e-15;
	params.ordering = ordering;
	LevenbergMarquardtOptimizer optimizer(graph, initial, params);

	pose2SLAM::Values result = optimizer.optimize();
	result.print("final result");

	/* Get covariances */
	Marginals marginals(graph, result, Marginals::CHOLESKY);
	Matrix covariance1 = marginals.marginalCovariance(1);
	Matrix covariance2 = marginals.marginalCovariance(2);

	print(covariance1, "Covariance1");
	print(covariance2, "Covariance2");

	return 0;
}

