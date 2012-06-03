/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PlanarSLAMSelfContained_advanced.cpp
 * @brief Simple robotics example with all typedefs internal to this script.
 * @author Alex Cunningham
 */

// add in headers for specific factors
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

// for all nonlinear keys
#include <gtsam/nonlinear/Symbol.h>

// implementations for structures - needed if self-contained, and these should be included last
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

// for modeling measurement uncertainty - all models included here
#include <gtsam/linear/NoiseModel.h>

// for points and poses
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

#include <cmath>
#include <iostream>

using namespace std;
using namespace gtsam;

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
	// create keys for variables
	Symbol i1('x',1), i2('x',2), i3('x',3);
	Symbol j1('l',1), j2('l',2);

	// create graph container and add factors to it
	NonlinearFactorGraph graph;

	/* add prior  */
	// gaussian for prior
	SharedDiagonal priorNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.3, 0.3, 0.1));
	Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
	PriorFactor<Pose2> posePrior(i1, priorMean, priorNoise); // create the factor
	graph.add(posePrior);  // add the factor to the graph

	/* add odometry */
	// general noisemodel for odometry
	SharedDiagonal odometryNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
	Pose2 odometry(2.0, 0.0, 0.0); // create a measurement for both factors (the same in this case)
	// create between factors to represent odometry
	BetweenFactor<Pose2> odom12(i1, i2, odometry, odometryNoise);
	BetweenFactor<Pose2> odom23(i2, i3, odometry, odometryNoise);
	graph.add(odom12); // add both to graph
	graph.add(odom23);

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

	// create bearing/range factors
	BearingRangeFactor<Pose2, Point2> meas11(i1, j1, bearing11, range11, meas_model);
	BearingRangeFactor<Pose2, Point2> meas21(i2, j1, bearing21, range21, meas_model);
	BearingRangeFactor<Pose2, Point2> meas32(i3, j2, bearing32, range32, meas_model);

	// add the factors
	graph.add(meas11);
	graph.add(meas21);
	graph.add(meas32);

	graph.print("Full Graph");

	// initialize to noisy points
	Values initial;
	initial.insert(i1, Pose2(0.5, 0.0, 0.2));
	initial.insert(i2, Pose2(2.3, 0.1,-0.2));
	initial.insert(i3, Pose2(4.1, 0.1, 0.1));
	initial.insert(j1, Point2(1.8, 2.1));
	initial.insert(j2, Point2(4.1, 1.8));

	initial.print("initial estimate");

	// optimize using Levenberg-Marquardt optimization with an ordering from colamd

	// first using sequential elimination
	LevenbergMarquardtParams lmParams;
	lmParams.linearSolverType = LevenbergMarquardtParams::SEQUENTIAL_CHOLESKY;
	Values resultSequential = LevenbergMarquardtOptimizer(graph, initial, lmParams).optimize();
	resultSequential.print("final result (solved with a sequential solver)");

	// then using multifrontal, advanced interface
	// Note that we keep the original optimizer object so we can use the COLAMD
	// ordering it computes.
	LevenbergMarquardtOptimizer optimizer(graph, initial);
	Values resultMultifrontal = optimizer.optimize();
	resultMultifrontal.print("final result (solved with a multifrontal solver)");

  // Print marginals covariances for all variables
	Marginals marginals(graph, resultMultifrontal, Marginals::CHOLESKY);
  print(marginals.marginalCovariance(i1), "i1 covariance");
  print(marginals.marginalCovariance(i2), "i2 covariance");
  print(marginals.marginalCovariance(i3), "i3 covariance");
  print(marginals.marginalCovariance(j1), "j1 covariance");
  print(marginals.marginalCovariance(j2), "j2 covariance");

	return 0;
}

