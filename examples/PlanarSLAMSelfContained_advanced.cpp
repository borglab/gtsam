/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PlanarSLAMSelfContained.cpp
 * @brief Simple robotics example from tutorial Figure 1.1 (left), with all typedefs
 * internal to this script.
 * @author Alex Cunningham
 */

#include <cmath>
#include <iostream>

// for all nonlinear keys
#include <gtsam/nonlinear/Key.h>

// for points and poses
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

// for modeling measurement uncertainty - all models included here
#include <gtsam/linear/NoiseModel.h>

// add in headers for specific factors
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

// implementations for structures - needed if self-contained, and these should be included last
#include <gtsam/nonlinear/TupleValues-inl.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>

// Main typedefs
typedef gtsam::TypedSymbol<gtsam::Pose2, 'x'> PoseKey;       // Key for poses, with type included
typedef gtsam::TypedSymbol<gtsam::Point2,'l'> PointKey;      // Key for points, with type included
typedef gtsam::LieValues<PoseKey> PoseValues;                // config type for poses
typedef gtsam::LieValues<PointKey> PointValues;              // config type for points
typedef gtsam::TupleValues2<PoseValues, PointValues> Values; // main config with two variable classes
typedef gtsam::NonlinearFactorGraph<Values> Graph;			 // graph structure
typedef gtsam::NonlinearOptimizer<Graph,Values,gtsam::GaussianFactorGraph,gtsam::GaussianSequentialSolver> OptimizerSeqential;   // optimization engine for this domain
typedef gtsam::NonlinearOptimizer<Graph,Values,gtsam::GaussianFactorGraph,gtsam::GaussianMultifrontalSolver> OptimizerMultifrontal;   // optimization engine for this domain

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
	PoseKey x1(1), x2(2), x3(3);
	PointKey l1(1), l2(2);

	// create graph container and add factors to it
	Graph::shared_ptr graph(new Graph);

	/* add prior  */
	// gaussian for prior
	SharedDiagonal prior_model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.3, 0.3, 0.1));
	Pose2 prior_measurement(0.0, 0.0, 0.0); // prior at origin
	PriorFactor<Values, PoseKey> posePrior(x1, prior_measurement, prior_model); // create the factor
	graph->add(posePrior);  // add the factor to the graph

	/* add odometry */
	// general noisemodel for odometry
	SharedDiagonal odom_model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
	Pose2 odom_measurement(2.0, 0.0, 0.0); // create a measurement for both factors (the same in this case)
	// create between factors to represent odometry
	BetweenFactor<Values,PoseKey> odom12(x1, x2, odom_measurement, odom_model);
	BetweenFactor<Values,PoseKey> odom23(x2, x3, odom_measurement, odom_model);
	graph->add(odom12); // add both to graph
	graph->add(odom23);

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
	BearingRangeFactor<Values, PoseKey, PointKey> meas11(x1, l1, bearing11, range11, meas_model);
	BearingRangeFactor<Values, PoseKey, PointKey> meas21(x2, l1, bearing21, range21, meas_model);
	BearingRangeFactor<Values, PoseKey, PointKey> meas32(x3, l2, bearing32, range32, meas_model);

	// add the factors
	graph->add(meas11);
	graph->add(meas21);
	graph->add(meas32);

	graph->print("Full Graph");

	// initialize to noisy points
	boost::shared_ptr<Values> initial(new Values);
	initial->insert(x1, Pose2(0.5, 0.0, 0.2));
	initial->insert(x2, Pose2(2.3, 0.1,-0.2));
	initial->insert(x3, Pose2(4.1, 0.1, 0.1));
	initial->insert(l1, Point2(1.8, 2.1));
	initial->insert(l2, Point2(4.1, 1.8));

	initial->print("initial estimate");

	// optimize using Levenberg-Marquardt optimization with an ordering from colamd

	// first using sequential elimination
	OptimizerSeqential::shared_values resultSequential = OptimizerSeqential::optimizeLM(*graph, *initial);
	resultSequential->print("final result (solved with a sequential solver)");

	// then using multifrontal, advanced interface
	// Note how we create an optimizer, call LM, then we get access to covariances
	Ordering::shared_ptr ordering = graph->orderingCOLAMD(*initial);
  OptimizerMultifrontal optimizerMF(graph, initial, ordering);
  OptimizerMultifrontal resultMF = optimizerMF.levenbergMarquardt();
  resultMF.values()->print("final result (solved with a multifrontal solver)");

  // Print marginals covariances for all variables
  print(resultMF.marginalCovariance(x1).second, "x1 covariance");
  print(resultMF.marginalCovariance(x2).second, "x2 covariance");
  print(resultMF.marginalCovariance(x3).second, "x3 covariance");
  print(resultMF.marginalCovariance(l1).second, "l1 covariance");
  print(resultMF.marginalCovariance(l2).second, "l2 covariance");

	return 0;
}

