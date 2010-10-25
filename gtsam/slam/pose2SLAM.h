/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  pose2SLAM.h
 *  @brief: 2D Pose SLAM
 *  @authors Frank Dellaert
 **/

#pragma once

#include <gtsam/base/LieVector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LieValues.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>

namespace gtsam {

	// Use pose2SLAM namespace for specific SLAM instance
	namespace pose2SLAM {

		// Keys and Values
		typedef TypedSymbol<Pose2, 'x'> Key;
		typedef LieValues<Key> Values;

		/**
		 * Create a circle of n 2D poses tangent to circle of radius R, first pose at (R,0)
		 * @param n number of poses
		 * @param R radius of circle
		 * @param c character to use for keys
		 * @return circle of n 2D poses
		 */
		Values circle(size_t n, double R);

		// Factors
		typedef PriorFactor<Values, Key> Prior;
		typedef BetweenFactor<Values, Key> Constraint;
		typedef NonlinearEquality<Values, Key> HardConstraint;

		// Graph
		struct Graph: public NonlinearFactorGraph<Values> {
			typedef BetweenFactor<Values, Key> Constraint;
			typedef Pose2 Pose;
			void addPrior(const Key& i, const Pose2& p, const SharedGaussian& model);
			void addConstraint(const Key& i, const Key& j, const Pose2& z, const SharedGaussian& model);
			void addHardConstraint(const Key& i, const Pose2& p);
		};

		// Optimizer
		typedef NonlinearOptimizer<Graph, Values, GaussianFactorGraph, GaussianSequentialSolver> OptimizerSequential;
    typedef NonlinearOptimizer<Graph, Values, GaussianFactorGraph, GaussianMultifrontalSolver> Optimizer;

	} // pose2SLAM

	/**
	 * Backwards compatibility
	 */
	typedef pose2SLAM::Values Pose2Values;
	typedef pose2SLAM::Prior Pose2Prior;
	typedef pose2SLAM::Constraint Pose2Factor;
	typedef pose2SLAM::Graph Pose2Graph;

} // namespace gtsam

