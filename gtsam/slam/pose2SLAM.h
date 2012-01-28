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
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/base/LieVector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
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

		/// Keys with Pose2 and symbol 'x'
		typedef TypedSymbol<Pose2, 'x'> Key;

		/**
		 * Create a circle of n 2D poses tangent to circle of radius R, first pose at (R,0)
		 * @param n number of poses
		 * @param R radius of circle
		 * @param c character to use for keys
		 * @return circle of n 2D poses
		 */
		DynamicValues circle(size_t n, double R);

		/// A prior factor on Key with Pose2 data type.
		typedef PriorFactor<Key> Prior;

		/// A factor to put constraints between two factors.
		typedef BetweenFactor<Key> Constraint;

		/// A hard constraint would enforce that the given key would have the input value in the results.
		typedef NonlinearEquality<Key> HardConstraint;

		/// Graph
		struct Graph: public NonlinearFactorGraph {

			/// Adds a factor between keys of the same type
			typedef BetweenFactor<Key> Constraint;

			/// A simple typedef from Pose2 to Pose
			typedef Pose2 Pose;

			/// Adds a Pose2 prior with a noise model to one of the keys in the nonlinear factor graph
			void addPrior(const Key& i, const Pose2& p, const SharedNoiseModel& model);

			/// Creates a between factor between keys i and j with a noise model with Pose2 z in the graph
			void addConstraint(const Key& i, const Key& j, const Pose2& z, const SharedNoiseModel& model);

			/// Creates a hard constraint for key i with the given Pose2 p.
			void addHardConstraint(const Key& i, const Pose2& p);
		};

		/// The sequential optimizer
		typedef NonlinearOptimizer<Graph, DynamicValues, GaussianFactorGraph, GaussianSequentialSolver> OptimizerSequential;

		/// The multifrontal optimizer
		typedef NonlinearOptimizer<Graph, DynamicValues, GaussianFactorGraph, GaussianMultifrontalSolver> Optimizer;

	} // pose2SLAM

	/**
	 * Backwards compatibility
	 */
	typedef pose2SLAM::Prior Pose2Prior;			///< Typedef for Prior class for backwards compatibility
	typedef pose2SLAM::Constraint Pose2Factor;		///< Typedef for Constraint class for backwards compatibility
	typedef pose2SLAM::Graph Pose2Graph;			///< Typedef for Graph class for backwards compatibility

} // namespace gtsam

