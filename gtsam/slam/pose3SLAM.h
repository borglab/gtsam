/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  pose3SLAM.h
 *  @brief: 3D Pose SLAM
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

	/// Use pose3SLAM namespace for specific SLAM instance
	namespace pose3SLAM {

		/// Creates a Key with data Pose3 and symbol 'x'
		typedef TypedSymbol<Pose3, 'x'> Key;

		/**
		 * Create a circle of n 3D poses tangent to circle of radius R, first pose at (R,0)
		 * @param n number of poses
		 * @param R radius of circle
		 * @return circle of n 3D poses
		 */
		Values circle(size_t n, double R);

		/// A prior factor on Key with Pose3 data type.
		typedef PriorFactor<Key> Prior;
		/// A factor to put constraints between two factors.
		typedef BetweenFactor<Key> Constraint;
		/// A hard constraint would enforce that the given key would have the input value in the results.
		typedef NonlinearEquality<Key> HardConstraint;

		/// Graph
		struct Graph: public NonlinearFactorGraph {

			/// Adds a factor between keys of the same type
			void addPrior(const Key& i, const Pose3& p,
					const SharedNoiseModel& model);

			/// Creates a between factor between keys i and j with a noise model with Pos3 z in the graph
			void addConstraint(const Key& i, const Key& j, const Pose3& z,
					const SharedNoiseModel& model);

			/// Creates a hard constraint for key i with the given Pose3 p.
			void addHardConstraint(const Key& i, const Pose3& p);
		};

		/// Optimizer
		typedef NonlinearOptimizer<Graph> Optimizer;

	} // pose3SLAM

	/**
	 * Backwards compatibility
	 */
	typedef pose3SLAM::Prior Pose3Prior;			///< Typedef for Prior class for backwards compatibility
	typedef pose3SLAM::Constraint Pose3Factor;		///< Typedef for Constraint class for backwards compatibility
	typedef pose3SLAM::Graph Pose3Graph;			///< Typedef for Graph class for backwards compatibility

} // namespace gtsam

