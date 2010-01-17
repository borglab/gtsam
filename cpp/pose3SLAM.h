/**
 *  @file  pose3SLAM.h
 *  @brief: 3D Pose SLAM
 *  @authors Frank Dellaert
 **/

#pragma once

#include "Key.h"
#include "Pose3.h"
#include "LieConfig.h"
#include "PriorFactor.h"
#include "BetweenFactor.h"
#include "NonlinearEquality.h"
#include "NonlinearFactorGraph.h"
#include "NonlinearOptimizer.h"

namespace gtsam {

	// Use pose3SLAM namespace for specific SLAM instance
	namespace pose3SLAM {

		// Keys and Config
		typedef TypedSymbol<Pose3, 'x'> Key;
		typedef LieConfig<Key, Pose3> Config;

		/**
		 * Create a circle of n 3D poses tangent to circle of radius R, first pose at (R,0)
		 * @param n number of poses
		 * @param R radius of circle
		 * @param c character to use for keys
		 * @return circle of n 3D poses
		 */
		Config circle(size_t n, double R);

		// Factors
		typedef PriorFactor<Config, Key, Pose3> Prior;
		typedef BetweenFactor<Config, Key, Pose3> Constraint;
		typedef NonlinearEquality<Config, Key, Pose3> HardConstraint;

		// Graph
		struct Graph: public NonlinearFactorGraph<Config> {
			void addPrior(const Key& i, const Pose3& p, const Matrix& cov);
			void addConstraint(const Key& i, const Key& j, const Pose3& z, const Matrix& cov);
			void addHardConstraint(const Key& i, const Pose3& p);
		};

		// Optimizer
		typedef NonlinearOptimizer<Graph, Config> Optimizer;

	} // pose3SLAM

	/**
	 * Backwards compatibility
	 */
	typedef pose3SLAM::Config Pose3Config;
	typedef pose3SLAM::Prior Pose3Prior;
	typedef pose3SLAM::Constraint Pose3Factor;
	typedef pose3SLAM::Graph Pose3Graph;

} // namespace gtsam

