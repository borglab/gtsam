/**
 *  @file  pose2SLAM.h
 *  @brief: 2D Pose SLAM
 *  @authors Frank Dellaert
 **/

#pragma once

#include "Key.h"
#include "Pose2.h"
#include "LieConfig.h"
#include "PriorFactor.h"
#include "BetweenFactor.h"
#include "NonlinearEquality.h"
#include "NonlinearFactorGraph.h"
#include "NonlinearOptimizer.h"

namespace gtsam {

	// Use pose2SLAM namespace for specific SLAM instance
	namespace pose2SLAM {

		// Keys and Config
		typedef Symbol<Pose2, 'x'> Key;
		typedef LieConfig<Key, Pose2> Config;

		/**
		 * Create a circle of n 2D poses tangent to circle of radius R, first pose at (R,0)
		 * @param n number of poses
		 * @param R radius of circle
		 * @param c character to use for keys
		 * @return circle of n 2D poses
		 */
		Config circle(size_t n, double R);

		// Factors
		typedef PriorFactor<Config, Key, Pose2> Prior;
		typedef BetweenFactor<Config, Key, Pose2> Constraint;
		typedef NonlinearEquality<Config, Key, Pose2> HardConstraint;

		// Graph
		struct Graph: public NonlinearFactorGraph<Config> {
			void addPrior(const Key& i, const Pose2& p, const Matrix& cov);
			void addConstraint(const Key& i, const Key& j, const Pose2& z, const Matrix& cov);
			void addHardConstraint(const Key& i, const Pose2& p);
		};

		// Optimizer
		typedef NonlinearOptimizer<Graph, Config> Optimizer;

	} // pose2SLAM

	/**
	 * Backwards compatibility
	 */
	typedef pose2SLAM::Config Pose2Config;
	typedef pose2SLAM::Prior Pose2Prior;
	typedef pose2SLAM::Constraint Pose2Factor;
	typedef pose2SLAM::Graph Pose2Graph;

} // namespace gtsam

