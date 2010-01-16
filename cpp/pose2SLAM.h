/**
 *  @file  pose2SLAM.h
 *  @brief: bearing/range measurements in 2D plane
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
		typedef BetweenFactor<Config, Key, Pose2> Odometry;
		typedef NonlinearEquality<Config, Key, Pose2> Constraint;

		// Graph
		struct Graph: public NonlinearFactorGraph<Config> {
			void addConstraint(const Key& i, const Pose2& p);
			void addPrior(const Key& i, const Pose2& p, const Matrix& cov);
			void add(const Key& i, const Key& j, const Pose2& z, const Matrix& cov);
		};

		// Optimizer
		typedef NonlinearOptimizer<Graph, Config> Optimizer;

	} // pose2SLAM

	/**
	 * Backwards compatibility
	 */
	typedef pose2SLAM::Prior Pose2Prior;
	typedef pose2SLAM::Odometry Pose2Factor;
	typedef pose2SLAM::Constraint Pose2Constraint;
	typedef pose2SLAM::Config Pose2Config;
	typedef pose2SLAM::Graph Pose2Graph;

} // namespace gtsam

