/**
 *  @file  pose2SLAM.h
 *  @brief: 2D Pose SLAM
 *  @authors Frank Dellaert
 **/

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LieConfig.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

	// Use pose2SLAM namespace for specific SLAM instance
	namespace pose2SLAM {

		// Keys and Config
		typedef TypedSymbol<Pose2, 'x'> Key;
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
			typedef BetweenFactor<Config, Key, Pose2> Constraint;
			typedef Pose2 Pose;
			void addPrior(const Key& i, const Pose2& p, const SharedGaussian& model);
			void addConstraint(const Key& i, const Key& j, const Pose2& z, const SharedGaussian& model);
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

