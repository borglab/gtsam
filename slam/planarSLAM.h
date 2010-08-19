/**
 *  @file  planarSLAM.h
 *  @brief: bearing/range measurements in 2D plane
 *  @authors Frank Dellaert
 **/

#pragma once

#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/TupleConfig.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

// We use gtsam namespace for generally useful factors
namespace gtsam {

	// Use planarSLAM namespace for specific SLAM instance
	namespace planarSLAM {

		// Keys and Config
		typedef TypedSymbol<Pose2, 'x'> PoseKey;
		typedef TypedSymbol<Point2, 'l'> PointKey;
		typedef LieConfig<PoseKey, Pose2> PoseConfig;
		typedef LieConfig<PointKey, Point2> PointConfig;
		typedef TupleConfig2<PoseConfig, PointConfig> Config;

		// Factors
		typedef NonlinearEquality<Config, PoseKey, Pose2> Constraint;
		typedef PriorFactor<Config, PoseKey, Pose2> Prior;
		typedef BetweenFactor<Config, PoseKey, Pose2> Odometry;
		typedef BearingFactor<Config, PoseKey, PointKey> Bearing;
		typedef RangeFactor<Config, PoseKey, PointKey> Range;
		typedef BearingRangeFactor<Config, PoseKey, PointKey> BearingRange;

		// Graph
		struct Graph: public NonlinearFactorGraph<Config> {
			Graph(){}
			Graph(const NonlinearFactorGraph<Config>& graph);
			void addPrior(const PoseKey& i, const Pose2& p, const SharedGaussian& model);
			void addPoseConstraint(const PoseKey& i, const Pose2& p);
			void addOdometry(const PoseKey& i, const PoseKey& j, const Pose2& z,
					const SharedGaussian& model);
			void addBearing(const PoseKey& i, const PointKey& j, const Rot2& z,
					const SharedGaussian& model);
			void addRange(const PoseKey& i, const PointKey& j, double z,
					const SharedGaussian& model);
		};

		// Optimizer
		typedef NonlinearOptimizer<Graph, Config> Optimizer;

	} // planarSLAM

} // namespace gtsam

