/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  planarSLAM.h
 *  @brief: bearing/range measurements in 2D plane
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/BearingFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimization.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/geometry/Pose2.h>

// We use gtsam namespace for generally useful factors
namespace gtsam {

	// Use planarSLAM namespace for specific SLAM instance
	namespace planarSLAM {

		/// Typedef for a PoseKey with Pose2 data and 'x' symbol
		typedef TypedSymbol<Pose2, 'x'> PoseKey;

		/// Typedef for a PointKey with Point2 data and 'l' symbol
		typedef TypedSymbol<Point2, 'l'> PointKey;

		/**
		 * List of typedefs for factors
		 */

		/// A hard constraint for PoseKeys to enforce particular values
		typedef NonlinearEquality<PoseKey> Constraint;
		/// A prior factor to bias the value of a PoseKey
		typedef PriorFactor<PoseKey> Prior;
		/// A factor between two PoseKeys set with a Pose2
		typedef BetweenFactor<PoseKey> Odometry;
		/// A factor between a PoseKey and a PointKey to express difference in rotation (set with a Rot2)
		typedef BearingFactor<PoseKey, PointKey> Bearing;
		/// A factor between a PoseKey and a PointKey to express distance between them (set with a double)
		typedef RangeFactor<PoseKey, PointKey> Range;
		/// A factor between a PoseKey and a PointKey to express difference in rotation and location
		typedef BearingRangeFactor<PoseKey, PointKey> BearingRange;

		/// Creates a NonlinearFactorGraph with the Values type
		struct Graph: public NonlinearFactorGraph {

			/// Default constructor for a NonlinearFactorGraph
			Graph(){}

			/// Creates a NonlinearFactorGraph based on another NonlinearFactorGraph
			Graph(const NonlinearFactorGraph& graph);

			/// Biases the value of PoseKey key with Pose2 p given a noise model
			void addPrior(const PoseKey& key, const Pose2& pose, const SharedNoiseModel& noiseModel);

			/// Creates a hard constraint to enforce Pose2 p for PoseKey poseKey's value
			void addPoseConstraint(const PoseKey& poseKey, const Pose2& pose);

			/// Creates a factor with a Pose2 between PoseKeys poseKey and pointKey (poseKey.e. an odometry measurement)
			void addOdometry(const PoseKey& poseKey, const PoseKey& pointKey, const Pose2& odometry,
					const SharedNoiseModel& model);

			/// Creates a factor with a Rot2 between a PoseKey poseKey and PointKey pointKey for difference in rotation
			void addBearing(const PoseKey& poseKey, const PointKey& pointKey, const Rot2& bearing,
					const SharedNoiseModel& model);

			/// Creates a factor with a Rot2 between a PoseKey poseKey and PointKey pointKey for difference in location
			void addRange(const PoseKey& poseKey, const PointKey& pointKey, double range,
					const SharedNoiseModel& model);

			/// Creates a factor with a Rot2 between a PoseKey poseKey and PointKey pointKey for difference in rotation and location
			void addBearingRange(const PoseKey& poseKey, const PointKey& pointKey,
					const Rot2& bearing, double range, const SharedNoiseModel& model);

			/// Optimize
			Values optimize(const Values& initialEstimate) {
				typedef NonlinearOptimizer<Graph> Optimizer;
				return *Optimizer::optimizeLM(*this, initialEstimate,
										NonlinearOptimizationParameters::LAMBDA);
			}
		};

		/// Optimizer
		typedef NonlinearOptimizer<Graph> Optimizer;

	} // planarSLAM

} // namespace gtsam

