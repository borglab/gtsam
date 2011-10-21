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

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/TupleValues.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/BearingFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

// We use gtsam namespace for generally useful factors
namespace gtsam {

	// Use planarSLAM namespace for specific SLAM instance
	namespace planarSLAM {

		/// Typedef for a PoseKey with Pose2 data and 'x' symbol
		typedef TypedSymbol<Pose2, 'x'> PoseKey;

		/// Typedef for a PointKey with Point2 data and 'l' symbol
		typedef TypedSymbol<Point2, 'l'> PointKey;

		/// Typedef for LieValues structure with PoseKey type
		typedef LieValues<PoseKey> PoseValues;

		/// Typedef for LieValues structure with PointKey type
		typedef LieValues<PointKey> PointValues;

		/// Typedef for the TupleValues2 to use PoseKeys and PointKeys
		typedef TupleValues2<PoseValues, PointValues> Values;

		/**
		 * List of typedefs for factors
		 */

		/// A hard constraint for PoseKeys to enforce particular values
		typedef NonlinearEquality<Values, PoseKey> Constraint;
		/// A prior factor to bias the value of a PoseKey
		typedef PriorFactor<Values, PoseKey> Prior;
		/// A factor between two PoseKeys set with a Pose2
		typedef BetweenFactor<Values, PoseKey> Odometry;
		/// A factor between a PoseKey and a PointKey to express difference in rotation (set with a Rot2)
		typedef BearingFactor<Values, PoseKey, PointKey> Bearing;
		/// A factor between a PoseKey and a PointKey to express distance between them (set with a double)
		typedef RangeFactor<Values, PoseKey, PointKey> Range;
		/// A factor between a PoseKey and a PointKey to express difference in rotation and location
		typedef BearingRangeFactor<Values, PoseKey, PointKey> BearingRange;

		/// Creates a NonlinearFactorGraph with the Values type
		struct Graph: public NonlinearFactorGraph<Values> {

			/// Default constructor for a NonlinearFactorGraph
			Graph(){}

			/// Creates a NonlinearFactorGraph based on another NonlinearFactorGraph
			Graph(const NonlinearFactorGraph<Values>& graph);

			/// Biases the value of PoseKey i with Pose2 p given a noise model
			void addPrior(const PoseKey& i, const Pose2& p, const SharedNoiseModel& model);

			/// Creates a hard constraint to enforce Pose2 p for PoseKey i's value
			void addPoseConstraint(const PoseKey& i, const Pose2& p);

			/// Creates a factor with a Pose2 between PoseKeys i and j (i.e. an odometry measurement)
			void addOdometry(const PoseKey& i, const PoseKey& j, const Pose2& z,
					const SharedNoiseModel& model);

			/// Creates a factor with a Rot2 between a PoseKey i and PointKey j for difference in rotation
			void addBearing(const PoseKey& i, const PointKey& j, const Rot2& z,
					const SharedNoiseModel& model);

			/// Creates a factor with a Rot2 between a PoseKey i and PointKey j for difference in location
			void addRange(const PoseKey& i, const PointKey& j, double z,
					const SharedNoiseModel& model);

			/// Creates a factor with a Rot2 between a PoseKey i and PointKey j for difference in rotation and location
			void addBearingRange(const PoseKey& i, const PointKey& j,
					const Rot2& z1, double z2, const SharedNoiseModel& model);
		};

		/// Optimizer
		typedef NonlinearOptimizer<Graph, Values> Optimizer;

	} // planarSLAM

} // namespace gtsam

