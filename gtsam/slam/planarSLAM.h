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
 *  @authors Frank Dellaert
 **/

#pragma once

#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/TupleValues.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

// We use gtsam namespace for generally useful factors
namespace gtsam {

	// Use planarSLAM namespace for specific SLAM instance
	namespace planarSLAM {

		// Keys and Values
		typedef TypedSymbol<Pose2, 'x'> PoseKey;
		typedef TypedSymbol<Point2, 'l'> PointKey;
		typedef LieValues<PoseKey> PoseValues;
		typedef LieValues<PointKey> PointValues;
		typedef TupleValues2<PoseValues, PointValues> Values;

		// Factors
		typedef NonlinearEquality<Values, PoseKey> Constraint;
		typedef PriorFactor<Values, PoseKey> Prior;
		typedef BetweenFactor<Values, PoseKey> Odometry;
		typedef BearingFactor<Values, PoseKey, PointKey> Bearing;
		typedef RangeFactor<Values, PoseKey, PointKey> Range;
		typedef BearingRangeFactor<Values, PoseKey, PointKey> BearingRange;

		// Graph
		struct Graph: public NonlinearFactorGraph<Values> {
			Graph(){}
			Graph(const NonlinearFactorGraph<Values>& graph);
			void addPrior(const PoseKey& i, const Pose2& p, const SharedGaussian& model);
			void addPoseConstraint(const PoseKey& i, const Pose2& p);
			void addOdometry(const PoseKey& i, const PoseKey& j, const Pose2& z,
					const SharedGaussian& model);
			void addBearing(const PoseKey& i, const PointKey& j, const Rot2& z,
					const SharedGaussian& model);
			void addRange(const PoseKey& i, const PointKey& j, double z,
					const SharedGaussian& model);
			void addBearingRange(const PoseKey& i, const PointKey& j,
					const Rot2& z1, double z2, const SharedGaussian& model);
		};

		// Optimizer
		typedef NonlinearOptimizer<Graph, Values> Optimizer;

	} // planarSLAM

} // namespace gtsam

