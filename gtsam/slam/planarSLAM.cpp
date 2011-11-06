/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  planarSLAM.cpp
 *  @brief: bearing/range measurements in 2D plane
 *  @author Frank Dellaert
 **/

#include <gtsam/slam/planarSLAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/nonlinear/NonlinearOptimization-inl.h>
#include <gtsam/nonlinear/TupleValues-inl.h>

// Use planarSLAM namespace for specific SLAM instance
namespace gtsam {

	INSTANTIATE_VALUES(planarSLAM::PointKey)
	INSTANTIATE_TUPLE_VALUES2(planarSLAM::PoseValues, planarSLAM::PointValues)
	INSTANTIATE_NONLINEAR_FACTOR_GRAPH(planarSLAM::Values)
	INSTANTIATE_NONLINEAR_OPTIMIZER(planarSLAM::Graph, planarSLAM::Values)

	namespace planarSLAM {

		Graph::Graph(const NonlinearFactorGraph<Values>& graph) :
				NonlinearFactorGraph<Values>(graph) {}

		void Graph::addPrior(const PoseKey& i, const Pose2& p,
				const SharedNoiseModel& model) {
			sharedFactor factor(new Prior(i, p, model));
			push_back(factor);
		}

	  void Graph::addPoseConstraint(const PoseKey& i, const Pose2& p) {
	  	sharedFactor factor(new Constraint(i, p));
			push_back(factor);
		}

		void Graph::addOdometry(const PoseKey& i, const PoseKey& j, const Pose2& z,
				const SharedNoiseModel& model) {
			sharedFactor factor(new Odometry(i, j, z, model));
			push_back(factor);
		}

		void Graph::addBearing(const PoseKey& i, const PointKey& j, const Rot2& z,
				const SharedNoiseModel& model) {
			sharedFactor factor(new Bearing(i, j, z, model));
			push_back(factor);
		}

		void Graph::addRange(const PoseKey& i, const PointKey& j, double z,
				const SharedNoiseModel& model) {
			sharedFactor factor(new Range(i, j, z, model));
			push_back(factor);
		}

		void Graph::addBearingRange(const PoseKey& i, const PointKey& j, const Rot2& z1,
				double z2, const SharedNoiseModel& model) {
			sharedFactor factor(new BearingRange(i, j, z1, z2, model));
			push_back(factor);
		}

	} // planarSLAM

} // gtsam
