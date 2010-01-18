/**
 *  @file  planarSLAM.cpp
 *  @brief: bearing/range measurements in 2D plane
 *  @authors Frank Dellaert
 **/

#include "planarSLAM.h"
#include "NonlinearFactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"
#include "TupleConfig-inl.h"

// Use planarSLAM namespace for specific SLAM instance
namespace gtsam {

	using namespace planarSLAM;
	INSTANTIATE_LIE_CONFIG(PointKey, Point2)
	INSTANTIATE_PAIR_CONFIG(PoseKey, Pose2, PointKey, Point2)
	INSTANTIATE_NONLINEAR_FACTOR_GRAPH(Config)
	INSTANTIATE_NONLINEAR_OPTIMIZER(Graph, Config)

	namespace planarSLAM {

	  void Graph::addPoseConstraint(const PoseKey& i, const Pose2& p) {
	  	sharedFactor factor(new Constraint(i, p));
			push_back(factor);
		}

		void Graph::addOdometry(const PoseKey& i, const PoseKey& j, const Pose2& z,
				const sharedGaussian& model) {
			sharedFactor factor(new Odometry(i, j, z, model));
			push_back(factor);
		}

		void Graph::addBearing(const PoseKey& i, const PointKey& j, const Rot2& z,
				const sharedGaussian& model) {
			sharedFactor factor(new Bearing(i, j, z, model));
			push_back(factor);
		}

		void Graph::addRange(const PoseKey& i, const PointKey& j, double z,
				const sharedGaussian& model) {
			sharedFactor factor(new Range(i, j, z, model));
			push_back(factor);
		}

	} // planarSLAM

} // gtsam
