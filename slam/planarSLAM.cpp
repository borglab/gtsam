/**
 *  @file  planarSLAM.cpp
 *  @brief: bearing/range measurements in 2D plane
 *  @authors Frank Dellaert
 **/

#include <gtsam/slam/planarSLAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/nonlinear/TupleConfig-inl.h>

// Use planarSLAM namespace for specific SLAM instance
namespace gtsam {

	using namespace planarSLAM;
	INSTANTIATE_LIE_CONFIG(PointKey)
	INSTANTIATE_TUPLE_CONFIG2(PoseConfig, PointConfig)
	INSTANTIATE_NONLINEAR_FACTOR_GRAPH(Config)
	INSTANTIATE_NONLINEAR_OPTIMIZER(Graph, Config)

	namespace planarSLAM {

		Graph::Graph(const NonlinearFactorGraph<Config>& graph) :
				NonlinearFactorGraph<Config>(graph) {}

		void Graph::addPrior(const PoseKey& i, const Pose2& p,
				const SharedGaussian& model) {
			sharedFactor factor(new Prior(i, p, model));
			push_back(factor);
		}

	  void Graph::addPoseConstraint(const PoseKey& i, const Pose2& p) {
	  	sharedFactor factor(new Constraint(i, p));
			push_back(factor);
		}

		void Graph::addOdometry(const PoseKey& i, const PoseKey& j, const Pose2& z,
				const SharedGaussian& model) {
			sharedFactor factor(new Odometry(i, j, z, model));
			push_back(factor);
		}

		void Graph::addBearing(const PoseKey& i, const PointKey& j, const Rot2& z,
				const SharedGaussian& model) {
			sharedFactor factor(new Bearing(i, j, z, model));
			push_back(factor);
		}

		void Graph::addRange(const PoseKey& i, const PointKey& j, double z,
				const SharedGaussian& model) {
			sharedFactor factor(new Range(i, j, z, model));
			push_back(factor);
		}

		void Graph::addBearingRange(const PoseKey& i, const PointKey& j, const Rot2& z1,
				double z2, const SharedGaussian& model) {
			sharedFactor factor(new BearingRange(i, j, z1, z2, model));
			push_back(factor);
		}

	} // planarSLAM

} // gtsam
