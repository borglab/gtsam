/**
 *  @file  pose3SLAM.cpp
 *  @brief: bearing/range measurements in 2D plane
 *  @authors Frank Dellaert
 **/

#include "pose3SLAM.h"
#include "LieConfig-inl.h"
#include "NonlinearFactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"

// Use pose3SLAM namespace for specific SLAM instance
namespace gtsam {

	using namespace pose3SLAM;
	INSTANTIATE_LIE_CONFIG(Key, Pose3)
	INSTANTIATE_NONLINEAR_FACTOR_GRAPH(Config)
	INSTANTIATE_NONLINEAR_OPTIMIZER(Graph, Config)

	namespace pose3SLAM {

		/* ************************************************************************* */
		Config circle(size_t n, double R) {
			Config x;
			double theta = 0, dtheta = 2 * M_PI / n;
			// Vehicle at p0 is looking towards y axis
			Rot3 R0(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
			for (size_t i = 0; i < n; i++, theta += dtheta)
				x.insert(i, Pose3(R0 * Rot3::yaw(-theta), Point3(cos(theta),
						sin(theta), 0)));
			return x;
		}

		/* ************************************************************************* */
		void Graph::addPrior(const Key& i, const Pose3& p,
				const sharedGaussian& model) {
			sharedFactor factor(new Prior(i, p, model));
			push_back(factor);
		}

		void Graph::addConstraint(const Key& i, const Key& j, const Pose3& z,
				const sharedGaussian& model) {
			sharedFactor factor(new Constraint(i, j, z, model));
			push_back(factor);
		}

		void Graph::addHardConstraint(const Key& i, const Pose3& p) {
			sharedFactor factor(new HardConstraint(i, p));
			push_back(factor);
		}

	/* ************************************************************************* */

	} // pose3SLAM

} // gtsam
