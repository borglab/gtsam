/**
 *  @file  pose2SLAM.cpp
 *  @brief: bearing/range measurements in 2D plane
 *  @authors Frank Dellaert
 **/

#include "pose2SLAM.h"
#include "LieConfig-inl.h"
#include "NonlinearFactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"

// Use pose2SLAM namespace for specific SLAM instance
namespace gtsam {

	using namespace pose2SLAM;
	INSTANTIATE_LIE_CONFIG(Key, Pose2)
	INSTANTIATE_NONLINEAR_FACTOR_GRAPH(Config)
	INSTANTIATE_NONLINEAR_OPTIMIZER(Graph, Config)

	namespace pose2SLAM {

		/* ************************************************************************* */
		Config circle(size_t n, double R) {
			Config x;
			double theta = 0, dtheta = 2 * M_PI / n;
			for (size_t i = 0; i < n; i++, theta += dtheta)
				x.insert(i, Pose2(cos(theta), sin(theta), M_PI_2 + theta));
			return x;
		}

		/* ************************************************************************* */
		void Graph::addPrior(const Key& i, const Pose2& p,
				const sharedGaussian& model) {
			sharedFactor factor(new Prior(i, p, model));
			push_back(factor);
		}

		void Graph::addConstraint(const Key& i, const Key& j, const Pose2& z,
				const sharedGaussian& model) {
			sharedFactor factor(new Constraint(i, j, z, model));
			push_back(factor);
		}

		void Graph::addHardConstraint(const Key& i, const Pose2& p) {
			sharedFactor factor(new HardConstraint(i, p));
			push_back(factor);
		}

	/* ************************************************************************* */

	} // pose2SLAM

} // gtsam
