/**
 *  @file  pose3SLAM.cpp
 *  @brief: bearing/range measurements in 2D plane
 *  @authors Frank Dellaert
 **/

#include <gtsam/slam/pose3SLAM.h>
#include <gtsam/nonlinear/LieConfig-inl.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>

// Use pose3SLAM namespace for specific SLAM instance
namespace gtsam {

	using namespace pose3SLAM;
	INSTANTIATE_LIE_CONFIG(Key, Pose3)
	INSTANTIATE_NONLINEAR_FACTOR_GRAPH(Config)
	INSTANTIATE_NONLINEAR_OPTIMIZER(Graph, Config)

	namespace pose3SLAM {

		/* ************************************************************************* */
		Config circle(size_t n, double radius) {
			Config x;
			double theta = 0, dtheta = 2 * M_PI / n;
			// We use aerospace/navlab convention, X forward, Y right, Z down
			// First pose will be at (R,0,0)
			// ^y   ^ X
			// |    |
			// z-->xZ--> Y  (z pointing towards viewer, Z pointing away from viewer)
			// Vehicle at p0 is looking towards y axis (X-axis points towards world y)
			Rot3 gR0(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
			for (size_t i = 0; i < n; i++, theta += dtheta) {
				Point3 gti(radius*cos(theta), radius*sin(theta), 0);
				Rot3 _0Ri = Rot3::yaw(-theta); // negative yaw goes counterclockwise, with Z down !
				Pose3 gTi(gR0 * _0Ri, gti);
				x.insert(i, gTi);
			}
			return x;
		}

		/* ************************************************************************* */
		void Graph::addPrior(const Key& i, const Pose3& p,
				const SharedGaussian& model) {
			sharedFactor factor(new Prior(i, p, model));
			push_back(factor);
		}

		void Graph::addConstraint(const Key& i, const Key& j, const Pose3& z,
				const SharedGaussian& model) {
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
