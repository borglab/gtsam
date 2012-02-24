/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  pose3SLAM.cpp
 *  @brief: bearing/range measurements in 2D plane
 *  @author Frank Dellaert
 **/

#include <gtsam/slam/pose3SLAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

// Use pose3SLAM namespace for specific SLAM instance
namespace gtsam {

	namespace pose3SLAM {

		/* ************************************************************************* */
		Values circle(size_t n, double radius) {
			Values x;
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
				x.insert(PoseKey(i), gTi);
			}
			return x;
		}

		/* ************************************************************************* */
		void Graph::addPrior(Index i, const Pose3& p, const SharedNoiseModel& model) {
			sharedFactor factor(new Prior(PoseKey(i), p, model));
			push_back(factor);
		}

		void Graph::addConstraint(Index i, Index j, const Pose3& z, const SharedNoiseModel& model) {
			sharedFactor factor(new Constraint(PoseKey(i), PoseKey(j), z, model));
			push_back(factor);
		}

		void Graph::addHardConstraint(Index i, const Pose3& p) {
			sharedFactor factor(new HardConstraint(PoseKey(i), p));
			push_back(factor);
		}

	/* ************************************************************************* */

	} // pose3SLAM

} // gtsam
