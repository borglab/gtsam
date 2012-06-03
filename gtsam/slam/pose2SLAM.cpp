/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  pose2SLAM.cpp
 *  @brief: Odometry measurements in 2D plane
 *  @author Frank Dellaert
 **/

#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Use pose2SLAM namespace for specific SLAM instance

namespace pose2SLAM {

  /* ************************************************************************* */
  Values circle(size_t n, double R) {
    Values x;
    double theta = 0, dtheta = 2 * M_PI / n;
    for (size_t i = 0; i < n; i++, theta += dtheta)
      x.insert(i, Pose2(cos(theta), sin(theta), M_PI_2 + theta));
    return x;
  }

  /* ************************************************************************* */
	Vector Values::xs() const {
		size_t j=0;
		Vector result(size());
		ConstFiltered<Pose2> poses = filter<Pose2>();
		BOOST_FOREACH(const ConstFiltered<Pose2>::KeyValuePair& keyValue, poses)
			result(j++) = keyValue.value.x();
		return result;
	}

  /* ************************************************************************* */
	Vector Values::ys() const {
		size_t j=0;
		Vector result(size());
		ConstFiltered<Pose2> poses = filter<Pose2>();
		BOOST_FOREACH(const ConstFiltered<Pose2>::KeyValuePair& keyValue, poses)
			result(j++) = keyValue.value.y();
		return result;
	}

  /* ************************************************************************* */
	Vector Values::thetas() const {
		size_t j=0;
		Vector result(size());
		ConstFiltered<Pose2> poses = filter<Pose2>();
		BOOST_FOREACH(const ConstFiltered<Pose2>::KeyValuePair& keyValue, poses)
			result(j++) = keyValue.value.theta	();
		return result;
	}

  /* ************************************************************************* */
  void Graph::addPrior(Key i, const Pose2& p, const SharedNoiseModel& model) {
    sharedFactor factor(new Prior(i, p, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPoseConstraint(Key i, const Pose2& p) {
    sharedFactor factor(new HardConstraint(i, p));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addOdometry(Key i1, Key i2, const Pose2& z,
      const SharedNoiseModel& model) {
    sharedFactor factor(new Odometry(i1, i2, z, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  Values Graph::optimize(const Values& initialEstimate) const {
    return LevenbergMarquardtOptimizer(*this, initialEstimate).optimize();
  }

  /* ************************************************************************* */

} // pose2SLAM
