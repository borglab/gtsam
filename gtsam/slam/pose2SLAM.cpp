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

#include <gtsam/linear/SimpleSPCGSolver.h>
#include <gtsam/slam/pose2SLAM.h>

// Use pose2SLAM namespace for specific SLAM instance

namespace pose2SLAM {

  /* ************************************************************************* */
  Values Values::Circle(size_t n, double R) {
    Values x;
    double theta = 0, dtheta = 2 * M_PI / n;
    for (size_t i = 0; i < n; i++, theta += dtheta)
      x.insert(i, Pose2(cos(theta), sin(theta), M_PI_2 + theta));
    return x;
  }

  /* ************************************************************************* */
	Matrix Values::poses() const {
		size_t j=0;
		ConstFiltered<Pose2> poses = filter<Pose2>();
    Matrix result(poses.size(),3);
		BOOST_FOREACH(const ConstFiltered<Pose2>::KeyValuePair& keyValue, poses) {
			const Pose2& r = keyValue.value;
			result.row(j++) = Matrix_(1,3, r.x(), r.y(), r.theta());
		}
		return result;
	}

  /* ************************************************************************* */
  void Graph::addPoseConstraint(Key i, const Pose2& p) {
    sharedFactor factor(new NonlinearEquality<Pose2>(i, p));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPosePrior(Key i, const Pose2& p, const SharedNoiseModel& model) {
    sharedFactor factor(new PriorFactor<Pose2>(i, p, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addRelativePose(Key i1, Key i2, const Pose2& z,
      const SharedNoiseModel& model) {
    sharedFactor factor(new BetweenFactor<Pose2>(i1, i2, z, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  Values Graph::optimize(const Values& initialEstimate, size_t verbosity) const {
    LevenbergMarquardtParams params;
    params.verbosity = (NonlinearOptimizerParams::Verbosity)verbosity;
    LevenbergMarquardtOptimizer optimizer(*this, initialEstimate,params);
    return optimizer.optimize();
  }

  /* ************************************************************************* */
  Values Graph::optimizeSPCG(const Values& initialEstimate, size_t verbosity) const {
    LevenbergMarquardtParams params;
    params.verbosity = (NonlinearOptimizerParams::Verbosity)verbosity;
    params.linearSolverType = SuccessiveLinearizationParams::CG;
    params.iterativeParams = boost::make_shared<SimpleSPCGSolverParameters>();
    return LevenbergMarquardtOptimizer(*this, initialEstimate, params).optimize();
  }

  /* ************************************************************************* */

} // pose2SLAM
