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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

// Use pose2SLAM namespace for specific SLAM instance

namespace pose2SLAM {

  /* ************************************************************************* */
  Values circle(size_t n, double R) {
    Values x;
    double theta = 0, dtheta = 2 * M_PI / n;
    for (size_t i = 0; i < n; i++, theta += dtheta)
      x.insert(PoseKey(i), Pose2(cos(theta), sin(theta), M_PI_2 + theta));
    return x;
  }

  /* ************************************************************************* */
  void Graph::addPrior(Index i, const Pose2& p,
      const SharedNoiseModel& model) {
    sharedFactor factor(new Prior(PoseKey(i), p, model));
    push_back(factor);
  }

  void Graph::addPoseConstraint(Index i, const Pose2& p) {
    sharedFactor factor(new HardConstraint(PoseKey(i), p));
    push_back(factor);
  }

  void Graph::addOdometry(Index i, Index j, const Pose2& z,
      const SharedNoiseModel& model) {
    sharedFactor factor(new Odometry(PoseKey(i), PoseKey(j), z, model));
    push_back(factor);
  }

/* ************************************************************************* */

} // pose2SLAM
