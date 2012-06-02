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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Use planarSLAM namespace for specific SLAM instance
namespace planarSLAM {

  /* ************************************************************************* */
  Graph::Graph(const NonlinearFactorGraph& graph) :
      NonlinearFactorGraph(graph) {}

  /* ************************************************************************* */
  void Graph::addPrior(Key i, const Pose2& p, const SharedNoiseModel& model) {
    sharedFactor factor(new Prior(i, p, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPoseConstraint(Key i, const Pose2& p) {
    sharedFactor factor(new Constraint(i, p));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addOdometry(Key i1, Key i2, const Pose2& odometry, const SharedNoiseModel& model) {
    sharedFactor factor(new Odometry(i1, i2, odometry, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addBearing(Key i, Key j, const Rot2& z, const SharedNoiseModel& model) {
    sharedFactor factor(new Bearing(i, j, z, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addRange(Key i, Key j, double z, const SharedNoiseModel& model) {
    sharedFactor factor(new Range(i, j, z, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addBearingRange(Key i, Key j, const Rot2& z1,
      double z2, const SharedNoiseModel& model) {
    sharedFactor factor(new BearingRange(i, j, z1, z2, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  Values Graph::optimize(const Values& initialEstimate) const {
    return LevenbergMarquardtOptimizer(*this, initialEstimate).optimize();
  }

  /* ************************************************************************* */

} // planarSLAM

