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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimization.h>

// Use planarSLAM namespace for specific SLAM instance
namespace planarSLAM {

  Graph::Graph(const NonlinearFactorGraph& graph) :
      NonlinearFactorGraph(graph) {}

  void Graph::addPrior(Index i, const Pose2& p, const SharedNoiseModel& model) {
    sharedFactor factor(new Prior(PoseKey(i), p, model));
    push_back(factor);
  }

  void Graph::addPoseConstraint(Index i, const Pose2& p) {
    sharedFactor factor(new Constraint(PoseKey(i), p));
    push_back(factor);
  }

  void Graph::addOdometry(Index i, Index j, const Pose2& odometry, const SharedNoiseModel& model) {
    sharedFactor factor(new Odometry(PoseKey(i), PoseKey(j), odometry, model));
    push_back(factor);
  }

  void Graph::addBearing(Index i, Index j, const Rot2& z, const SharedNoiseModel& model) {
    sharedFactor factor(new Bearing(PoseKey(i), PointKey(j), z, model));
    push_back(factor);
  }

  void Graph::addRange(Index i, Index j, double z, const SharedNoiseModel& model) {
    sharedFactor factor(new Range(PoseKey(i), PointKey(j), z, model));
    push_back(factor);
  }

  void Graph::addBearingRange(Index i, Index j, const Rot2& z1,
      double z2, const SharedNoiseModel& model) {
    sharedFactor factor(new BearingRange(PoseKey(i), PointKey(j), z1, z2, model));
    push_back(factor);
  }

} // planarSLAM

