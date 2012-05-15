/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file visualSLAM.cpp
 * @date Jan 14, 2010
 * @author richard
 */

#include <gtsam/slam/visualSLAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace visualSLAM {

  /* ************************************************************************* */
  void Graph::addMeasurement(const Point2& measured, const SharedNoiseModel& model,
      Index poseKey, Index pointKey, const shared_ptrK& K) {
    boost::shared_ptr<ProjectionFactor> factor(new ProjectionFactor(measured, model, PoseKey(poseKey), PointKey(pointKey), K));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPoseConstraint(Index poseKey, const Pose3& p) {
    boost::shared_ptr<PoseConstraint> factor(new PoseConstraint(PoseKey(poseKey), p));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPointConstraint(Index pointKey, const Point3& p) {
    boost::shared_ptr<PointConstraint> factor(new PointConstraint(PointKey(pointKey), p));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPosePrior(Index poseKey, const Pose3& p, const SharedNoiseModel& model) {
    boost::shared_ptr<PosePrior> factor(new PosePrior(PoseKey(poseKey), p, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPointPrior(Index pointKey, const Point3& p, const SharedNoiseModel& model) {
    boost::shared_ptr<PointPrior> factor(new PointPrior(PointKey(pointKey), p, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addRangeFactor(Index poseKey, Index pointKey, double range, const SharedNoiseModel& model) {
    push_back(boost::shared_ptr<RangeFactor>(new RangeFactor(PoseKey(poseKey), PointKey(pointKey), range, model)));
  }

  /* ************************************************************************* */

}
