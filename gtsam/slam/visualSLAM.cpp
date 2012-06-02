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
       Key poseKey, Key pointKey, const shared_ptrK& K) {
    boost::shared_ptr<ProjectionFactor> factor(new ProjectionFactor(measured, model, poseKey, pointKey, K));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPoseConstraint(Key poseKey, const Pose3& p) {
    boost::shared_ptr<PoseConstraint> factor(new PoseConstraint(poseKey, p));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPointConstraint(Key pointKey, const Point3& p) {
    boost::shared_ptr<PointConstraint> factor(new PointConstraint(pointKey, p));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPosePrior(Key poseKey, const Pose3& p, const SharedNoiseModel& model) {
    boost::shared_ptr<PosePrior> factor(new PosePrior(poseKey, p, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addPointPrior(Key pointKey, const Point3& p, const SharedNoiseModel& model) {
    boost::shared_ptr<PointPrior> factor(new PointPrior(pointKey, p, model));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addRangeFactor(Key poseKey, Key pointKey, double range, const SharedNoiseModel& model) {
    push_back(boost::shared_ptr<RangeFactor>(new RangeFactor(poseKey, pointKey, range, model)));
  }

  /* ************************************************************************* */

}
