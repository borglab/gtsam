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
#include <gtsam/slam/BetweenFactor.h>
#include <boost/make_shared.hpp>

using boost::make_shared;

namespace visualSLAM {

  /* ************************************************************************* */
  Vector Values::xs() const {
    size_t j=0;
    ConstFiltered<Pose3> poses = filter<Pose3>();
    Vector result(poses.size());
    BOOST_FOREACH(const ConstFiltered<Pose3>::KeyValuePair& keyValue, poses)
      result(j++) = keyValue.value.x();
    return result;
  }

  /* ************************************************************************* */
  Vector Values::ys() const {
    size_t j=0;
    ConstFiltered<Pose3> poses = filter<Pose3>();
    Vector result(poses.size());
    BOOST_FOREACH(const ConstFiltered<Pose3>::KeyValuePair& keyValue, poses)
      result(j++) = keyValue.value.y();
    return result;
  }

  /* ************************************************************************* */
  Vector Values::zs() const {
    size_t j=0;
    ConstFiltered<Pose3> poses = filter<Pose3>();
    Vector result(poses.size());
    BOOST_FOREACH(const ConstFiltered<Pose3>::KeyValuePair& keyValue, poses)
      result(j++) = keyValue.value.z();
    return result;
  }

  Matrix Values::points() const {
    size_t j=0;
    ConstFiltered<Point3> points = filter<Point3>();
    Matrix result(points.size(),3);
    BOOST_FOREACH(const ConstFiltered<Point3>::KeyValuePair& keyValue, points)
      result.row(j++) = keyValue.value.vector();
    return result;
  }

  /* ************************************************************************* */
  void Graph::addMeasurement(const Point2& measured, const SharedNoiseModel& model,
       Key poseKey, Key pointKey, const shared_ptrK K) {
    push_back(make_shared<ProjectionFactor>(measured, model, poseKey, pointKey, K));
  }

  /* ************************************************************************* */
  void Graph::addStereoMeasurement(const StereoPoint2& measured, const SharedNoiseModel& model,
       Key poseKey, Key pointKey, const shared_ptrKStereo K) {
  	push_back(make_shared<StereoFactor>(measured, model, poseKey, pointKey, K));
  }

  /* ************************************************************************* */
  void Graph::addPoseConstraint(Key poseKey, const Pose3& p) {
  	push_back(make_shared<PoseConstraint>(poseKey, p));
  }

  /* ************************************************************************* */
  void Graph::addPointConstraint(Key pointKey, const Point3& p) {
  	push_back(make_shared<PointConstraint>(pointKey, p));
  }

  /* ************************************************************************* */
  void Graph::addPosePrior(Key poseKey, const Pose3& p, const SharedNoiseModel& model) {
  	push_back(make_shared<PosePrior>(poseKey, p, model));
  }

  /* ************************************************************************* */
  void Graph::addPointPrior(Key pointKey, const Point3& p, const SharedNoiseModel& model) {
  	push_back(make_shared<PointPrior>(pointKey, p, model));
  }

  /* ************************************************************************* */
  void Graph::addRangeFactor(Key poseKey, Key pointKey, double range, const SharedNoiseModel& model) {
    push_back(make_shared<RangeFactor>(poseKey, pointKey, range, model));
  }

  /* ************************************************************************* */
  void Graph::addOdometry(Key x1, Key x2, const Pose3& odometry, const SharedNoiseModel& model) {
    push_back(make_shared<BetweenFactor<Pose3> >(x1, x2, odometry, model));
  }

  /* ************************************************************************* */
}
