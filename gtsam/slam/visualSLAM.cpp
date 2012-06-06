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
    boost::shared_ptr<ProjectionFactor> factor(new ProjectionFactor(measured, model, poseKey, pointKey, K));
    push_back(factor);
  }

  /* ************************************************************************* */
  void Graph::addStereoMeasurement(const StereoPoint2& measured, const SharedNoiseModel& model,
       Key poseKey, Key pointKey, const shared_ptrKStereo K) {
    boost::shared_ptr<StereoFactor> factor(new StereoFactor(measured, model, poseKey, pointKey, K));
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
  void Graph::addOdometry(Key x1, Key x2, const Pose3& odometry, const SharedNoiseModel& model) {
    push_back(boost::shared_ptr<BetweenFactor<Pose3> >(new BetweenFactor<Pose3>(x1, x2, odometry, model)));
  }

  /* ************************************************************************* */
}
