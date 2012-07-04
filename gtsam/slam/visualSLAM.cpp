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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <boost/make_shared.hpp>

using boost::make_shared;

namespace visualSLAM {

  /* ************************************************************************* */
  Matrix Values::points() const {
    size_t j=0;
    ConstFiltered<Point3> points = allPoints();
    Matrix result(points.size(),3);
    BOOST_FOREACH(const ConstFiltered<Point3>::KeyValuePair& keyValue, points)
      result.row(j++) = keyValue.value.vector();
    return result;
  }

  /* ************************************************************************* */
  void Graph::addPointConstraint(Key pointKey, const Point3& p) {
  	push_back(make_shared<NonlinearEquality<Point3> >(pointKey, p));
  }

  /* ************************************************************************* */
  void Graph::addPointPrior(Key pointKey, const Point3& p, const SharedNoiseModel& model) {
  	push_back(make_shared<PriorFactor<Point3> >(pointKey, p, model));
  }

  /* ************************************************************************* */
  void Graph::addMeasurement(const Point2& measured, const SharedNoiseModel& model,
       Key poseKey, Key pointKey, const shared_ptrK K) {
    push_back(
        make_shared<GenericProjectionFactor<Pose3, Point3> >
          (measured, model, poseKey, pointKey, K));
  }

  /* ************************************************************************* */
  void Graph::addMeasurements(Key i, const KeyVector& J, const Matrix& Z,
      const SharedNoiseModel& model, const shared_ptrK K) {
    if ( Z.rows()!=2) throw std::invalid_argument("addMeasurements: Z must be 2*K");
    if (Z.cols() != J.size())
      throw std::invalid_argument(
          "addMeasurements: J and Z must have same number of entries");
    for(size_t k=0;k<Z.cols();k++)
      addMeasurement(Point2(Z(0,k),Z(1,k)),model,i, J[k], K);
  }

  /* ************************************************************************* */
  void Graph::addStereoMeasurement(const StereoPoint2& measured, const SharedNoiseModel& model,
       Key poseKey, Key pointKey, const shared_ptrKStereo K) {
  	push_back(make_shared<GenericStereoFactor<Pose3, Point3> >(measured, model, poseKey, pointKey, K));
  }

  /* ************************************************************************* */
  void Graph::addRangeFactor(Key poseKey, Key pointKey, double range, const SharedNoiseModel& model) {
    push_back(make_shared<gtsam::RangeFactor<Pose3, Point3> >(poseKey, pointKey, range, model));
  }

  /* ************************************************************************* */
}
