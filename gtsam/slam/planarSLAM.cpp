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
  Matrix Values::points() const {
    size_t j=0;
    ConstFiltered<Point2> points = filter<Point2>();
    Matrix result(points.size(),2);
    BOOST_FOREACH(const ConstFiltered<Point2>::KeyValuePair& keyValue, points)
      result.row(j++) = keyValue.value.vector();
    return result;
  }

  /* ************************************************************************* */
  void Graph::addPointConstraint(Key pointKey, const Point2& p) {
  	push_back(boost::make_shared<NonlinearEquality<Point2> >(pointKey, p));
  }

  /* ************************************************************************* */
  void Graph::addPointPrior(Key pointKey, const Point2& p, const SharedNoiseModel& model) {
  	push_back(boost::make_shared<PriorFactor<Point2> >(pointKey, p, model));
  }

  /* ************************************************************************* */
  void Graph::addBearing(Key i, Key j, const Rot2& z, const SharedNoiseModel& model) {
    push_back(boost::make_shared<BearingFactor<Pose2, Point2> >(i, j, z, model));
  }

  /* ************************************************************************* */
  void Graph::addRange(Key i, Key j, double z, const SharedNoiseModel& model) {
    push_back(boost::make_shared<RangeFactor<Pose2, Point2> >(i, j, z, model));
  }

  /* ************************************************************************* */
  void Graph::addBearingRange(Key i, Key j, const Rot2& z1,
      double z2, const SharedNoiseModel& model) {
    push_back(boost::make_shared<BearingRangeFactor<Pose2, Point2> >(i, j, z1, z2, model));
  }

  /* ************************************************************************* */

} // planarSLAM

