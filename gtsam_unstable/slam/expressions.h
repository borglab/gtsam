/**
 * @file expressions.h
 * @brief Common expressions for solving geometry/slam/sfm problems
 * @date Oct 1, 2014
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam_unstable/nonlinear/Expression.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace gtsam {

typedef Expression<Point2> Point2_;
typedef Expression<Point3> Point3_;
typedef Expression<Rot3> Rot3_;
typedef Expression<Pose3> Pose3_;
typedef Expression<Cal3_S2> Cal3_S2_;

Point3_ transform_to(const Pose3_& x, const Point3_& p) {
  return Point3_(x, &Pose3::transform_to, p);
}

Point2_ project(const Point3_& p_cam) {
  return Point2_(PinholeCamera<Cal3_S2>::project_to_camera, p_cam);
}

template<class CAL>
Point2_ uncalibrate(const Expression<CAL>& K, const Point2_& xy_hat) {
  return Point2_(K, &CAL::uncalibrate, xy_hat);
}

} // \namespace gtsam

