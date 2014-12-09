/**
 * @file expressions.h
 * @brief Common expressions for solving geometry/slam/sfm problems
 * @date Oct 1, 2014
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam_unstable/nonlinear/expressions.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace gtsam {

// 2D Geometry

typedef Expression<Point2> Point2_;
typedef Expression<Rot2> Rot2_;
typedef Expression<Pose2> Pose2_;

Point2_ transform_to(const Pose2_& x, const Point2_& p) {
  return Point2_(x, &Pose2::transform_to, p);
}

// 3D Geometry

typedef Expression<Point3> Point3_;
typedef Expression<Rot3> Rot3_;
typedef Expression<Pose3> Pose3_;

Point3_ transform_to(const Pose3_& x, const Point3_& p) {
  return Point3_(x, &Pose3::transform_to, p);
}

// Projection

typedef Expression<Cal3_S2> Cal3_S2_;

Point2_ project(const Point3_& p_cam) {
  return Point2_(PinholeCamera<Cal3_S2>::project_to_camera, p_cam);
}

Point2 project6(const Pose3& x, const Point3& p, const Cal3_S2& K,
    OptionalJacobian<2, 6> Dpose, OptionalJacobian<2, 3> Dpoint, OptionalJacobian<2, 5> Dcal) {
  return PinholeCamera<Cal3_S2>(x, K).project(p, Dpose, Dpoint, Dcal);
}

Point2_ project3(const Pose3_& x, const Point3_& p, const Cal3_S2_& K) {
  return Point2_(project6, x, p, K);
}

template<class CAL>
Point2_ uncalibrate(const Expression<CAL>& K, const Point2_& xy_hat) {
  return Point2_(K, &CAL::uncalibrate, xy_hat);
}

} // \namespace gtsam

