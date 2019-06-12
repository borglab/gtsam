/**
 * @file expressions.h
 * @brief Common expressions for solving geometry/slam/sfm problems
 * @date Oct 1, 2014
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/nonlinear/expressions.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace gtsam {

// 2D Geometry

typedef Expression<Point2> Point2_;
typedef Expression<Rot2> Rot2_;
typedef Expression<Pose2> Pose2_;

inline Point2_ transformTo(const Pose2_& x, const Point2_& p) {
  return Point2_(x, &Pose2::transformTo, p);
}

// 3D Geometry

typedef Expression<Point3> Point3_;
typedef Expression<Unit3> Unit3_;
typedef Expression<Rot3> Rot3_;
typedef Expression<Pose3> Pose3_;

inline Point3_ transformTo(const Pose3_& x, const Point3_& p) {
  return Point3_(x, &Pose3::transformTo, p);
}

inline Point3_ transformFrom(const Pose3_& x, const Point3_& p) {
  return Point3_(x, &Pose3::transformFrom, p);
}

namespace internal {
// define getter that returns value rather than reference
inline Rot3 rotation(const Pose3& pose, OptionalJacobian<3, 6> H) {
  return pose.rotation(H);
}
}  // namespace internal

inline Rot3_ rotation(const Pose3_& pose) {
  return Rot3_(internal::rotation, pose);
}

inline Point3_ rotate(const Rot3_& x, const Point3_& p) {
  return Point3_(x, &Rot3::rotate, p);
}

inline Unit3_ rotate(const Rot3_& x, const Unit3_& p) {
  return Unit3_(x, &Rot3::rotate, p);
}

inline Point3_ unrotate(const Rot3_& x, const Point3_& p) {
  return Point3_(x, &Rot3::unrotate, p);
}

inline Unit3_ unrotate(const Rot3_& x, const Unit3_& p) {
  return Unit3_(x, &Rot3::unrotate, p);
}

#ifndef GTSAM_TYPEDEF_POINTS_TO_VECTORS
namespace internal {
// define a rotate and unrotate for Vector3
inline Vector3 rotate(const Rot3& R, const Vector3& v,
               OptionalJacobian<3, 3> H1 = boost::none,
               OptionalJacobian<3, 3> H2 = boost::none) {
  return R.rotate(v, H1, H2);
}
inline Vector3 unrotate(const Rot3& R, const Vector3& v,
                 OptionalJacobian<3, 3> H1 = boost::none,
                 OptionalJacobian<3, 3> H2 = boost::none) {
  return R.unrotate(v, H1, H2);
}
}  // namespace internal
inline Expression<Vector3> rotate(const Rot3_& R,
                                  const Expression<Vector3>& v) {
  return Expression<Vector3>(internal::rotate, R, v);
}
inline Expression<Vector3> unrotate(const Rot3_& R,
                                    const Expression<Vector3>& v) {
  return Expression<Vector3>(internal::unrotate, R, v);
}
#endif

// Projection

typedef Expression<Cal3_S2> Cal3_S2_;
typedef Expression<Cal3Bundler> Cal3Bundler_;

/// Expression version of PinholeBase::Project
inline Point2_ project(const Point3_& p_cam) {
  Point2 (*f)(const Point3&, OptionalJacobian<2, 3>) = &PinholeBase::Project;
  return Point2_(f, p_cam);
}

inline Point2_ project(const Unit3_& p_cam) {
  Point2 (*f)(const Unit3&, OptionalJacobian<2, 2>) = &PinholeBase::Project;
  return Point2_(f, p_cam);
}

namespace internal {
// Helper template for project2 expression below
template <class CAMERA, class POINT>
Point2 project4(const CAMERA& camera, const POINT& p, OptionalJacobian<2, CAMERA::dimension> Dcam,
                OptionalJacobian<2, FixedDimension<POINT>::value> Dpoint) {
  return camera.project2(p, Dcam, Dpoint);
}
}

template <class CAMERA, class POINT>
Point2_ project2(const Expression<CAMERA>& camera_, const Expression<POINT>& p_) {
  return Point2_(internal::project4<CAMERA, POINT>, camera_, p_);
}

namespace internal {
// Helper template for project3 expression below
template <class CALIBRATION, class POINT>
inline Point2 project6(const Pose3& x, const Point3& p, const Cal3_S2& K,
                       OptionalJacobian<2, 6> Dpose, OptionalJacobian<2, 3> Dpoint,
                       OptionalJacobian<2, 5> Dcal) {
  return PinholeCamera<Cal3_S2>(x, K).project(p, Dpose, Dpoint, Dcal);
}
}

template <class CALIBRATION, class POINT>
inline Point2_ project3(const Pose3_& x, const Expression<POINT>& p,
                        const Expression<CALIBRATION>& K) {
  return Point2_(internal::project6<CALIBRATION, POINT>, x, p, K);
}

template <class CALIBRATION>
Point2_ uncalibrate(const Expression<CALIBRATION>& K, const Point2_& xy_hat) {
  return Point2_(K, &CALIBRATION::uncalibrate, xy_hat);
}

}  // \namespace gtsam
