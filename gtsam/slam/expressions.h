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
#include <gtsam/geometry/Line3.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace gtsam {

// 2D Geometry

typedef Expression<Point2> Point2_;
typedef Expression<Rot2> Rot2_;
typedef Expression<Pose2> Pose2_;

inline Point2_ transformTo(const Pose2_& x, const Point2_& p) {
  return Point2_(x, &Pose2::transformTo, p);
}

inline Double_ range(const Point2_& p, const Point2_& q) {
  return Double_(Range<Point2, Point2>(), p, q);
}

// 3D Geometry

typedef Expression<Point3> Point3_;
typedef Expression<Unit3> Unit3_;
typedef Expression<Rot3> Rot3_;
typedef Expression<Pose3> Pose3_;
typedef Expression<Line3> Line3_;
typedef Expression<OrientedPlane3> OrientedPlane3_;

inline Point3_ transformTo(const Pose3_& x, const Point3_& p) {
  return Point3_(x, &Pose3::transformTo, p);
}

inline Point3_ transformFrom(const Pose3_& x, const Point3_& p) {
  return Point3_(x, &Pose3::transformFrom, p);
}

inline Line3_ transformTo(const Pose3_ &wTc, const Line3_ &wL) {
  Line3 (*f)(const Pose3 &, const Line3 &,
             OptionalJacobian<4, 6>, OptionalJacobian<4, 4>) = &transformTo;
  return Line3_(f, wTc, wL);
}

inline Pose3_ transformPoseTo(const Pose3_& p, const Pose3_& q) {
  return Pose3_(p, &Pose3::transformPoseTo, q);
}

inline Point3_ normalize(const Point3_& a){
  Point3 (*f)(const Point3 &, OptionalJacobian<3, 3>) = &normalize;
  return Point3_(f, a);
}

inline Point3_ cross(const Point3_& a, const Point3_& b) {
  Point3 (*f)(const Point3 &, const Point3 &,
             OptionalJacobian<3, 3>, OptionalJacobian<3, 3>) = &cross;
  return Point3_(f, a, b);
}

inline Double_ dot(const Point3_& a, const Point3_& b) {
  double (*f)(const Point3 &, const Point3 &,
             OptionalJacobian<1, 3>, OptionalJacobian<1, 3>) = &dot;
  return Double_(f, a, b);
}

namespace internal {
// define getter that returns value rather than reference
inline Rot3 rotation(const Pose3& pose, OptionalJacobian<3, 6> H) {
  return pose.rotation(H);
}

inline Point3 translation(const Pose3& pose, OptionalJacobian<3, 6> H) {
  return pose.translation(H);
}
}  // namespace internal

inline Rot3_ rotation(const Pose3_& pose) {
  return Rot3_(internal::rotation, pose);
}

inline Point3_ translation(const Pose3_& pose) {
  return Point3_(internal::translation, pose);
}

inline Point3_ rotate(const Rot3_& x, const Point3_& p) {
  return Point3_(x, &Rot3::rotate, p);
}

inline Point3_ point3(const Unit3_& v) {
  return Point3_(&Unit3::point3, v);
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

inline Double_ distance(const OrientedPlane3_& p) {
  return Double_(&OrientedPlane3::distance, p);
}

inline Unit3_ normal(const OrientedPlane3_& p) {
  return Unit3_(&OrientedPlane3::normal, p);
}

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
inline Point2 project6(const Pose3& x, const POINT& p, const CALIBRATION& K,
                       OptionalJacobian<2, 6> Dpose, OptionalJacobian<2, 3> Dpoint,
                       OptionalJacobian<2, 5> Dcal) {
  return PinholeCamera<CALIBRATION>(x, K).project(p, Dpose, Dpoint, Dcal);
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

template <class CALIBRATION>
inline Pose3_ getPose(const Expression<PinholeCamera<CALIBRATION> > & cam) {
  return Pose3_(&PinholeCamera<CALIBRATION>::getPose, cam);
}


/// logmap
// TODO(dellaert): Should work but fails because of a type deduction conflict.
// template <typename T>
// gtsam::Expression<typename gtsam::traits<T>::TangentVector> logmap(
//     const gtsam::Expression<T> &x1, const gtsam::Expression<T> &x2) {
//   return gtsam::Expression<typename gtsam::traits<T>::TangentVector>(
//       x1, &T::logmap, x2);
// }

template <typename T>
gtsam::Expression<typename gtsam::traits<T>::TangentVector> logmap(
    const gtsam::Expression<T> &x1, const gtsam::Expression<T> &x2) {
  return Expression<typename gtsam::traits<T>::TangentVector>(
      gtsam::traits<T>::Logmap, between(x1, x2));
}

}  // \namespace gtsam
