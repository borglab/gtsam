/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CalibratedCamera.cpp
 * @brief Calibrated camera for which only pose is unknown
 * @date Aug 17, 2009
 * @author Frank Dellaert
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/CalibratedCamera.h>

namespace gtsam {

/* ************************************************************************* */
Matrix26 PinholeBase::Dpose(const Point2& pn, double d) {
  // optimized version of derivatives, see CalibratedCamera.nb
  const double u = pn.x(), v = pn.y();
  double uv = u * v, uu = u * u, vv = v * v;
  Matrix26 Dpn_pose;
  Dpn_pose << uv, -1 - uu, v, -d, 0, d * u, 1 + vv, -uv, -u, 0, -d, d * v;
  return Dpn_pose;
}

/* ************************************************************************* */
Matrix23 PinholeBase::Dpoint(const Point2& pn, double d, const Matrix3& R) {
  // optimized version of derivatives, see CalibratedCamera.nb
  const double u = pn.x(), v = pn.y();
  Matrix23 Dpn_point;
  Dpn_point << //
      R(0, 0) - u * R(0, 2), R(1, 0) - u * R(1, 2), R(2, 0) - u * R(2, 2), //
  /**/R(0, 1) - v * R(0, 2), R(1, 1) - v * R(1, 2), R(2, 1) - v * R(2, 2);
  Dpn_point *= d;
  return Dpn_point;
}

/* ************************************************************************* */
Pose3 PinholeBase::LevelPose(const Pose2& pose2, double height) {
  const double st = sin(pose2.theta()), ct = cos(pose2.theta());
  const Point3 x(st, -ct, 0), y(0, 0, -1), z(ct, st, 0);
  const Rot3 wRc(x, y, z);
  const Point3 t(pose2.x(), pose2.y(), height);
  return Pose3(wRc, t);
}

/* ************************************************************************* */
Pose3 PinholeBase::LookatPose(const Point3& eye, const Point3& target,
    const Point3& upVector) {
  Point3 zc = target - eye;
  zc = zc / zc.norm();
  Point3 xc = (-upVector).cross(zc); // minus upVector since yc is pointing down
  xc = xc / xc.norm();
  Point3 yc = zc.cross(xc);
  return Pose3(Rot3(xc, yc, zc), eye);
}

/* ************************************************************************* */
bool PinholeBase::equals(const PinholeBase &camera, double tol) const {
  return pose_.equals(camera.pose(), tol);
}

/* ************************************************************************* */
void PinholeBase::print(const std::string& s) const {
  pose_.print(s + ".pose");
}

/* ************************************************************************* */
const Pose3& PinholeBase::pose(OptionalJacobian<6, 6> H) const {
  if (H) {
    H->setZero();
    H->block(0, 0, 6, 6) = I_6x6;
  }
  return pose_;
}

/* ************************************************************************* */
Point2 PinholeBase::project_to_camera(const Point3& P,
    OptionalJacobian<2, 3> Dpoint) {
#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  if (P.z() <= 0)
  throw CheiralityException();
#endif
  double d = 1.0 / P.z();
  const double u = P.x() * d, v = P.y() * d;
  if (Dpoint)
    *Dpoint << d, 0.0, -u * d, 0.0, d, -v * d;
  return Point2(u, v);
}

/* ************************************************************************* */
Point2 PinholeBase::project2(const Point3& point, OptionalJacobian<2, 6> Dpose,
    OptionalJacobian<2, 3> Dpoint) const {

  Matrix3 Rt; // calculated by transform_to if needed
  const Point3 q = pose().transform_to(point, boost::none, Dpoint ? &Rt : 0);
  Point2 pn = project_to_camera(q);

  if (Dpose || Dpoint) {
    const double z = q.z(), d = 1.0 / z;
    if (Dpose)
      *Dpose = PinholeBase::Dpose(pn, d);
    if (Dpoint)
      *Dpoint = PinholeBase::Dpoint(pn, d, Rt.transpose()); // TODO transpose
  }
  return pn;
}

/* ************************************************************************* */
Point3 PinholeBase::backproject_from_camera(const Point2& p,
    const double depth) {
  return Point3(p.x() * depth, p.y() * depth, depth);
}

/* ************************************************************************* */
CalibratedCamera CalibratedCamera::Level(const Pose2& pose2, double height) {
  return CalibratedCamera(LevelPose(pose2, height));
}

/* ************************************************************************* */
CalibratedCamera CalibratedCamera::Lookat(const Point3& eye,
    const Point3& target, const Point3& upVector) {
  return CalibratedCamera(LookatPose(eye, target, upVector));
}

/* ************************************************************************* */
Point2 CalibratedCamera::project(const Point3& point,
    OptionalJacobian<2, 6> Dcamera, OptionalJacobian<2, 3> Dpoint) const {
  return project2(point, Dcamera, Dpoint);
}

/* ************************************************************************* */
CalibratedCamera CalibratedCamera::retract(const Vector& d) const {
  return CalibratedCamera(pose().retract(d));
}

/* ************************************************************************* */
Vector CalibratedCamera::localCoordinates(const CalibratedCamera& T2) const {
  return pose().localCoordinates(T2.pose());
}

/* ************************************************************************* */
}
