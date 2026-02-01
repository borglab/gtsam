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

using namespace std;

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
Matrix23 PinholeBase::Dpoint(const Point2& pn, double d, const Matrix3& Rt) {
  // optimized version of derivatives, see CalibratedCamera.nb
  const double u = pn.x(), v = pn.y();
  Matrix23 Dpn_point;
  Dpn_point << //
      Rt(0, 0) - u * Rt(2, 0), Rt(0, 1) - u * Rt(2, 1), Rt(0, 2) - u * Rt(2, 2), //
  /**/Rt(1, 0) - v * Rt(2, 0), Rt(1, 1) - v * Rt(2, 1), Rt(1, 2) - v * Rt(2, 2);
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
void PinholeBase::print(const string& s) const {
  pose_.print(s + ".pose");
}

/* ************************************************************************* */
const Pose3& PinholeBase::getPose(OptionalJacobian<6, 6> H) const {
  if (H) {
    H->setZero();
    H->block(0, 0, 6, 6) = I_6x6;
  }
  return pose_;
}

/* ************************************************************************* */
Point2 PinholeBase::Project(const Point3& pc, OptionalJacobian<2, 3> Dpoint) {
  double d = 1.0 / pc.z();
  const double u = pc.x() * d, v = pc.y() * d;
  if (Dpoint)
    *Dpoint << d, 0.0, -u * d, 0.0, d, -v * d;
  return Point2(u, v);
}

/* ************************************************************************* */
Point2 PinholeBase::Project(const Unit3& pc, OptionalJacobian<2, 2> Dpoint) {
  if (Dpoint) {
    Matrix32 Dpoint3_pc;
    Matrix23 Duv_point3;
    Point2 uv = Project(pc.point3(Dpoint3_pc), Duv_point3);
    *Dpoint = Duv_point3 * Dpoint3_pc;
    return uv;
  } else
    return Project(pc.point3());
}

/* ************************************************************************* */
pair<Point2, bool> PinholeBase::projectSafe(const Point3& pw) const {
  const Point3 pc = pose().transformTo(pw);
  const Point2 pn = Project(pc);
  return make_pair(pn, pc.z() > 0);
}

/* ************************************************************************* */
Point2 PinholeBase::project2(const Point3& point, OptionalJacobian<2, 6> Dpose,
    OptionalJacobian<2, 3> Dpoint) const {

  Matrix3 Rt; // calculated by transformTo if needed
  const Point3 q = pose().transformTo(point, {}, Dpoint ? &Rt : 0);
#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  if (q.z() <= 0)
    throw CheiralityException();
#endif
  const Point2 pn = Project(q);

  if (Dpose || Dpoint) {
    const double d = 1.0 / q.z();
    if (Dpose)
      *Dpose = PinholeBase::Dpose(pn, d);
    if (Dpoint)
      *Dpoint = PinholeBase::Dpoint(pn, d, Rt);
  }
  return pn;
}

/* ************************************************************************* */
Point2 PinholeBase::project2(const Unit3& pw, OptionalJacobian<2, 6> Dpose,
    OptionalJacobian<2, 2> Dpoint) const {

  // world to camera coordinate
  Matrix23 Dpc_rot;
  Matrix2 Dpc_point;
  const Unit3 pc = pose().rotation().unrotate(pw, Dpose ? &Dpc_rot : 0,
      Dpoint ? &Dpc_point : 0);
#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  if (pc.unitVector().z() <= 0)
    throw CheiralityException();
#endif
  // camera to normalized image coordinate
  Matrix2 Dpn_pc;
  const Point2 pn = Project(pc, Dpose || Dpoint ? &Dpn_pc : 0);

  // chain the Jacobian matrices
  if (Dpose) {
    // only rotation is important
    Matrix26 Dpc_pose;
    Dpc_pose.setZero();
    Dpc_pose.leftCols<3>() = Dpc_rot;
    *Dpose = Dpn_pc * Dpc_pose; // 2x2 * 2x6
  }
  if (Dpoint)
    *Dpoint = Dpn_pc * Dpc_point; // 2x2 * 2*2
  return pn;
}
/* ************************************************************************* */
Point3 PinholeBase::BackprojectFromCamera(const Point2& p,
    const double depth, OptionalJacobian<3, 2> Dpoint, OptionalJacobian<3, 1> Ddepth) {
  if (Dpoint)
    *Dpoint << depth, 0, 0, depth, 0, 0;
  if (Ddepth)
    *Ddepth << p.x(), p.y(), 1;
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
