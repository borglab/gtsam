/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SphericalCamera.h
 * @brief Calibrated camera with spherical projection
 * @date Aug 26, 2021
 * @author Luca Carlone
 */

#include <gtsam/geometry/SphericalCamera.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Matrix26 SphericalCamera::Dpose(const Point2& pn, double d) {
//  // optimized version of derivatives, see CalibratedCamera.nb
//  const double u = pn.x(), v = pn.y();
//  double uv = u * v, uu = u * u, vv = v * v;
//  Matrix26 Dpn_pose;
//  Dpn_pose << uv, -1 - uu, v, -d, 0, d * u, 1 + vv, -uv, -u, 0, -d, d * v;
//  return Dpn_pose;
}

/* ************************************************************************* */
Matrix23 SphericalCamera::Dpoint(const Point2& pn, double d, const Matrix3& Rt) {
//  // optimized version of derivatives, see CalibratedCamera.nb
//  const double u = pn.x(), v = pn.y();
//  Matrix23 Dpn_point;
//  Dpn_point << //
//      Rt(0, 0) - u * Rt(2, 0), Rt(0, 1) - u * Rt(2, 1), Rt(0, 2) - u * Rt(2, 2), //
//  /**/Rt(1, 0) - v * Rt(2, 0), Rt(1, 1) - v * Rt(2, 1), Rt(1, 2) - v * Rt(2, 2);
//  Dpn_point *= d;
//  return Dpn_point;
}

/* ************************************************************************* */
bool SphericalCamera::equals(const SphericalCamera &camera, double tol) const {
  return pose_.equals(camera.pose(), tol);
}

/* ************************************************************************* */
void SphericalCamera::print(const string& s) const {
  pose_.print(s + ".pose");
}

/* ************************************************************************* */
pair<Unit3, bool> SphericalCamera::projectSafe(const Point3& pw) const {
  const Point3 pc = pose().transformTo(pw);
  Unit3::FromPoint3(pc);
  return make_pair(pn, pc.norm() > 1e-8);
}

/* ************************************************************************* */
Unit3 SphericalCamera::project2(const Point3& pw, OptionalJacobian<2, 6> Dpose,
    OptionalJacobian<2, 3> Dpoint) const {

  Matrix3 Dtf_pose, Dtf_point; // calculated by transformTo if needed
  const Point3 pc = pose().transformTo(pw, Dpoint ? &Dtf_pose : 0, Dpoint ? &Dtf_point : 0);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  if (pc.norm() <= 1e-8)
    throw CheiralityException();
#endif
  Matrix Dunit; // calculated by FromPoint3 if needed
  Unit3 pn = Unit3::FromPoint3(Point3(pc), Dpoint ? &Dunit : 0);

  if (Dpose)
    *Dpose = Dunit * Dtf_pose; //2x3 * 3x6 = 2x6
  if (Dpoint)
    *Dpoint = Dunit * Dtf_point; //2x3 * 3x3 = 2x3
  return pn;
}

/* ************************************************************************* */
Unit3 SphericalCamera::project2(const Unit3& pw, OptionalJacobian<2, 6> Dpose,
    OptionalJacobian<2, 2> Dpoint) const {
  return project2(Point3(pw), Dpose, Dpoint);
}
/* ************************************************************************* */
Point3 SphericalCamera::BackprojectFromCamera(const Unit3& pu, const double depth) {
  return depth * pu;
}

/* ************************************************************************* */
Unit3 SphericalCamera::project(const Point3& point,
    OptionalJacobian<2, 6> Dcamera, OptionalJacobian<2, 3> Dpoint) const {
  return project2(point, Dcamera, Dpoint);
}

/* ************************************************************************* */
}
