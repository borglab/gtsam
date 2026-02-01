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
bool SphericalCamera::equals(const SphericalCamera& camera, double tol) const {
  return pose_.equals(camera.pose(), tol);
}

/* ************************************************************************* */
void SphericalCamera::print(const string& s) const { pose_.print(s + ".pose"); }

/* ************************************************************************* */
pair<Unit3, bool> SphericalCamera::projectSafe(const Point3& pw) const {
  const Point3 pc = pose().transformTo(pw);
  Unit3 pu = Unit3::FromPoint3(pc);
  return make_pair(pu, pc.norm() > 1e-8);
}

/* ************************************************************************* */
Unit3 SphericalCamera::project2(const Point3& pw, OptionalJacobian<2, 6> Dpose,
                                OptionalJacobian<2, 3> Dpoint) const {
  Matrix36 Dtf_pose;
  Matrix3 Dtf_point;  // calculated by transformTo if needed
  const Point3 pc =
      pose().transformTo(pw, Dpose ? &Dtf_pose : 0, Dpoint ? &Dtf_point : 0);

  if (pc.norm() <= 1e-8) throw("point cannot be at the center of the camera");

  Matrix23 Dunit;  // calculated by FromPoint3 if needed
  Unit3 pu = Unit3::FromPoint3(Point3(pc), Dpoint ? &Dunit : 0);

  if (Dpose) *Dpose = Dunit * Dtf_pose;     // 2x3 * 3x6 = 2x6
  if (Dpoint) *Dpoint = Dunit * Dtf_point;  // 2x3 * 3x3 = 2x3
  return pu;
}

/* ************************************************************************* */
Unit3 SphericalCamera::project2(const Unit3& pwu, OptionalJacobian<2, 6> Dpose,
                                OptionalJacobian<2, 2> Dpoint) const {
  Matrix23 Dtf_rot;
  Matrix2 Dtf_point;  // calculated by transformTo if needed
  const Unit3 pu = pose().rotation().unrotate(pwu, Dpose ? &Dtf_rot : 0,
                                              Dpoint ? &Dtf_point : 0);

  if (Dpose)
    *Dpose << Dtf_rot, Matrix::Zero(2, 3);  // 2x6 (translation part is zero)
  if (Dpoint) *Dpoint = Dtf_point;          // 2x2
  return pu;
}

/* ************************************************************************* */
Point3 SphericalCamera::backproject(const Unit3& pu, const double depth) const {
  return pose().transformFrom(depth * pu);
}

/* ************************************************************************* */
Unit3 SphericalCamera::backprojectPointAtInfinity(const Unit3& p) const {
  return pose().rotation().rotate(p);
}

/* ************************************************************************* */
Unit3 SphericalCamera::project(const Point3& point,
                               OptionalJacobian<2, 6> Dcamera,
                               OptionalJacobian<2, 3> Dpoint) const {
  return project2(point, Dcamera, Dpoint);
}

/* ************************************************************************* */
Vector2 SphericalCamera::reprojectionError(
    const Point3& point, const Unit3& measured, OptionalJacobian<2, 6> Dpose,
    OptionalJacobian<2, 3> Dpoint) const {
  // project point
  if (Dpose || Dpoint) {
    Matrix26 H_project_pose;
    Matrix23 H_project_point;
    Matrix22 H_error;
    Unit3 projected = project2(point, H_project_pose, H_project_point);
    Vector2 error = measured.errorVector(projected, {}, H_error);
    if (Dpose) *Dpose = H_error * H_project_pose;
    if (Dpoint) *Dpoint = H_error * H_project_point;
    return error;
  } else {
    return measured.errorVector(project2(point, Dpose, Dpoint));
  }
}

/* ************************************************************************* */
}  // namespace gtsam
