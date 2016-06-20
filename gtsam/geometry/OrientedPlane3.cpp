/* ----------------------------------------------------------------------------

 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file OrientedPlane3.cpp
 * @date Dec 19, 2013
 * @author Alex Trevor
 * @author Zhaoyang Lv
 * @brief  A plane, represented by a normal direction and perpendicular distance
 */

#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/Point2.h>
#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void OrientedPlane3::print(const string& s) const {
  Vector4 coeffs = planeCoefficients();
  cout << s << " : " << coeffs << endl;
}

/* ************************************************************************* */
OrientedPlane3 OrientedPlane3::transform(const Pose3& xr, OptionalJacobian<3, 3> Hp,
    OptionalJacobian<3, 6> Hr) const {
  Matrix23 D_rotated_plane;
  Matrix22 D_rotated_pose;
  Unit3 n_rotated = xr.rotation().unrotate(n_, D_rotated_plane, D_rotated_pose);

  Vector3 unit_vec = n_rotated.unitVector();
  double pred_d = n_.unitVector().dot(xr.translation()) + d_;

  if (Hr) {
    *Hr = Matrix::Zero(3,6);
    Hr->block<2, 3>(0, 0) = D_rotated_plane;
    Hr->block<1, 3>(2, 3) = unit_vec;
  }
  if (Hp) {
    Vector2 hpp = n_.basis().transpose() * xr.translation();
    *Hp = Z_3x3;
    Hp->block<2, 2>(0, 0) = D_rotated_pose;
    Hp->block<1, 2>(2, 0) = hpp;
    (*Hp)(2, 2) = 1;
  }

  return OrientedPlane3(unit_vec(0), unit_vec(1), unit_vec(2), pred_d);
}

/* ************************************************************************* */
Vector3 OrientedPlane3::error(const OrientedPlane3& plane) const {
  Vector2 n_error = -n_.localCoordinates(plane.n_);
  return Vector3(n_error(0), n_error(1), d_ - plane.d_);
}

/* ************************************************************************* */
Vector3 OrientedPlane3::errorVector(const OrientedPlane3& other, OptionalJacobian<3, 3> H1,
                                    OptionalJacobian<3, 3> H2) const {
  Matrix22 H_n_error_this, H_n_error_other;
  Vector2 n_error = n_.errorVector(other.normal(), H1 ? &H_n_error_this : 0,
                                   H2 ? &H_n_error_other : 0);

  double d_error = d_ - other.d_;

  if (H1) {
    *H1 << H_n_error_this, Vector2::Zero(), 0, 0, 1;
  }
  if (H2) {
    *H2 << H_n_error_other, Vector2::Zero(), 0, 0, -1;
  }

  return Vector3(n_error(0), n_error(1), d_error);
}

/* ************************************************************************* */
OrientedPlane3 OrientedPlane3::retract(const Vector3& v) const {
  return OrientedPlane3(n_.retract(Vector2(v(0), v(1))), d_ + v(2));
}

/* ************************************************************************* */
Vector3 OrientedPlane3::localCoordinates(const OrientedPlane3& y) const {
  Vector2 n_local = n_.localCoordinates(y.n_);
  double d_local = d_ - y.d_;
  return Vector3(n_local(0), n_local(1), -d_local);
}

}
