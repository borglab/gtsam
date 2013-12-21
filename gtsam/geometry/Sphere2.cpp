/* ----------------------------------------------------------------------------

 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file Sphere2.h
 * @date Feb 02, 2011
 * @author Can Erdogan
 * @author Frank Dellaert
 * @brief The Sphere2 class - basically a point on a unit sphere
 */

#include <gtsam/geometry/Sphere2.h>
#include <gtsam/geometry/Point2.h>
#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Matrix Sphere2::basis() const {

  // Return cached version if exists
  if (B_.rows() == 3)
    return B_;

  // Get the axis of rotation with the minimum projected length of the point
  Point3 axis;
  double mx = fabs(p_.x()), my = fabs(p_.y()), mz = fabs(p_.z());
  if ((mx <= my) && (mx <= mz))
    axis = Point3(1.0, 0.0, 0.0);
  else if ((my <= mx) && (my <= mz))
    axis = Point3(0.0, 1.0, 0.0);
  else if ((mz <= mx) && (mz <= my))
    axis = Point3(0.0, 0.0, 1.0);
  else
    assert(false);

  // Create the two basis vectors
  Point3 b1 = p_.cross(axis);
  b1 = b1 / b1.norm();
  Point3 b2 = p_.cross(b1);
  b2 = b2 / b2.norm();

  // Create the basis matrix
  B_ = Matrix(3, 2);
  B_ << b1.x(), b2.x(), b1.y(), b2.y(), b1.z(), b2.z();
  return B_;
}

/* ************************************************************************* */
/// The print fuction
void Sphere2::print(const std::string& s) const {
  cout << s << ":" << p_ << endl;
}

/* ************************************************************************* */
Matrix Sphere2::skew() const {
  return skewSymmetric(p_.x(), p_.y(), p_.z());
}

/* ************************************************************************* */
Vector Sphere2::error(const Sphere2& q, boost::optional<Matrix&> H) const {
  Matrix Bt = basis().transpose();
  Vector xi = Bt * q.p_.vector();
  if (H)
    *H = Bt * q.basis();
  return xi;
}

/* ************************************************************************* */
double Sphere2::distance(const Sphere2& q, boost::optional<Matrix&> H) const {
  Vector xi = error(q,H);
  double theta = xi.norm();
  if (H)
    *H = (xi.transpose() / theta) * (*H);
  return theta;
}

/* ************************************************************************* */
Sphere2 Sphere2::retract(const Vector& v) const {

  // Get the vector form of the point and the basis matrix
  Vector p = Point3::Logmap(p_);
  Matrix B = basis();

  // Compute the 3D xi_hat vector
  Vector xi_hat = v(0) * B.col(0) + v(1) * B.col(1);
  Vector newPoint = p + xi_hat;

  // Project onto the manifold, i.e. the closest point on the circle to the new location;
  // same as putting it onto the unit circle
  Vector projected = newPoint / newPoint.norm();

  return Sphere2(Point3::Expmap(projected));
}

/* ************************************************************************* */
Vector Sphere2::localCoordinates(const Sphere2& y) const {

  // Make sure that the angle different between x and y is less than 90. Otherwise,
  // we can project x + xi_hat from the tangent space at x to y.
  double cosAngle = y.p_.dot(p_);
  assert(cosAngle > 0.0 && "Can not retract from x to y.");

  // Get the basis matrix
  Matrix B = basis();

  // Create the vector forms of p and q (the Point3 of y).
  Vector p = Point3::Logmap(p_);
  Vector q = Point3::Logmap(y.p_);

  // Compute the basis coefficients [v0,v1] = (B'q)/(p'q).
  double alpha = p.transpose() * q;
  assert(alpha != 0.0);
  Matrix coeffs = (B.transpose() * q) / alpha;
  Vector result = Vector_(2, coeffs(0, 0), coeffs(1, 0));
  return result;
}
/* ************************************************************************* */

}
