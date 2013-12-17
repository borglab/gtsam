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
 * @brief Develop a Sphere2 class - basically a point on a unit sphere
 */

#include <gtsam/geometry/Sphere2.h>
#include <gtsam/geometry/Point2.h>

using namespace std;

namespace gtsam {

Sphere2::~Sphere2() {
}

Sphere2 Sphere2::retract(const Vector& v) const {

  // Get the vector form of the point and the basis matrix
  Vector p = Point3::Logmap(p_);
  Vector axis;
  Matrix B = getBasis(&axis);

  // Compute the 3D ξ^ vector
  Vector xi_hat = v(0) * B.col(0) + v(1) * B.col(1);
  Vector newPoint = p + xi_hat;

  // Project onto the manifold, i.e. the closest point on the circle to the new location; same as
  // putting it onto the unit circle
  Vector projected = newPoint / newPoint.norm();

#ifdef DEBUG_SPHERE2_RETRACT
  cout << "retract output for Matlab visualization (copy/paste =/): \n";
  cout << "p = [" << p.transpose() << "];\n";
  cout << "b1 = [" << B.col(0).transpose() << "];\n";
  cout << "b2 = [" << B.col(1).transpose() << "];\n";
  cout << "axis = [" << axis.transpose() << "];\n";
  cout << "xi_hat = [" << xi_hat.transpose() << "];\n";
  cout << "newPoint = [" << newPoint.transpose() << "];\n";
  cout << "projected = [" << projected.transpose() << "];\n";
#endif

  Sphere2 result(Point3::Expmap(projected));
  return result;
}

Vector Sphere2::localCoordinates(const Sphere2& y) const {

  // Make sure that the angle different between x and y is less than 90. Otherwise,
  // we can project x + ξ^ from the tangent space at x to y.
  double cosAngle = y.p_.dot(p_);
  assert(cosAngle > 0.0 && "Can not retract from x to y in the first place.");

  // Get the basis matrix
  Matrix B = getBasis();

  // Create the vector forms of p and q (the Point3 of y).
  Vector p = Point3::Logmap(p_);
  Vector q = Point3::Logmap(y.p_);

  // Compute the basis coefficients [ξ1,ξ2] = (B'q)/(p'q).
  double alpha = p.transpose() * q;
  assert(alpha != 0.0);
  Matrix coeffs = (B.transpose() * q) / alpha;
  Vector result = Vector_(2, coeffs(0, 0), coeffs(1, 0));
  return result;
}

Matrix Sphere2::getBasis(Vector* axisOutput) const {

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
  Matrix basis = Matrix_(3, 2, b1.x(), b2.x(), b1.y(), b2.y(), b1.z(), b2.z());

  // Return the axis if requested
  if (axisOutput != NULL)
    *axisOutput = Point3::Logmap(axis);
  return basis;
}

}
