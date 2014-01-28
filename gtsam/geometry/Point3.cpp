/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Point3.cpp
 * @brief 3D Point
 */

#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Lie-inl.h>

using namespace std;

namespace gtsam {

/** Explicit instantiation of base class to export members */
INSTANTIATE_LIE(Point3);

/* ************************************************************************* */
bool Point3::equals(const Point3 & q, double tol) const {
  return (fabs(x_ - q.x()) < tol && fabs(y_ - q.y()) < tol
      && fabs(z_ - q.z()) < tol);
}

/* ************************************************************************* */

void Point3::print(const string& s) const {
  cout << s << *this << endl;
}

/* ************************************************************************* */

bool Point3::operator==(const Point3& q) const {
  return x_ == q.x_ && y_ == q.y_ && z_ == q.z_;
}

/* ************************************************************************* */
Point3 Point3::operator+(const Point3& q) const {
  return Point3(x_ + q.x_, y_ + q.y_, z_ + q.z_);
}

/* ************************************************************************* */
Point3 Point3::operator-(const Point3& q) const {
  return Point3(x_ - q.x_, y_ - q.y_, z_ - q.z_);
}

/* ************************************************************************* */
Point3 Point3::operator*(double s) const {
  return Point3(x_ * s, y_ * s, z_ * s);
}

/* ************************************************************************* */
Point3 Point3::operator/(double s) const {
  return Point3(x_ / s, y_ / s, z_ / s);
}

/* ************************************************************************* */
Point3 Point3::add(const Point3 &q, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  if (H1)
    *H1 = eye(3, 3);
  if (H2)
    *H2 = eye(3, 3);
  return *this + q;
}

/* ************************************************************************* */
Point3 Point3::sub(const Point3 &q, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  if (H1)
    *H1 = eye(3, 3);
  if (H2)
    *H2 = -eye(3, 3);
  return *this - q;
}

/* ************************************************************************* */
Point3 Point3::cross(const Point3 &q) const {
  return Point3(y_ * q.z_ - z_ * q.y_, z_ * q.x_ - x_ * q.z_,
      x_ * q.y_ - y_ * q.x_);
}

/* ************************************************************************* */
double Point3::dot(const Point3 &q) const {
  return (x_ * q.x_ + y_ * q.y_ + z_ * q.z_);
}

/* ************************************************************************* */
double Point3::norm() const {
  return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
}

/* ************************************************************************* */
Point3 Point3::normalize(boost::optional<Matrix&> H) const {
  Point3 normalized = *this / norm();
  if (H) {
    // 3*3 Derivative
    double x2 = x_ * x_, y2 = y_ * y_, z2 = z_ * z_;
    double xy = x_ * y_, xz = x_ * z_, yz = y_ * z_;
    H->resize(3, 3);
    *H << y2 + z2, -xy, -xz, /**/-xy, x2 + z2, -yz, /**/-xz, -yz, x2 + y2;
    *H /= pow(x2 + y2 + z2, 1.5);
  }
  return normalized;
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const Point3& p) {
  os << '[' << p.x() << ", " << p.y() << ", " << p.z() << "]\';";
  return os;
}

/* ************************************************************************* */

} // namespace gtsam
