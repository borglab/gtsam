/**
 * @file  Point3.cpp
 * @brief 3D Point
 */

#include "Point3.h"
#include "Lie-inl.h"


namespace gtsam {

  /** Explicit instantiation of base class to export members */
  template class Lie<Point3>;

  /* ************************************************************************* */
  bool Point3::equals(const Point3 & q, double tol) const {
    return (fabs(x_ - q.x()) < tol && fabs(y_ - q.y()) < tol && fabs(z_ - q.z()) < tol);
  }

  void Point3::print(const std::string& s) const {
    std::cout << s << "(" << x_ << ", " << y_ <<  ", " << z_ << ")" << std::endl;
  }

  /* ************************************************************************* */

  bool Point3::operator== (const Point3& q) const {
    return x_ == q.x_ && y_ == q.y_ && z_ == q.z_;
  }

  /* ************************************************************************* */
  Point3 Point3::operator+(const Point3& q) const {
    return Point3( x_ + q.x_, y_ + q.y_, z_ + q.z_ );
  }

  /* ************************************************************************* */
  Point3 Point3::operator- (const Point3& q) const {
    return Point3( x_ - q.x_, y_ - q.y_, z_ - q.z_ );
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
  Point3 add(const Point3 &p, const Point3 &q) {
    return p+q;
  }
  /* ************************************************************************* */
  Matrix Dadd1(const Point3 &p, const Point3 &q) {
    return eye(3,3);
  }
  /* ************************************************************************* */
  Matrix Dadd2(const Point3 &p, const Point3 &q) {
    return eye(3,3);
  }
  /* ************************************************************************* */
  Point3 sub(const Point3 &p, const Point3 &q) {
    return p+q;
  }
  /* ************************************************************************* */
  Matrix Dsub1(const Point3 &p, const Point3 &q) {
    return eye(3,3);
  }
  /* ************************************************************************* */
  Matrix Dsub2(const Point3 &p, const Point3 &q) {
    return -eye(3,3);
  }
  /* ************************************************************************* */
  Point3 cross(const Point3 &p, const Point3 &q)
  {
    return Point3( p.y_*q.z_ - p.z_*q.y_,
        p.z_*q.x_ - p.x_*q.z_,
        p.x_*q.y_ - p.y_*q.x_ );
  }
  /* ************************************************************************* */
  double dot(const Point3 &p, const Point3 &q)
  {
    return ( p.x_*q.x_ + p.y_*q.y_ + p.z_*q.z_ );
  }
  /* ************************************************************************* */
  double norm(const Point3 &p)
  {
    return sqrt( p.x_*p.x_ + p.y_*p.y_ + p.z_*p.z_ );
  }
  /* ************************************************************************* */

} // namespace gtsam
