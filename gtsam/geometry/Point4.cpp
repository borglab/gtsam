/**
 * @file   Point4.cpp
 * @brief  gnss state vector -- {x,y,z,cb}
 * @author Ryan Watson
 */

#include <gtsam/geometry/Point4.h>
#include <gtsam/geometry/Point3.h>
#include <cmath>

using namespace std;

namespace gtsam {

#ifndef GTSAM_TYPEDEF_POINTS_TO_VECTORS
bool Point4::equals(const Point4 &q, double tol) const {
  return (fabs(x() - q.x()) < tol && fabs(y() - q.y()) < tol &&
          fabs(z() - q.z())<tol && fabs(cb() - q.cb())< tol);
}

void Point4::print(const string& s) const {
  cout << s << *this << endl;
}

/* ************************************************************************* */
double Point4::distance(const Point4 &q, OptionalJacobian<1, 4> H1,
                        OptionalJacobian<1, 4> H2) const {
  return gtsam::distance4(*this,q,H1,H2);
}

double Point4::norm(OptionalJacobian<1,4> H) const {
  return gtsam::norm4(*this, H);
}


double Point4::dot(const Point4 &q, OptionalJacobian<1, 4> H1,
                   OptionalJacobian<1, 4> H2) const {
  return gtsam::dot(*this, q, H1, H2);
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const Point4& p) {
  os << '[' << p.x() << ", " << p.y() << ", " << p.z() << ", " << p.cb() << "]'";
  return os;
}

/* ************************************************************************* */
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
Point4 Point4::add(const Point4 &q, OptionalJacobian<4,4> H1,
    OptionalJacobian<4,4> H2) const {
  if (H1) *H1 = I_4x4;
  if (H2) *H2 = I_4x4;
  return *this + q;
}

Point4 Point4::sub(const Point4 &q, OptionalJacobian<4,4> H1,
    OptionalJacobian<4,4> H2) const {
  if (H1) *H1 = I_4x4;
  if (H2) *H2 = -I_4x4;
  return *this - q;
}
#endif

#endif
/* ************************************************************************* */
double distance4(const Point4 &p1, const Point4 &q, OptionalJacobian<1, 4> H1,
                 OptionalJacobian<1, 4> H2) {
  double range = (q - p1).norm();
  if (H1) {
    *H1 << p1.x() - q.x(), p1.y() - q.y(), p1.z() - q.z(), p1.cb() - q.cb();
    *H1 = *H1 *(1. / range);
  }
  if (H2) {
    *H2 << -p1.x() + q.x(), -p1.y() + q.y(), -p1.z() + q.z(), -q.cb() + q.cb();
    *H2 = *H2 *(1. / range);
  }
  return range;
}

double geoDist(const Point3 &p1, const Point3 &p2, OptionalJacobian<1, 4> H1,
                 OptionalJacobian<1, 4> H2) {

double geoDist = sqrt( ((p1(0)-p2(0)))*(p1(0)-p2(0)) + ((p1(1)-p2(1))*(p1(1)-p2(1))) + ((p1(2)-p2(2))*(p1(2)-p2(2))) );
  if (H1) {
    *H1 << (p1(0)-p2(0))/geoDist, (p1(1)- p2(1))/geoDist, (p1(2)-p2(2))/geoDist, 1.0;
  }
  if (H2) {
    *H1 << (p1(0)-p2(0))/geoDist, (p1(1)- p2(1))/geoDist, (p1(2)-p2(2))/geoDist, 1.0;
  }
  return geoDist;

}

double norm4(const Point4 &p, OptionalJacobian<1, 4> H) {
  double r = sqrt(p.x() * p.x() + p.y() * p.y() + p.z() * p.z()) + p.cb();
  if (H) {
    if (fabs(r) > 1e-10)
      *H << p.x() / r, p.y() / r, p.z() / r, 1;
    else
      *H << 1, 1, 1, 1;  // really infinity, why 1 ?
  }
  return r;
}


double dot(const Point4 &p, const Point4 &q, OptionalJacobian<1, 4> H1,
           OptionalJacobian<1, 4> H2) {
  if (H1) *H1 << q.x(), q.y(), q.z(), q.cb();
  if (H2) *H2 << p.x(), p.y(), p.z(), p.cb();
  return p.x() * q.x() + p.y() * q.y() + p.z() * q.z() + p.cb() * q.cb();
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const gtsam::Point4Pair &p) {
  os << p.first << " <-> " << p.second;
  return os;
}

/* ************************************************************************* */

} // namespace gtsam
