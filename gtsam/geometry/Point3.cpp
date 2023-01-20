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
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
double distance3(const Point3 &p1, const Point3 &q, OptionalJacobian<1, 3> H1,
                 OptionalJacobian<1, 3> H2) {
  double d = (q - p1).norm();
  if (H1) {
    *H1 << p1.x() - q.x(), p1.y() - q.y(), p1.z() - q.z();
    *H1 = *H1 *(1. / d);
  }
  if (H2) {
    *H2 << -p1.x() + q.x(), -p1.y() + q.y(), -p1.z() + q.z();
    *H2 = *H2 *(1. / d);
  }
  return d;
}

double norm3(const Point3 &p, OptionalJacobian<1, 3> H) {
  double r = sqrt(p.x() * p.x() + p.y() * p.y() + p.z() * p.z());
  if (H) {
    if (std::abs(r) > 1e-10)
      *H << p.x() / r, p.y() / r, p.z() / r;
    else
      *H << 1, 1, 1;  // really infinity, why 1 ?
  }
  return r;
}

Point3 normalize(const Point3 &p, OptionalJacobian<3, 3> H) {
  Point3 normalized = p / p.norm();
  if (H) {
    // 3*3 Derivative
    double x2 = p.x() * p.x(), y2 = p.y() * p.y(), z2 = p.z() * p.z();
    double xy = p.x() * p.y(), xz = p.x() * p.z(), yz = p.y() * p.z();
    *H << y2 + z2, -xy, -xz, /**/ -xy, x2 + z2, -yz, /**/ -xz, -yz, x2 + y2;
    *H /= pow(x2 + y2 + z2, 1.5);
  }
  return normalized;
}

Point3 cross(const Point3 &p, const Point3 &q, OptionalJacobian<3, 3> H1,
             OptionalJacobian<3, 3> H2) {
  if (H1) *H1 << skewSymmetric(-q.x(), -q.y(), -q.z());
  if (H2) *H2 << skewSymmetric(p.x(), p.y(), p.z());
  return Point3(p.y() * q.z() - p.z() * q.y(), p.z() * q.x() - p.x() * q.z(),
                p.x() * q.y() - p.y() * q.x());
}

double dot(const Point3 &p, const Point3 &q, OptionalJacobian<1, 3> H1,
           OptionalJacobian<1, 3> H2) {
  if (H1) *H1 << q.x(), q.y(), q.z();
  if (H2) *H2 << p.x(), p.y(), p.z();
  return p.x() * q.x() + p.y() * q.y() + p.z() * q.z();
}

Point3Pair means(const std::vector<Point3Pair> &abPointPairs) {
  const size_t n = abPointPairs.size();
  if (n == 0) throw std::invalid_argument("Point3::mean input Point3Pair vector is empty");
  Point3 aSum(0, 0, 0), bSum(0, 0, 0);
  for (const Point3Pair &abPair : abPointPairs) {
    aSum += abPair.first;
    bSum += abPair.second;
  }
  const double f = 1.0 / n;
  return {aSum * f, bSum * f};
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const gtsam::Point3Pair &p) {
  os << p.first << " <-> " << p.second;
  return os;
}

/* ************************************************************************* */

} // namespace gtsam
