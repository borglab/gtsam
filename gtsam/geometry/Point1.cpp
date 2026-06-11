/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Point1.cpp
 * @brief   1D Point
 * @author  Sven Lilge
 */

#include <gtsam/geometry/Point1.h>

#include <cmath>
#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
double norm1(const Point1& p, OptionalJacobian<1, 1> H) {
  double r = std::abs(p.x());
  if (H) {
    if (std::abs(p.x()) > 1e-10)
      *H << (p.x() >= 0 ? 1.0 : -1.0);
    else
      *H << 1.0;  // derivative of abs(x) at x=0 is undefined, using 1 as
                  // convention
  }
  return r;
}

/* ************************************************************************* */
double distance1(const Point1& p1, const Point1& q, OptionalJacobian<1, 1> H1,
                 OptionalJacobian<1, 1> H2) {
  Point1 d = q - p1;
  if (H1 || H2) {
    Matrix11 H;
    double r = norm1(d, H);
    if (H1) *H1 = -H;
    if (H2) *H2 = H;
    return r;
  } else {
    return std::abs(d.x());
  }
}

/* ************************************************************************* */
ostream& operator<<(ostream& os, const gtsam::Point1Pair& p) {
  os << p.first << " <-> " << p.second;
  return os;
}

}  // namespace gtsam
