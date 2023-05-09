/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Point2.cpp
 * @brief   2D Point
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Point2.h>
#include <cmath>
#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
double norm2(const Point2& p, OptionalJacobian<1,2> H) {
  double r = std::sqrt(p.x() * p.x() + p.y() * p.y());
  if (H) {
    if (std::abs(r) > 1e-10)
      *H << p.x() / r, p.y() / r;
    else
      *H << 1, 1;  // really infinity, why 1 ?
  }
  return r;
}

/* ************************************************************************* */
double distance2(const Point2& p, const Point2& q, OptionalJacobian<1, 2> H1,
                 OptionalJacobian<1, 2> H2) {
  Point2 d = q - p;
  if (H1 || H2) {
    Matrix12 H;
    double r = norm2(d, H);
    if (H1) *H1 = -H;
    if (H2) *H2 =  H;
    return r;
  } else {
    return d.norm();
  }
}

/* ************************************************************************* */
// Math inspired by http://paulbourke.net/geometry/circlesphere/
std::optional<Point2> circleCircleIntersection(double R_d, double r_d,
    double tol) {

  double R2_d2 = R_d*R_d; // Yes, RD-D2 !
  double f = 0.5 + 0.5*(R2_d2 - r_d*r_d);
  double h2 = R2_d2 - f*f; // just right triangle rule

  // h^2<0 is equivalent to (d > (R + r) || d < (R - r))
  // Hence, there are only solutions if >=0
  if (h2<-tol) return {}; // allow *slightly* negative
  else if (h2<tol) return Point2(f,0.0); // one solution
  else return Point2(f,std::sqrt(h2)); // two solutions
}

/* ************************************************************************* */
list<Point2> circleCircleIntersection(Point2 c1, Point2 c2,
    std::optional<Point2> fh) {

  list<Point2> solutions;
  // If fh==std::nullopt, there are no solutions, i.e., d > (R + r) || d < (R - r)
  if (fh) {
    // vector between circle centers
    Point2 c12 = c2-c1;

    // Determine p2, the point where the line through the circle
    // intersection points crosses the line between the circle centers.
    Point2 p2 = c1 + fh->x() * c12;

    // If h == 0, the circles are touching, so just return one point
    if (fh->y()==0.0)
      solutions.push_back(p2);
    else {
      // determine the offsets of the intersection points from p
      Point2 offset = fh->y() * Point2(-c12.y(), c12.x());

      // Determine the absolute intersection points.
      solutions.push_back(p2 + offset);
      solutions.push_back(p2 - offset);
    }
  }
  return solutions;
}

/* ************************************************************************* */
list<Point2> circleCircleIntersection(Point2 c1, double r1, Point2 c2,
    double r2, double tol) {

  // distance between circle centers.
  double d = distance2(c1, c2);

  // centers coincide, either no solution or infinite number of solutions.
  if (d<1e-9) return list<Point2>();

  // Calculate f and h given normalized radii
  double _d = 1.0/d, R_d = r1*_d, r_d=r2*_d;
  std::optional<Point2> fh = circleCircleIntersection(R_d,r_d);

  // Call version that takes fh
  return circleCircleIntersection(c1, c2, fh);
}

Point2Pair means(const std::vector<Point2Pair> &abPointPairs) {
  const size_t n = abPointPairs.size();
  if (n == 0) throw std::invalid_argument("Point2::mean input Point2Pair vector is empty");
  Point2 aSum(0, 0), bSum(0, 0);
  for (const Point2Pair &abPair : abPointPairs) {
    aSum += abPair.first;
    bSum += abPair.second;
  }
  const double f = 1.0 / n;
  return {aSum * f, bSum * f};
}
  
/* ************************************************************************* */
ostream &operator<<(ostream &os, const gtsam::Point2Pair &p) {
  os << p.first << " <-> " << p.second;
  return os;
}

} // namespace gtsam
