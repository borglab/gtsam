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
#include <gtsam/base/Lie-inl.h>
#include <boost/foreach.hpp>
#include <cmath>

using namespace std;

namespace gtsam {

/** Explicit instantiation of base class to export members */
INSTANTIATE_LIE(Point2);

static const Matrix oneOne = Matrix_(1, 2, 1.0, 1.0);

/* ************************************************************************* */
void Point2::print(const string& s) const {
  cout << s << *this << endl;
}

/* ************************************************************************* */
bool Point2::equals(const Point2& q, double tol) const {
  return (fabs(x_ - q.x()) < tol && fabs(y_ - q.y()) < tol);
}

/* ************************************************************************* */
double Point2::norm(boost::optional<Matrix&> H) const {
  double r = sqrt(x_ * x_ + y_ * y_);
  if (H) {
    Matrix D_result_d;
    if (fabs(r) > 1e-10)
      D_result_d = Matrix_(1, 2, x_ / r, y_ / r);
    else
      D_result_d = oneOne; // TODO: really infinity, why 1 here??
    *H = D_result_d;
  }
  return r;
}

/* ************************************************************************* */
static const Matrix I2 = eye(2);
double Point2::distance(const Point2& point, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  Point2 d = point - *this;
  if (H1 || H2) {
    Matrix H;
    double r = d.norm(H);
    if (H1) *H1 = -H;
    if (H2) *H2 =  H;
    return r;
  } else
    return d.norm();
}

/* ************************************************************************* */
list<Point2> Point2::CircleCircleIntersection(double R, Point2 c, double r) {

  list<Point2> solutions;

  // Math inspired by http://paulbourke.net/geometry/circlesphere/
  // Changed to avoid sqrt in case there are 0 or 1 intersections, and only one div

  // squared distance between circle centers.
  double d2 = c.x() * c.x() + c.y() * c.y();

  // Check for solutions
  double R2 = R*R;
  double b = R2 - r*r + d2;
  double b2 = b*b;

  // Return empty list if no solution
  // test below is equivalent to h2>0 == !(d > (R + r) || d < (R - r))
  if (4*R2*d2 >= b2) {
    // Determine p2, the point where the line through the circle
    // intersection points crosses the Line between the circle centers.
    double i2 = 1.0/d2;
    double f = 0.5*b*i2;
    Point2 p2 = f * c;

    // if touching, just return one point
    double h2 = R2 - 0.25*b2*i2;
    if (h2 < 1e-9)
      solutions.push_back(p2);
    else {
      // determine the offsets of the intersection points from p
      Point2 offset = sqrt(h2*i2) * Point2(-c.y(), c.x());

      // Determine the absolute intersection points.
      solutions.push_back(p2 + offset);
      solutions.push_back(p2 - offset);
    }
  }
  return solutions;
}

/* ************************************************************************* */
list<Point2> Point2::CircleCircleIntersection(Point2 c1, double r1, Point2 c2, double r2) {
  list<Point2> solutions = Point2::CircleCircleIntersection(r1,c2-c1,r2);
  BOOST_FOREACH(Point2& p, solutions) p+= c1;
  return solutions;
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const Point2& p) {
  os << '(' << p.x() << ", " << p.y() << ')';
  return os;
}

} // namespace gtsam
