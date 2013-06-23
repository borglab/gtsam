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
// Calculate h, the distance of the intersections of two circles from the center line.
// This is a dimensionless fraction of the distance d between the circle centers,
// and also determines how "good" the intersection is. If the circles do not intersect
// or they are identical, returns boost::none. If one solution, h -> 0.
// @param R_d : R/d, ratio of radius of first circle to distance between centers
// @param r_d : r/d, ratio of radius of second circle to distance between centers
// @param tol: absolute tolerance below which we consider touching circles
// Math inspired by http://paulbourke.net/geometry/circlesphere/
static boost::optional<double> circleCircleQuality(double R_d, double r_d, double tol=1e-9) {

  double R2_d2 = R_d*R_d; // Yes, RD-D2 !
  double f = 0.5 + 0.5*(R2_d2 - r_d*r_d);
  double h2 = R2_d2 - f*f;

  // h^2<0 is equivalent to (d > (R + r) || d < (R - r))
  // Hence, there are only solutions if >=0
  if (h2<-tol) return boost::none; // allow *slightly* negative
  else if (h2<tol) return 0.0; // one solution
  else return sqrt(h2); // two solutions
}

/* ************************************************************************* */
// Math inspired by http://paulbourke.net/geometry/circlesphere/
list<Point2> Point2::CircleCircleIntersection(double R, Point2 c, double r) {

  list<Point2> solutions;

  // distance between circle centers.
  double d2 = c.x() * c.x() + c.y() * c.y(), d = sqrt(d2);

  // circles coincide, either no solution or infinite number of solutions.
  if (d2<1e-9) return solutions;

  // Calculate h, the distance of the intersections from the center line,
  // as a dimensionless fraction of  the distance d.
  // It is the solution of a quadratic, so it has either 2 solutions, is 0, or none
  double _d = 1.0/d, R_d = R*_d, r_d=r*_d;
  boost::optional<double> h = circleCircleQuality(R_d,r_d);

  // If h== boost::none, there are no solutions, i.e., d > (R + r) || d < (R - r)
  if (h) {
    // Determine p2, the point where the line through the circle
    // intersection points crosses the line between the circle centers.
    double f = 0.5 + 0.5*(R_d*R_d - r_d*r_d);
    Point2 p2 = f * c;

    // If h == 0, the circles are touching, so just return one point
    if (h==0.0)
      solutions.push_back(p2);
    else {
      // determine the offsets of the intersection points from p
      Point2 offset = (*h) * Point2(-c.y(), c.x());

      // Determine the absolute intersection points.
      solutions.push_back(p2 + offset);
      solutions.push_back(p2 - offset);
    }
  }
  return solutions;
}

//list<Point2> Point2::CircleCircleIntersection(double R, Point2 c, double r) {
//
//  list<Point2> solutions;
//
//  // Math inspired by http://paulbourke.net/geometry/circlesphere/
//  // Changed to avoid sqrt in case there are 0 or 1 intersections, and only one div
//
//  // squared distance between circle centers.
//  double d2 = c.x() * c.x() + c.y() * c.y();
//
//  // A crucial quantity we compute is h, a the distance of the intersections
//  // from the center line, as a dimensionless fraction of  the distance d.
//  // It is the solution of a quadratic, so it has either 2 solutions, is 0, or none
//  // We calculate it as sqrt(h^2*d^4)/d^2, but first check whether h^2*d^4>=0
//  double R2 = R*R;
//  double R2d2 = R2*d2; // yes, R2-D2!
//  double b = R2 + d2 - r*r;
//  double b2 = b*b;
//  double h2d4 = R2d2 - 0.25*b2; // h^2*d^4
//
//  // h^2*d^4<0 is equivalent to (d > (R + r) || d < (R - r))
//  // Hence, there are only solutions if >=0
//  if (h2d4>=0) {
//    // Determine p2, the point where the line through the circle
//    // intersection points crosses the line between the circle centers.
//    double i2 = 1.0/d2;
//    double f = 0.5*b*i2;
//    Point2 p2 = f * c;
//
//    // If h^2*d^4 == 0, the circles are touching, so just return one point
//    if (h2d4 < 1e-9)
//      solutions.push_back(p2);
//    else {
//      // determine the offsets of the intersection points from p
//      double h = sqrt(h2d4)*i2; // h = sqrt(h^2*d^4)/d^2
//      Point2 offset = h * Point2(-c.y(), c.x());
//
//      // Determine the absolute intersection points.
//      solutions.push_back(p2 + offset);
//      solutions.push_back(p2 - offset);
//    }
//  }
//  return solutions;
//}
//
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
