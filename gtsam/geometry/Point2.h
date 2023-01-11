/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Point2.h
 * @brief   2D Point
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/std_optional_serialization.h>
#include <boost/serialization/nvp.hpp>

#include <optional>

namespace gtsam {

/// As of GTSAM 4, in order to make GTSAM more lean,
/// it is now possible to just typedef Point2 to Vector2
typedef Vector2 Point2;
  
// Convenience typedef
using Point2Pair = std::pair<Point2, Point2>;
GTSAM_EXPORT std::ostream &operator<<(std::ostream &os, const gtsam::Point2Pair &p);

using Point2Pairs = std::vector<Point2Pair>;

/// Distance of the point from the origin, with Jacobian
GTSAM_EXPORT double norm2(const Point2& p, OptionalJacobian<1, 2> H = boost::none);

/// distance between two points
GTSAM_EXPORT double distance2(const Point2& p1, const Point2& q,
                 OptionalJacobian<1, 2> H1 = boost::none,
                 OptionalJacobian<1, 2> H2 = boost::none);

// For MATLAB wrapper
typedef std::vector<Point2, Eigen::aligned_allocator<Point2> > Point2Vector;

/// multiply with scalar
inline Point2 operator*(double s, const Point2& p) {
  return p * s;
}

/*
 * @brief Circle-circle intersection, given normalized radii.
 * Calculate f and h, respectively the parallel and perpendicular distance of
 * the intersections of two circles along and from the line connecting the centers.
 * Both are dimensionless fractions of the distance d between the circle centers.
 * If the circles do not intersect or they are identical, returns boost::none.
 * If one solution (touching circles, as determined by tol), h will be exactly zero.
 * h is a good measure for how accurate the intersection will be, as when circles touch
 * or nearly touch, the intersection is ill-defined with noisy radius measurements.
 * @param R_d : R/d, ratio of radius of first circle to distance between centers
 * @param r_d : r/d, ratio of radius of second circle to distance between centers
 * @param tol: absolute tolerance below which we consider touching circles
 * @return optional Point2 with f and h, boost::none if no solution.
 */
GTSAM_EXPORT std::optional<Point2> circleCircleIntersection(double R_d, double r_d, double tol = 1e-9);

/*
 * @brief Circle-circle intersection, from the normalized radii solution.
 * @param c1 center of first circle
 * @param c2 center of second circle
 * @return list of solutions (0,1, or 2). Identical circles will return empty list, as well.
 */
GTSAM_EXPORT std::list<Point2> circleCircleIntersection(Point2 c1, Point2 c2, std::optional<Point2> fh);
  
/// Calculate the two means of a set of Point2 pairs
GTSAM_EXPORT Point2Pair means(const std::vector<Point2Pair> &abPointPairs);

/**
 * @brief Intersect 2 circles
 * @param c1 center of first circle
 * @param r1 radius of first circle
 * @param c2 center of second circle
 * @param r2 radius of second circle
 * @param tol: absolute tolerance below which we consider touching circles
 * @return list of solutions (0,1, or 2). Identical circles will return empty list, as well.
 */
GTSAM_EXPORT std::list<Point2> circleCircleIntersection(Point2 c1, double r1,
    Point2 c2, double r2, double tol = 1e-9);

template <typename A1, typename A2>
struct Range;

template <>
struct Range<Point2, Point2> {
  typedef double result_type;
  double operator()(const Point2& p, const Point2& q,
                    OptionalJacobian<1, 2> H1 = boost::none,
                    OptionalJacobian<1, 2> H2 = boost::none) {
    return distance2(p, q, H1, H2);
  }
};

} // \ namespace gtsam

