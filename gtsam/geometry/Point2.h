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
#include <boost/serialization/nvp.hpp>

namespace gtsam {

#ifdef GTSAM_TYPEDEF_POINTS_TO_VECTORS

  /// As of GTSAM 4, in order to make GTSAM more lean,
  /// it is now possible to just typedef Point2 to Vector2
  typedef Vector2 Point2;

#else

/**
 * A 2D point
 * Complies with the Testable Concept
 * Functional, so no set functions: once created, a point is constant.
 * @addtogroup geometry
 * \nosubgrouping
 */
class Point2 : public Vector2 {
private:

public:
  enum { dimension = 2 };
  /// @name Standard Constructors
  /// @{

  /// default constructor
  Point2() {}

  using Vector2::Vector2;

  /// @}
  /// @name Advanced Constructors
  /// @{

  /// construct from 2D vector
  explicit Point2(const Vector2& v):Vector2(v) {}
  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  GTSAM_EXPORT void print(const std::string& s = "") const;

  /// equals with an tolerance, prints out message if unequal
  GTSAM_EXPORT bool equals(const Point2& q, double tol = 1e-9) const;

  /// @}
  /// @name Group
  /// @{

  /// identity
  inline static Point2 identity() {return Point2(0,0);}

  /// @}
  /// @name Vector Space
  /// @{

  /** creates a unit vector */
  Point2 unit() const { return *this/norm(); }

  /** norm of point, with derivative */
  GTSAM_EXPORT double norm(OptionalJacobian<1,2> H = boost::none) const;

  /** distance between two points */
  GTSAM_EXPORT double distance(const Point2& p2, OptionalJacobian<1,2> H1 = boost::none,
      OptionalJacobian<1,2> H2 = boost::none) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// equality
  inline bool operator ==(const Point2& q) const {return x()==q.x() && y()==q.y();}

  /// get x
  inline double x() const {return (*this)[0];}

  /// get y
  inline double y() const {return (*this)[1];}

  /// return vectorized form (column-wise).
  const Vector2& vector() const { return *this; }

  /// @}

  /// Streaming
  GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const Point2& p);

 private:
  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Vector2);}

 /// @}
};

template<>
struct traits<Point2> : public internal::VectorSpace<Point2> {
};

#endif // GTSAM_TYPEDEF_POINTS_TO_VECTORS

/// Distance of the point from the origin, with Jacobian
GTSAM_EXPORT double norm2(const Point2& p, OptionalJacobian<1, 2> H = boost::none);

/// distance between two points
GTSAM_EXPORT double distance2(const Point2& p1, const Point2& q,
                 OptionalJacobian<1, 2> H1 = boost::none,
                 OptionalJacobian<1, 2> H2 = boost::none);

// Convenience typedef
typedef std::pair<Point2, Point2> Point2Pair;
GTSAM_EXPORT std::ostream &operator<<(std::ostream &os, const gtsam::Point2Pair &p);

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
GTSAM_EXPORT boost::optional<Point2> circleCircleIntersection(double R_d, double r_d, double tol = 1e-9);

/*
 * @brief Circle-circle intersection, from the normalized radii solution.
 * @param c1 center of first circle
 * @param c2 center of second circle
 * @return list of solutions (0,1, or 2). Identical circles will return empty list, as well.
 */
GTSAM_EXPORT std::list<Point2> circleCircleIntersection(Point2 c1, Point2 c2, boost::optional<Point2> fh);

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

} // \ namespace gtsam

