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

/**
 * A 2D point
 * Complies with the Testable Concept
 * Functional, so no set functions: once created, a point is constant.
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Point2 : public Vector2 {
private:

public:
  enum { dimension = 2 };
  /// @name Standard Constructors
  /// @{

  /// default constructor
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
    // Deprecated default constructor initializes to zero, in contrast to new behavior below
    Point2() { setZero(); }
#else
    Point2() {
//      throw std::runtime_error("Point2 default");
    }
#endif

  using Vector2::Vector2;

  /// @}
  /// @name Advanced Constructors
  /// @{

  /// construct from 2D vector
  explicit Point2(const Vector2& v):Vector2(v) {}

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
  static boost::optional<Point2> CircleCircleIntersection(double R_d, double r_d,
      double tol = 1e-9);

  /*
   * @brief Circle-circle intersection, from the normalized radii solution.
   * @param c1 center of first circle
   * @param c2 center of second circle
   * @return list of solutions (0,1, or 2). Identical circles will return empty list, as well.
   */
  static std::list<Point2> CircleCircleIntersection(Point2 c1, Point2 c2, boost::optional<Point2>);

  /**
   * @brief Intersect 2 circles
   * @param c1 center of first circle
   * @param r1 radius of first circle
   * @param c2 center of second circle
   * @param r2 radius of second circle
   * @param tol: absolute tolerance below which we consider touching circles
   * @return list of solutions (0,1, or 2). Identical circles will return empty list, as well.
   */
  static std::list<Point2> CircleCircleIntersection(Point2 c1, double r1,
      Point2 c2, double r2, double tol = 1e-9);

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  void print(const std::string& s = "") const;

  /// equals with an tolerance, prints out message if unequal
  bool equals(const Point2& q, double tol = 1e-9) const;

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
  double norm(OptionalJacobian<1,2> H = boost::none) const;

  /** distance between two points */
  double distance(const Point2& p2, OptionalJacobian<1,2> H1 = boost::none,
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

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
  /// @name Deprecated
  /// @{
  inline void operator += (const Point2& q) {x_+=q.x_;y_+=q.y_;}
  inline void operator *= (double s) {x_*=s;y_*=s;}
  Point2 inverse() const { return -(*this);}
  Point2 compose(const Point2& q) const { return (*this)+q;}
  Point2 between(const Point2& q) const { return q-(*this);}
  Vector2 localCoordinates(const Point2& q) const { return between(q);}
  Point2 retract(const Vector2& v) const { return compose(Point2(v));}
  static Vector2 Logmap(const Point2& p) { return p;}
  static Point2 Expmap(const Vector2& v) { return Point2(v);}
  inline double dist(const Point2& p2) const {return distance();}
  /// @}
#endif

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

// Convenience typedef
typedef std::pair<Point2, Point2> Point2Pair;
std::ostream &operator<<(std::ostream &os, const gtsam::Point2Pair &p);

// For MATLAB wrapper
typedef std::vector<Point2> Point2Vector;

/// multiply with scalar
inline Point2 operator*(double s, const Point2& p) {
return p * s;
}

template<>
struct traits<Point2> : public internal::VectorSpace<Point2> {
};

} // \ namespace gtsam

