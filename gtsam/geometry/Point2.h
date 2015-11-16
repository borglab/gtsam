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
class GTSAM_EXPORT Point2 {
private:

  double x_, y_;

public:
  enum { dimension = 2 };
  /// @name Standard Constructors
  /// @{

  /// default constructor
  Point2(): x_(0), y_(0) {}

  /// construct from doubles
  Point2(double x, double y): x_(x), y_(y) {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  /// construct from 2D vector
  Point2(const Vector2& v) {
    x_ = v(0);
    y_ = v(1);
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
  inline static Point2 identity() {return Point2();}

  /// inverse
  inline Point2 operator- () const {return Point2(-x_,-y_);}

  /// add
  inline Point2 operator + (const Point2& q) const {return Point2(x_+q.x_,y_+q.y_);}

  /// subtract
  inline Point2 operator - (const Point2& q) const {return Point2(x_-q.x_,y_-q.y_);}

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

  /** @deprecated The following function has been deprecated, use distance above */
  inline double dist(const Point2& p2) const {
    return (p2 - *this).norm();
  }

  /// multiply with a scalar
  inline Point2 operator * (double s) const {return Point2(x_*s,y_*s);}

  /// divide by a scalar
  inline Point2 operator / (double q) const {return Point2(x_/q,y_/q);}

  /// @}
  /// @name Standard Interface
  /// @{

  /// equality
  inline bool operator ==(const Point2& q) const {return x_==q.x_ && y_==q.y_;}

  /// get x
  double x() const {return x_;}

  /// get y
  double y() const {return y_;}

  /// return vectorized form (column-wise). TODO: why does this function exist?
  Vector2 vector() const { return Vector2(x_, y_); }

  /// @}

  /// @name Deprecated
  /// @{
  inline void operator += (const Point2& q) {x_+=q.x_;y_+=q.y_;}
  inline void operator *= (double s) {x_*=s;y_*=s;}
  Point2 inverse() const { return -(*this);}
  Point2 compose(const Point2& q) const { return (*this)+q;}
  Point2 between(const Point2& q) const { return q-(*this);}
  Vector2 localCoordinates(const Point2& q) const { return between(q).vector();}
  Point2 retract(const Vector2& v) const { return compose(Point2(v));}
  static Vector2 Logmap(const Point2& p) { return p.vector();}
  static Point2 Expmap(const Vector2& v) { return Point2(v);}
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
    ar & BOOST_SERIALIZATION_NVP(x_);
    ar & BOOST_SERIALIZATION_NVP(y_);
  }

  /// @}
};

// For MATLAB wrapper
typedef std::vector<Point2> Point2Vector;

/// multiply with scalar
inline Point2 operator*(double s, const Point2& p) {return p*s;}

template<>
struct traits<Point2> : public internal::VectorSpace<Point2> {};

} // \ namespace gtsam

