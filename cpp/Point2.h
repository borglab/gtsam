/**
 * @file    Point2.h
 * @brief   2D Point
 * @author  Frank Dellaert
 */

#pragma once

#include <boost/serialization/nvp.hpp>
#include "Vector.h"
#include "Matrix.h"
#include "Testable.h"
#include "Lie.h"

namespace gtsam {

  /**
   * A 2D point
   * Derived from testable so has standard print and equals, and assert_equals works
   * Functional, so no set functions: once created, a point is constant.
   */
  class Point2: Testable<Point2>, public Lie<Point2> {
  private:
    double x_, y_;
		
  public:
    Point2(): x_(0), y_(0) {}
    Point2(const Point2 &p) : x_(p.x_), y_(p.y_) {}
    Point2(double x, double y): x_(x), y_(y) {}
    Point2(const Vector& v) : x_(v(0)), y_(v(1)) {}

    /** print with optional string */
    void print(const std::string& s = "") const;

    /** equals with an tolerance, prints out message if unequal*/
    bool equals(const Point2& q, double tol = 1e-9) const;

    /** get functions for x, y */
    double x() const {return x_;}
    double y() const {return y_;}

    /** return vectorized form (column-wise) */
    Vector vector() const { return Vector_(2, x_, y_); }

    /** operators */
    inline bool   operator ==(const Point2& q) const {return x_==q.x_ && q.y_==q.y_;}
    inline Point2 operator + (const Point2& q) const {return Point2(x_+q.x_,y_+q.y_);}
    inline Point2 operator - (const Point2& q) const {return Point2(x_-q.x_,y_-q.y_);}
    inline Point2 operator / (double q) const {return Point2(x_/q,y_/q);}

    /** distance between two points */
    double dist(const Point2& p2) const;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
      void serialize(Archive & ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(x_);
      ar & BOOST_SERIALIZATION_NVP(y_);
    }
  };

  // Lie group functions

  // Dimensionality of the tangent space
  inline size_t dim(const Point2& obj) { return 2; }

  // Exponential map around identity - just create a Point2 from a vector
  template<> inline Point2 expmap(const Vector& dp) { return Point2(dp); }

  // Log map around identity - just return the Point2 as a vector
  inline Vector logmap(const Point2& dp) { return Vector_(2, dp.x(), dp.y()); }

  // "Compose", just adds the coordinates of two points.
  inline Point2 compose(const Point2& p1, const Point2& p0) { return p0+p1; }
  inline Matrix Dcompose1(const Point2& p1, const Point2& p0) {
    return Matrix_(2,2,
        1.0, 0.0,
        0.0, 1.0); }
  inline Matrix Dcompose2(const Point2& p1, const Point2& p0) {
    return Matrix_(2,2,
        1.0, 0.0,
        0.0, 1.0); }

  // "Inverse" - negates each coordinate such that compose(p,inverse(p))=Point2()
  inline Point2 inverse(const Point2& p) { return Point2(-p.x(), -p.y()); }

}

