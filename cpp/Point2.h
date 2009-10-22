/**
 * @file    Point2.h
 * @brief   2D Point
 * @author  Frank Dellaert
 */

#pragma once

#include <boost/serialization/nvp.hpp>
#include "Vector.h"
#include "Testable.h"

namespace gtsam {

  /**
   * A 2D point
   * Derived from testable so has standard print and equals, and assert_equals works
   * Functional, so no set functions: once created, a point is constant.
   */
  class Point2: Testable<Point2> {
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

    /** return DOF, dimensionality of tangent space */
    size_t dim() const { return 2;}
		
    /** Given 3-dim tangent vector, create new rotation */
    Point2 exmap(const Vector& d) const { 
      return Point2(x_+d(0),y_+d(1));
    }
		
    /** return vectorized form (column-wise) */
    Vector vector() const {
      Vector v(2);
      v(0)=x_;v(1)=y_;
      return v;
    }

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
}

