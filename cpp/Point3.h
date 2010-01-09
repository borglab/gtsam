/**
 * @file   Point3.h
 * @brief  3D Point
 * @author Alireza Fathi
 * @author Christian Potthast
 * @author Frank Dellaert
 */

// \callgraph

#pragma once

#include <boost/serialization/nvp.hpp>

#include "Matrix.h"
#include "Testable.h"
#include "Lie.h"

namespace gtsam {

  /** A 3D point */
  class Point3: Testable<Point3>, public Lie<Point3> {
  private:
    double x_, y_, z_;  
		
  public:
    Point3(): x_(0), y_(0), z_(0) {}
    Point3(const Point3 &p) : x_(p.x_), y_(p.y_), z_(p.z_) {}
    Point3(double x, double y, double z): x_(x), y_(y), z_(z) {}
    Point3(const Vector& v) : x_(v(0)), y_(v(1)), z_(v(2)) {}

    /** print with optional string */
    void print(const std::string& s = "") const;

    /** equals with an tolerance */
    bool equals(const Point3& p, double tol = 1e-9) const;

    /** return vectorized form (column-wise)*/
    Vector vector() const {
      //double r[] = { x_, y_, z_ };
      Vector v(3); v(0)=x_; v(1)=y_; v(2)=z_;
      return v;
    }

    /** get functions for x, y, z */
    double x() const {return x_;}
    double y() const {return y_;}
    double z() const {return z_;}

    /** operators */
    Point3 operator - () const { return Point3(-x_,-y_,-z_);}
    bool   operator ==(const Point3& q) const;
    Point3 operator + (const Point3& q) const;
    Point3 operator - (const Point3& q) const;
    Point3 operator * (double s) const;
    Point3 operator / (double s) const;

    /** distance between two points */
    double dist(const Point3& p2) const {
      return sqrt(pow(x()-p2.x(),2.0) + pow(y()-p2.y(),2.0) + pow(z()-p2.z(),2.0));
    }

    /** friends */
    friend Point3 cross(const Point3 &p1, const Point3 &p2);
    friend double dot(const Point3 &p1, const Point3 &p2);
    friend double norm(const Point3 &p1);

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
      void serialize(Archive & ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(x_);
      ar & BOOST_SERIALIZATION_NVP(y_);
      ar & BOOST_SERIALIZATION_NVP(z_);
    }
  };


  /** return DOF, dimensionality of tangent space */

  // Dimensionality of the tangent space
  inline size_t dim(const Point3&) { return 3; }

  // Exponential map at identity - just create a Point3 from x,y,z
  template<> inline Point3 expmap(const Vector& dp) { return Point3(dp); }

  // Log map at identity - return the x,y,z of this point
  inline Vector logmap(const Point3& dp) { return Vector_(3, dp.x(), dp.y(), dp.z()); }

  // "Compose" - just adds coordinates of two points
  inline Point3 compose(const Point3& p1, const Point3& p0) { return p0+p1; }
  inline Matrix Dcompose1(const Point3& p1, const Point3& p0) {
    return Matrix_(3,3,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0);
  }
  inline Matrix Dcompose2(const Point3& p1, const Point3& p0) {
    return Matrix_(3,3,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0);
  }

  // "Inverse" - negates the coordinates such that compose(p, inverse(p)) = Point3()
  inline Point3 inverse(const Point3& p) { return Point3(-p.x(), -p.y(), -p.z()); }


  // Syntactic sugar for multiplying coordinates by a scalar s*p
  inline Point3 operator*(double s, const Point3& p) { return p*s;}

  /** add two points, add(p,q) is same as p+q */
  Point3   add (const Point3 &p, const Point3 &q);
  Matrix Dadd1(const Point3 &p, const Point3 &q);
  Matrix Dadd2(const Point3 &p, const Point3 &q);

  /** subtract two points, sub(p,q) is same as p-q */
  Point3   sub (const Point3 &p, const Point3 &q);
  Matrix Dsub1(const Point3 &p, const Point3 &q);
  Matrix Dsub2(const Point3 &p, const Point3 &q);

  /** cross product */
  Point3 cross(const Point3 &p, const Point3 &q); 

  /** dot product */
  double dot(const Point3 &p, const Point3 &q);

  /** dot product */
  double norm(const Point3 &p);
}
