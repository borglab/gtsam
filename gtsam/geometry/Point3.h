/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Lie.h>

namespace gtsam {

  /**
   * A 3D point
   * @ingroup geometry
   */
  class Point3 {
  public:
	  /// dimension of the variable - used to autodetect sizes
	  static const size_t dimension = 3;

  private:
    double x_, y_, z_;  
		
  public:
    Point3(): x_(0), y_(0), z_(0) {}
    Point3(const Point3 &p) : x_(p.x_), y_(p.y_), z_(p.z_) {}
    Point3(double x, double y, double z): x_(x), y_(y), z_(z) {}
    Point3(const Vector& v) : x_(v(0)), y_(v(1)), z_(v(2)) {}

    /// @name Testable
    /// @{

    /** print with optional string */
    void print(const std::string& s = "") const;

    /** equals with an tolerance */
    bool equals(const Point3& p, double tol = 1e-9) const;

    /// @}
    /// @name Group
    /// @{

    /// identity for group operation
		inline static Point3 identity() {
			return Point3();
		}

    /// "Inverse" - negates the coordinates such that compose(p, inverse(p)) = Point3()
    inline Point3 inverse() const { return Point3(-x_, -y_, -z_); }

    /// "Compose" - just adds coordinates of two points
    inline Point3 compose(const Point3& p2,
    		boost::optional<Matrix&> H1=boost::none,
    		boost::optional<Matrix&> H2=boost::none) const {
  	  if (H1) *H1 = eye(3);
  	  if (H2) *H2 = eye(3);
  	  return *this + p2;
    }

  	/// MATLAB version returns shared pointer
  	boost::shared_ptr<Point3> compose_(const Point3& p2) {
  		return boost::shared_ptr<Point3>(new Point3(compose(p2)));
  	}

    /// @}
    /// @name Manifold
    /// @{

    /// dimension of the variable - used to autodetect sizes
    inline static size_t Dim() { return dimension; }

    /// return dimensionality of tangent space, DOF = 3
    inline size_t dim() const { return dimension; }

  	/// Updates a with tangent space delta
  	inline Point3 retract(const Vector& v) const { return compose(Expmap(v)); }

    /// MATLAB version returns shared pointer
    boost::shared_ptr<Point3> retract_(const Vector& v) {
      return boost::shared_ptr<Point3>(new Point3(retract(v)));
    }

  	/// Returns inverse retraction
  	inline Vector localCoordinates(const Point3& t2) const { return Logmap(t2) - Logmap(*this); }

    /// @}
    /// @name Lie Group
    /// @{

    /** Exponential map at identity - just create a Point3 from x,y,z */
    static inline Point3 Expmap(const Vector& v) { return Point3(v); }

    /** Log map at identity - return the x,y,z of this point */
    static inline Vector Logmap(const Point3& dp) { return Vector_(3, dp.x(), dp.y(), dp.z()); }

    /// @}
    /// @name Vector Operators
    /// @{

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

    /** dot product */
    double norm() const;

    /** cross product @return this x q */
    Point3 cross(const Point3 &q) const;

    /** dot product @return this * q*/
    double dot(const Point3 &q) const;

    /// @}

    /** Between using the default implementation */
    inline Point3 between(const Point3& p2,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const {
      if(H1) *H1 = -eye(3);
      if(H2) *H2 = eye(3);
      return p2 - *this;
    }

  	/// MATLAB version returns shared pointer
  	boost::shared_ptr<Point3> between_(const Point3& p2) {
  		return boost::shared_ptr<Point3>(new Point3(between(p2)));
  	}

    /** return vectorized form (column-wise)*/
    Vector vector() const {
      Vector v(3); v(0)=x_; v(1)=y_; v(2)=z_;
      return v;
    }

    /** get functions for x, y, z */
    inline double x() const {return x_;}
    inline double y() const {return y_;}
    inline double z() const {return z_;}

    /** add two points, add(this,q) is same as this + q */
    Point3 add (const Point3 &q,
  	      boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

    /** subtract two points, sub(this,q) is same as this - q */
    Point3 sub (const Point3 &q,
  	      boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(x_);
      ar & BOOST_SERIALIZATION_NVP(y_);
      ar & BOOST_SERIALIZATION_NVP(z_);
    }
  };

  /// Syntactic sugar for multiplying coordinates by a scalar s*p
  inline Point3 operator*(double s, const Point3& p) { return p*s;}

}
