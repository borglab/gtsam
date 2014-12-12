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

#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/OptionalJacobian.h>

#include <boost/serialization/nvp.hpp>

#include <cmath>

namespace gtsam {

  /**
   * A 3D point
   * @addtogroup geometry
   * \nosubgrouping
   */
  class GTSAM_EXPORT Point3 {

  private:

    double x_, y_, z_;  
    
  public:

    /// @name Standard Constructors
    /// @{

    /// Default constructor creates a zero-Point3
    Point3(): x_(0), y_(0), z_(0) {}

    /// Construct from x, y, and z coordinates
    Point3(double x, double y, double z): x_(x), y_(y), z_(z) {}

    /// @}
    /// @name Advanced Constructors
    /// @{

    /// Construct from 3-element vector
    Point3(const Vector& v) {
      if(v.size() != 3)
        throw std::invalid_argument("Point3 constructor from Vector requires that the Vector have dimension 3");
      x_ = v(0);
      y_ = v(1);
      z_ = v(2);
    }

    /// @}
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

    /// syntactic sugar for inverse, i.e., -p == inverse(p)
    Point3 operator - () const { return Point3(-x_,-y_,-z_);}

    /// "Compose" - just adds coordinates of two points
    inline Point3 compose(const Point3& p2,
        OptionalJacobian<3,3> H1=boost::none,
        OptionalJacobian<3,3> H2=boost::none) const {
      if (H1) *H1 << I_3x3;
      if (H2) *H2 << I_3x3;
      return *this + p2;
    }

    ///syntactic sugar for adding two points, i.e., p+q == compose(p,q)
    Point3 operator + (const Point3& q) const;

    /** Between using the default implementation */
    inline Point3 between(const Point3& p2,
        OptionalJacobian<3,3> H1=boost::none,
        OptionalJacobian<3,3> H2=boost::none) const {
      if(H1) *H1 = -I_3x3;
      if(H2) *H2 = I_3x3;
      return p2 - *this;
    }

    /// syntactic sugar for subtracting points, i.e., q-p == between(p,q)
    Point3 operator - (const Point3& q) const;

    /// @}
    /// @name Manifold
    /// @{

    /// dimension of the variable - used to autodetect sizes
    inline static size_t Dim() { return 3; }

    /// return dimensionality of tangent space, DOF = 3
    inline size_t dim() const { return 3; }

    /// Updates a with tangent space delta
    inline Point3 retract(const Vector& v) const { return Point3(*this + v); }

    /// Returns inverse retraction
    inline Vector3 localCoordinates(const Point3& q) const { return (q -*this).vector(); }

    /// @}
    /// @name Lie Group
    /// @{

    /** Exponential map at identity - just create a Point3 from x,y,z */
    static inline Point3 Expmap(const Vector& v) { return Point3(v); }

    /** Log map at identity - return the x,y,z of this point */
    static inline Vector3 Logmap(const Point3& dp) { return Vector3(dp.x(), dp.y(), dp.z()); }

    /// Left-trivialized derivative of the exponential map
    static Matrix3 dexpL(const Vector& v) {
      return I_3x3;
    }

    /// Left-trivialized derivative inverse of the exponential map
    static Matrix3 dexpInvL(const Vector& v) {
      return I_3x3;
    }

    /// @}
    /// @name Vector Space
    /// @{

    ///multiply with a scalar
    Point3 operator * (double s) const;

    ///divide by a scalar
    Point3 operator / (double s) const;

    /** distance between two points */
    inline double distance(const Point3& p2,
        OptionalJacobian<1,3> H1 = boost::none, OptionalJacobian<1,3> H2 = boost::none) const {
      double d = (p2 - *this).norm();
      if (H1) {
        *H1 << x_-p2.x(), y_-p2.y(), z_-p2.z();
        *H1 = *H1 *(1./d);
      }

      if (H2) {
        *H2 << -x_+p2.x(), -y_+p2.y(), -z_+p2.z();
        *H2 << *H2 *(1./d);
      }
      return d;
    }

    /** @deprecated The following function has been deprecated, use distance above */
    inline double dist(const Point3& p2) const {
      return (p2 - *this).norm();
    }

    /** Distance of the point from the origin, with Jacobian */
    double norm(OptionalJacobian<1,3> H = boost::none) const;

    /** normalize, with optional Jacobian */
    Point3 normalize(OptionalJacobian<3, 3> H = boost::none) const;

    /** cross product @return this x q */
    Point3 cross(const Point3 &q) const;

    /** dot product @return this * q*/
    double dot(const Point3 &q) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /// equality
    bool   operator ==(const Point3& q) const;

    /** return vectorized form (column-wise)*/
    Vector3 vector() const { return Vector3(x_,y_,z_); }

    /// get x
    inline double x() const {return x_;}

    /// get y
    inline double y() const {return y_;}

    /// get z
    inline double z() const {return z_;}

    /** add two points, add(this,q) is same as this + q */
    Point3 add (const Point3 &q,
          OptionalJacobian<3, 3> H1=boost::none, OptionalJacobian<3, 3> H2=boost::none) const;

    /** subtract two points, sub(this,q) is same as this - q */
    Point3 sub (const Point3 &q,
          OptionalJacobian<3,3> H1=boost::none, OptionalJacobian<3,3> H2=boost::none) const;

    /// @}

    /// Output stream operator
    GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const Point3& p);

  private:

    /// @name Advanced Interface
    /// @{

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(x_);
      ar & BOOST_SERIALIZATION_NVP(y_);
      ar & BOOST_SERIALIZATION_NVP(z_);
    }

    /// @}

  };

  /// Syntactic sugar for multiplying coordinates by a scalar s*p
  inline Point3 operator*(double s, const Point3& p) { return p*s;}

  // Define GTSAM traits
  namespace traits {

  template<>
  struct GTSAM_EXPORT is_group<Point3> : public boost::true_type{
  };

  template<>
  struct GTSAM_EXPORT is_manifold<Point3> : public boost::true_type{
  };

  template<>
  struct GTSAM_EXPORT dimension<Point3> : public boost::integral_constant<int, 3>{
  };

  }
}
