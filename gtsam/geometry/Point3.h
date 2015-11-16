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

#include <gtsam/base/VectorSpace.h>
#include <gtsam/dllexport.h>
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
    enum { dimension = 3 };

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
    explicit Point3(const Vector3& v) {
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
    inline static Point3 identity() { return Point3();}

    /// inverse
    Point3 operator - () const { return Point3(-x_,-y_,-z_);}

    /// add vector on right
    inline Point3 operator +(const Vector3& v) const {
      return Point3(x_ + v[0], y_ + v[1], z_ + v[2]);
    }

    /// add
    Point3 operator + (const Point3& q) const;

    /// subtract
    Point3 operator - (const Point3& q) const;

    /// @}
    /// @name Vector Space
    /// @{

    ///multiply with a scalar
    Point3 operator * (double s) const;

    ///divide by a scalar
    Point3 operator / (double s) const;

    /** distance between two points */
    double distance(const Point3& p2, OptionalJacobian<1, 3> H1 = boost::none,
                    OptionalJacobian<1, 3> H2 = boost::none) const;

    /** @deprecated The following function has been deprecated, use distance above */
    inline double dist(const Point3& p2) const {
      return (p2 - *this).norm();
    }

    /** Distance of the point from the origin, with Jacobian */
    double norm(OptionalJacobian<1,3> H = boost::none) const;

    /** normalize, with optional Jacobian */
    Point3 normalize(OptionalJacobian<3, 3> H = boost::none) const;

    /** cross product @return this x q */
    Point3 cross(const Point3 &q, OptionalJacobian<3, 3> H_p = boost::none, //
                                  OptionalJacobian<3, 3> H_q = boost::none) const;

    /** dot product @return this * q*/
    double dot(const Point3 &q, OptionalJacobian<1, 3> H_p = boost::none, //
                                OptionalJacobian<1, 3> H_q = boost::none) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /// equality
    bool operator ==(const Point3& q) const;

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

    /// @name Deprecated
    /// @{
    Point3 inverse() const { return -(*this);}
    Point3 compose(const Point3& q) const { return (*this)+q;}
    Point3 between(const Point3& q) const { return q-(*this);}
    Vector3 localCoordinates(const Point3& q) const { return between(q).vector();}
    Point3 retract(const Vector3& v) const { return compose(Point3(v));}
    static Vector3 Logmap(const Point3& p) { return p.vector();}
    static Point3 Expmap(const Vector3& v) { return Point3(v);}
    /// @}

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
      ar & BOOST_SERIALIZATION_NVP(z_);
    }

    /// @}
  };

// Convenience typedef
typedef std::pair<Point3, Point3> Point3Pair;
std::ostream &operator<<(std::ostream &os, const gtsam::Point3Pair &p);

/// Syntactic sugar for multiplying coordinates by a scalar s*p
inline Point3 operator*(double s, const Point3& p) { return p*s;}

template<>
struct traits<Point3> : public internal::VectorSpace<Point3> {};

template<>
struct traits<const Point3> : public internal::VectorSpace<Point3> {};

template <typename A1, typename A2>
struct Range;

template <>
struct Range<Point3, Point3> {
  typedef double result_type;
  double operator()(const Point3& p, const Point3& q,
                    OptionalJacobian<1, 3> H1 = boost::none,
                    OptionalJacobian<1, 3> H2 = boost::none) {
    return p.distance(q, H1, H2);
  }
};

}  // namespace gtsam

