/**
 * @file   Point4.h
 * @brief  gnss state vector -- {x,y,z,cb}
 * @author Ryan Watson
 */

// \callgraph

#pragma once
#include <iostream>

#include <gtsam/config.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/dllexport.h>
#include <boost/serialization/nvp.hpp>
#include <gtsam/geometry/Point3.h>

namespace gtsam {

#ifdef GTSAM_TYPEDEF_POINTS_TO_VECTORS

  /// As of GTSAM 4, in order to make GTSAM more lean,
  /// it is now possible to just typedef Point3 to Vector3
  typedef Vector4 Point4;

#else

/**
   * A 4D point is just a Vector4 with some additional methods
   * @addtogroup geometry
   * \nosubgrouping
   */
class GTSAM_EXPORT Point4 : public Vector4 {

  public:

    enum { dimension = 4 };

    /// @name Standard Constructors
    /// @{

    // Deprecated default constructor initializes to zero, in contrast to new behavior below
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
    Point4() { setZero(); }
#endif

    using Vector4::Vector4;

    /// @}
    /// @name Testable
    /// @{

    /** print with optional string */
    void print(const std::string& s = "") const;

    /** equals with an tolerance */
    bool equals(const Point4& p, double tol = 1e-9) const;

    /// @}
    /// @name Group
    /// @{

    /// identity for group operation
    inline static Point4 identity() { return Point4(0.0, 0.0, 0.0, 0.0); }

    /// @}
    /// @name Vector Space
    /// @{

    /** distance between two points */
    double distance(const Point4& p2, OptionalJacobian<1, 4> H1 = boost::none,
                    OptionalJacobian<1, 4> H2 = boost::none) const;

    /** Distance of the point from the origin, with Jacobian */
    double norm(OptionalJacobian<1,4> H = boost::none) const;


    /** dot product @return this * q*/
    double dot(const Point4 &q, OptionalJacobian<1, 4> H_p = boost::none, //
                                OptionalJacobian<1, 4> H_q = boost::none) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /// return as Vector4
    const Vector4& vector() const { return *this; }

    /// get x
    inline double x() const {return (*this)[0];}

    /// get y
    inline double y() const {return (*this)[1];}

    /// get z
    inline double z() const {return (*this)[2];}

    /// get cb
    inline double cb() const {return (*this)[3];}
    /// @}

    /// Output stream operator
    GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const Point4& p);

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
    /// @name Deprecated
    /// @{
    Point4 inverse() const { return -(*this);}
    Point4 compose(const Point4& q) const { return (*this)+q;}
    Point4 between(const Point4& q) const { return q-(*this);}
    Vector4 localCoordinates(const Point4& q) const { return between(q);}
    Point4 retract(const Vector4& v) const { return compose(Point4(v));}
    static Vector4 Logmap(const Point4& p) { return p;}
    static Point4 Expmap(const Vector4& v) { return Point4(v);}
    inline double dist(const Point4& q) const { return (q - *this).norm(); }
    Point4 add(const Point4& q, OptionalJacobian<4, 4> H1 = boost::none,
               OptionalJacobian<4, 4> H2 = boost::none) const;
    Point4 sub(const Point4& q, OptionalJacobian<4, 4> H1 = boost::none,
               OptionalJacobian<4, 4> H2 = boost::none) const;
  /// @}
#endif

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Vector4);
    }
  };

template<>
struct traits<Point4> : public internal::VectorSpace<Point4> {};

template<>
struct traits<const Point4> : public internal::VectorSpace<Point4> {};

#endif // GTSAM_TYPEDEF_POINTS_TO_VECTORS

// Convenience typedef
typedef std::pair<Point4, Point4> Point4Pair;
std::ostream &operator<<(std::ostream &os, const gtsam::Point4Pair &p);

/// distance between two points
double distance4(const Point4& p1, const Point4& q,
                 OptionalJacobian<1, 4> H1 = boost::none,
                 OptionalJacobian<1, 4> H2 = boost::none);


/// computed pseudorange
double geoDist(const Point3& p1, const Point3& p2,
                 OptionalJacobian<1, 4> H1 = boost::none,
                 OptionalJacobian<1, 4> H2 = boost::none);


/// Distance of the point from the origin, with Jacobian
double norm4(const Point4& p, OptionalJacobian<1, 4> H = boost::none);

/// dot product
double dot(const Point4& p, const Point4& q,
           OptionalJacobian<1, 4> H_p = boost::none,
           OptionalJacobian<1, 4> H_q = boost::none);

template <typename A1, typename A2>
struct Range;

template <>
struct Range<Point4, Point4> {
  typedef double result_type;
  double operator()(const Point4& p, const Point4& q,
                    OptionalJacobian<1, 4> H1 = boost::none,
                    OptionalJacobian<1, 4> H2 = boost::none) {
    return distance4(p, q, H1, H2);
  }
};

}  // namespace gtsam

