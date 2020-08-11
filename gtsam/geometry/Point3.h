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

#include <gtsam/config.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <boost/serialization/nvp.hpp>

namespace gtsam {

#ifdef GTSAM_TYPEDEF_POINTS_TO_VECTORS

  /// As of GTSAM 4, in order to make GTSAM more lean,
  /// it is now possible to just typedef Point3 to Vector3
  typedef Vector3 Point3;

#else

/**
   * A 3D point is just a Vector3 with some additional methods
   * @addtogroup geometry
   * \nosubgrouping
   */
class Point3 : public Vector3 {

  public:

    enum { dimension = 3 };

    /// @name Standard Constructors
    /// @{

    using Vector3::Vector3;

    /// @}
    /// @name Testable
    /// @{

    /** print with optional string */
	GTSAM_EXPORT void print(const std::string& s = "") const;

    /** equals with an tolerance */
	GTSAM_EXPORT bool equals(const Point3& p, double tol = 1e-9) const;

    /// @}
    /// @name Group
    /// @{

    /// identity for group operation
    inline static Point3 identity() { return Point3(0.0, 0.0, 0.0); }

    /// @}
    /// @name Vector Space
    /// @{

    /** distance between two points */
	GTSAM_EXPORT double distance(const Point3& p2, OptionalJacobian<1, 3> H1 = boost::none,
                    OptionalJacobian<1, 3> H2 = boost::none) const;

    /** Distance of the point from the origin, with Jacobian */
	GTSAM_EXPORT double norm(OptionalJacobian<1,3> H = boost::none) const;

    /** normalize, with optional Jacobian */
	GTSAM_EXPORT Point3 normalized(OptionalJacobian<3, 3> H = boost::none) const;

    /** cross product @return this x q */
	GTSAM_EXPORT Point3 cross(const Point3 &q, OptionalJacobian<3, 3> H_p = boost::none, //
                                  OptionalJacobian<3, 3> H_q = boost::none) const;

    /** dot product @return this * q*/
	GTSAM_EXPORT double dot(const Point3 &q, OptionalJacobian<1, 3> H_p = boost::none, //
                                OptionalJacobian<1, 3> H_q = boost::none) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /// return as Vector3
    const Vector3& vector() const { return *this; }

    /// get x
    inline double x() const {return (*this)[0];}

    /// get y
    inline double y() const {return (*this)[1];}

    /// get z
    inline double z() const {return (*this)[2];}

    /// @}

    /// Output stream operator
    GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const Point3& p);

   private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Vector3);
    }
  };

template<>
struct traits<Point3> : public internal::VectorSpace<Point3> {};

template<>
struct traits<const Point3> : public internal::VectorSpace<Point3> {};

#endif // GTSAM_TYPEDEF_POINTS_TO_VECTORS

// Convenience typedef
typedef std::pair<Point3, Point3> Point3Pair;
GTSAM_EXPORT std::ostream &operator<<(std::ostream &os, const gtsam::Point3Pair &p);

/// distance between two points
GTSAM_EXPORT double distance3(const Point3& p1, const Point3& q,
	                          OptionalJacobian<1, 3> H1 = boost::none,
                              OptionalJacobian<1, 3> H2 = boost::none);

/// Distance of the point from the origin, with Jacobian
GTSAM_EXPORT double norm3(const Point3& p, OptionalJacobian<1, 3> H = boost::none);

/// normalize, with optional Jacobian
GTSAM_EXPORT Point3 normalize(const Point3& p, OptionalJacobian<3, 3> H = boost::none);

/// cross product @return this x q
GTSAM_EXPORT Point3 cross(const Point3& p, const Point3& q,
                          OptionalJacobian<3, 3> H_p = boost::none,
                          OptionalJacobian<3, 3> H_q = boost::none);

/// dot product
GTSAM_EXPORT double dot(const Point3& p, const Point3& q,
                        OptionalJacobian<1, 3> H_p = boost::none,
                        OptionalJacobian<1, 3> H_q = boost::none);

template <typename A1, typename A2>
struct Range;

template <>
struct Range<Point3, Point3> {
  typedef double result_type;
  double operator()(const Point3& p, const Point3& q,
                    OptionalJacobian<1, 3> H1 = boost::none,
                    OptionalJacobian<1, 3> H2 = boost::none) {
    return distance3(p, q, H1, H2);
  }
};

}  // namespace gtsam

