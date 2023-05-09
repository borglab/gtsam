/* ----------------------------------------------------------------------------

 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file Unit3.h
 * @date Feb 02, 2011
 * @author Can Erdogan
 * @author Frank Dellaert
 * @author Alex Trevor
 * @brief Develop a Unit3 class - basically a point on a unit sphere
 */

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/dllexport.h>

#include <boost/optional.hpp>
#include <boost/serialization/nvp.hpp>

#include <random>
#include <string>

#ifdef GTSAM_USE_TBB
#include <mutex> // std::mutex
#endif

namespace gtsam {

/// Represents a 3D point on a unit sphere.
class Unit3 {

private:

  Vector3 p_; ///< The location of the point on the unit sphere
  mutable boost::optional<Matrix32> B_; ///< Cached basis
  mutable boost::optional<Matrix62> H_B_; ///< Cached basis derivative

#ifdef GTSAM_USE_TBB
  mutable std::mutex B_mutex_; ///< Mutex to protect the cached basis.
#endif

public:

  enum {
    dimension = 2
  };

  /// @name Constructors
  /// @{

  /// Default constructor
  Unit3() :
      p_(1.0, 0.0, 0.0) {
  }

  /// Construct from point
  explicit Unit3(const Vector3& p) :
      p_(p.normalized()) {
  }

  /// Construct from x,y,z
  Unit3(double x, double y, double z) :
      p_(x, y, z) {
    p_.normalize();
  }

  /// Construct from 2D point in plane at focal length f
  /// Unit3(p,1) can be viewed as normalized homogeneous coordinates of 2D point
  explicit Unit3(const Point2& p, double f) : p_(p.x(), p.y(), f) {
    p_.normalize();
  }

  /// Copy constructor
  Unit3(const Unit3& u) {
    p_ = u.p_;
  }

  /// Copy assignment
  Unit3& operator=(const Unit3 & u) {
    p_ = u.p_;
    B_ = u.B_;
    H_B_ = u.H_B_;
    return *this;
  }

  /// Named constructor from Point3 with optional Jacobian
  GTSAM_EXPORT static Unit3 FromPoint3(const Point3& point, //
      OptionalJacobian<2, 3> H = boost::none);

  /**
   * Random direction, using boost::uniform_on_sphere
   * Example:
   *   std::mt19937 engine(42);
   *   Unit3 unit = Unit3::Random(engine);
   */
  GTSAM_EXPORT static Unit3 Random(std::mt19937 & rng);

  /// @}

  /// @name Testable
  /// @{

  friend std::ostream& operator<<(std::ostream& os, const Unit3& pair);

  /// The print fuction
  GTSAM_EXPORT void print(const std::string& s = std::string()) const;

  /// The equals function with tolerance
  bool equals(const Unit3& s, double tol = 1e-9) const {
    return equal_with_abs_tol(p_, s.p_, tol);
  }
  /// @}

  /// @name Other functionality
  /// @{

  /**
   * Returns the local coordinate frame to tangent plane
   * It is a 3*2 matrix [b1 b2] composed of two orthogonal directions
   * tangent to the sphere at the current direction.
   * Provides derivatives of the basis with the two basis vectors stacked up as a 6x1.
   */
  GTSAM_EXPORT const Matrix32& basis(OptionalJacobian<6, 2> H = boost::none) const;

  /// Return skew-symmetric associated with 3D point on unit sphere
  GTSAM_EXPORT Matrix3 skew() const;

  /// Return unit-norm Point3
  GTSAM_EXPORT Point3 point3(OptionalJacobian<3, 2> H = boost::none) const;

  /// Return unit-norm Vector
  GTSAM_EXPORT Vector3 unitVector(OptionalJacobian<3, 2> H = boost::none) const;

  /// Return scaled direction as Point3
  friend Point3 operator*(double s, const Unit3& d) {
    return Point3(s * d.p_);
  }

  /// Return dot product with q
  GTSAM_EXPORT double dot(const Unit3& q, OptionalJacobian<1,2> H1 = boost::none, //
                             OptionalJacobian<1,2> H2 = boost::none) const;

  /// Signed, vector-valued error between two directions
  /// @deprecated, errorVector has the proper derivatives, this confusingly has only the second.
  GTSAM_EXPORT Vector2 error(const Unit3& q, OptionalJacobian<2, 2> H_q = boost::none) const;

  /// Signed, vector-valued error between two directions
  /// NOTE(hayk): This method has zero derivatives if this (p) and q are orthogonal.
  GTSAM_EXPORT Vector2 errorVector(const Unit3& q, OptionalJacobian<2, 2> H_p = boost::none, //
                      OptionalJacobian<2, 2> H_q = boost::none) const;

  /// Distance between two directions
  GTSAM_EXPORT double distance(const Unit3& q, OptionalJacobian<1, 2> H = boost::none) const;

  /// Cross-product between two Unit3s
  Unit3 cross(const Unit3& q) const {
    return Unit3(p_.cross(q.p_));
  }

  /// Cross-product w Point3
  Point3 cross(const Point3& q) const {
    return point3().cross(q);
  }

  /// @}

  /// @name Manifold
  /// @{

  /// Dimensionality of tangent space = 2 DOF
  inline static size_t Dim() {
    return 2;
  }

  /// Dimensionality of tangent space = 2 DOF
  inline size_t dim() const {
    return 2;
  }

  enum CoordinatesMode {
    EXPMAP, ///< Use the exponential map to retract
    RENORM ///< Retract with vector addition and renormalize.
  };

  /// The retract function
  GTSAM_EXPORT Unit3 retract(const Vector2& v, OptionalJacobian<2,2> H = boost::none) const;

  /// The local coordinates function
  GTSAM_EXPORT Vector2 localCoordinates(const Unit3& s) const;

  /// @}

private:

  /// @name Advanced Interface
  /// @{
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(p_);
  }

  /// @}

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

// Define GTSAM traits
template<> struct traits<Unit3> : public internal::Manifold<Unit3> {
};

template<> struct traits<const Unit3> : public internal::Manifold<Unit3> {
};

} // namespace gtsam

