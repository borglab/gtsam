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
#include <gtsam/base/Vector.h>
#include <gtsam/base/VectorSerialization.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/dllexport.h>


#include <random>
#include <string>

#ifdef GTSAM_USE_TBB
#include <mutex> // std::mutex
#endif

namespace gtsam {

/// Represents a 3D point on a unit sphere.
class GTSAM_EXPORT Unit3 {

private:

  Vector3 p_; ///< The location of the point on the unit sphere
  mutable std::optional<Matrix32> B_; ///< Cached basis
  mutable std::optional<Matrix62> H_B_; ///< Cached basis derivative

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
  explicit Unit3(const Vector3& p);

  /// Construct from x,y,z
  Unit3(double x, double y, double z);

  /// Construct from 2D point in plane at focal length f
  /// Unit3(p,1) can be viewed as normalized homogeneous coordinates of 2D point
  explicit Unit3(const Point2& p, double f);

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
  static Unit3 FromPoint3(const Point3& point, //
      OptionalJacobian<2, 3> H = {});

  /**
   * Random direction, using boost::uniform_on_sphere
   * Example:
   *   std::mt19937 engine(42);
   *   Unit3 unit = Unit3::Random(engine);
   */
  static Unit3 Random(std::mt19937 & rng);

  /// @}

  /// @name Testable
  /// @{

  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Unit3& pair);

  /// The print fuction
  void print(const std::string& s = std::string()) const;

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
  const Matrix32& basis(OptionalJacobian<6, 2> H = {}) const;

  /// Return skew-symmetric associated with 3D point on unit sphere
  Matrix3 skew() const;

  /// Return unit-norm Point3
  Point3 point3(OptionalJacobian<3, 2> H = {}) const;

  /// Return unit-norm Vector
  Vector3 unitVector(OptionalJacobian<3, 2> H = {}) const;

  /// Return scaled direction as Point3
  friend Point3 operator*(double s, const Unit3& d) {
    return Point3(s * d.p_);
  }

  /// Return dot product with q
  double dot(const Unit3& q, OptionalJacobian<1,2> H1 = {}, //
                             OptionalJacobian<1,2> H2 = {}) const;

  /// Signed, vector-valued error between two directions
  /// @deprecated, errorVector has the proper derivatives, this confusingly has only the second.
  Vector2 error(const Unit3& q, OptionalJacobian<2, 2> H_q = {}) const;

  /// Signed, vector-valued error between two directions
  /// NOTE(hayk): This method has zero derivatives if this (p) and q are orthogonal.
  Vector2 errorVector(const Unit3& q, OptionalJacobian<2, 2> H_p = {}, //
                      OptionalJacobian<2, 2> H_q = {}) const;

  /// Distance between two directions
  double distance(const Unit3& q, OptionalJacobian<1, 2> H = {}) const;

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
  Unit3 retract(const Vector2& v, OptionalJacobian<2,2> H = {}) const;

  /// The local coordinates function
  Vector2 localCoordinates(const Unit3& s) const;

  /// @}

private:

  /// @name Advanced Interface
  /// @{
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(p_);
  }
#endif

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

