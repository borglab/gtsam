/* ----------------------------------------------------------------------------

 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file OrientedPlane3.h
 * @date Dec 19, 2013
 * @author Alex Trevor
 * @author Frank Dellaert
 * @author Zhaoyang Lv
 * @brief An infinite plane, represented by a normal direction and perpendicular distance
 */

#pragma once

#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Pose3.h>
#include <string>

namespace gtsam {

/**
 * @brief Represents an infinite plane in 3D, which is composed of a planar
 * normal and its perpendicular distance to the origin.
 * Currently it provides a transform of the plane, and a norm 1 differencing of
 * two planes.
 * Refer to Trevor12iros for more math details.
 */
class GTSAM_EXPORT OrientedPlane3 {
private:
  Unit3 n_;     ///< The direction of the planar normal
  double d_;    ///< The perpendicular distance to this plane

public:
  enum {
    dimension = 3
  };

  /// @name Constructors
  /// @{

  /// Default constructor
  OrientedPlane3() :
    n_(), d_(0.0) {
  }

  /// Construct from a Unit3 and a distance
  OrientedPlane3(const Unit3& n, double d) :
    n_(n), d_(d) {
  }

  /// Construct from a vector of plane coefficients
  explicit OrientedPlane3(const Vector4& vec)
      : n_(vec(0), vec(1), vec(2)), d_(vec(3)) {}

  /// Construct from four numbers of plane coeffcients (a, b, c, d)
  OrientedPlane3(double a, double b, double c, double d) {
    n_ = Unit3(a, b, c);
    d_ = d;
  }

  /// @}
  /// @name Testable
  /// @{

  /// The print function
  void print(const std::string& s = std::string()) const;

  /// The equals function with tolerance
  bool equals(const OrientedPlane3& s, double tol = 1e-9) const {
    return (n_.equals(s.n_, tol) && (std::abs(d_ - s.d_) < tol));
  }

  /// @}

  /** Transforms a plane to the specified pose
   * @param xr a transformation in current coordiante
   * @param Hp optional Jacobian wrpt the destination plane
   * @param Hr optional jacobian wrpt the pose transformation
   * @return the transformed plane
   */
  OrientedPlane3 transform(const Pose3& xr,
                           OptionalJacobian<3, 3> Hp = boost::none,
                           OptionalJacobian<3, 6> Hr = boost::none) const;

  /** Computes the error between the two planes, with derivatives.
   *  This uses Unit3::errorVector, as opposed to the other .error() in this
   *  class, which uses Unit3::localCoordinates. This one has correct
   *  derivatives.
   *  NOTE(hayk): The derivatives are zero when normals are exactly orthogonal.
   * @param other the other plane
   */
  Vector3 errorVector(const OrientedPlane3& other,
                      OptionalJacobian<3, 3> H1 = boost::none,
                      OptionalJacobian<3, 3> H2 = boost::none) const;

  /// Dimensionality of tangent space = 3 DOF
  inline static size_t Dim() {
    return 3;
  }

  /// Dimensionality of tangent space = 3 DOF
  inline size_t dim() const {
    return 3;
  }

  /// The retract function
  OrientedPlane3 retract(const Vector3& v,
                        OptionalJacobian<3, 3> H = boost::none) const;

  /// The local coordinates function
  Vector3 localCoordinates(const OrientedPlane3& s) const;

  /// Returns the plane coefficients
  inline Vector4 planeCoefficients() const {
    Vector3 unit_vec = n_.unitVector();
    return Vector4(unit_vec[0], unit_vec[1], unit_vec[2], d_);
  }

  /// Return the normal
  inline Unit3 normal() const {
    return n_;
  }

  /// Return the perpendicular distance to the origin
  inline double distance() const {
    return d_;
  }
};

template<> struct traits<OrientedPlane3> : public internal::Manifold<
OrientedPlane3> {
};

template<> struct traits<const OrientedPlane3> : public internal::Manifold<
OrientedPlane3> {
};

}  // namespace gtsam

