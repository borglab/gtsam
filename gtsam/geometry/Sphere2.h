/* ----------------------------------------------------------------------------

 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file Sphere2.h
 * @date Feb 02, 2011
 * @author Can Erdogan
 * @brief Develop a Sphere2 class - basically a point on a unit sphere
 */

#pragma once

#include <gtsam/geometry/Point3.h>

namespace gtsam {

/// Represents a 3D point on a unit sphere.
class Sphere2 {

private:

  Point3 p_; ///< The location of the point on the unit sphere

  /// Returns the axis of rotations
  Matrix getBasis(Vector* axisOutput = NULL) const;

public:

  /// The constructors
  Sphere2() :
      p_(Point3(1.0, 0.0, 0.0)) {
  }

  /// Copy constructor
  Sphere2(const Sphere2& s) {
    p_ = s.p_ / s.p_.norm();
  }

  /// Destructor
  ~Sphere2() {
  }

  /// Field constructor
  Sphere2(const Point3& p) {
    p_ = p / p.norm();
  }

  /// @name Testable
  /// @{

  /// The print fuction
  void print(const std::string& s = std::string()) const;

  /// The equals function with tolerance
  bool equals(const Sphere2& s, double tol = 1e-9) const {
    return p_.equals(s.p_, tol);
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

  /// The retract function
  Sphere2 retract(const Vector& v) const;

  /// The local coordinates function
  Vector localCoordinates(const Sphere2& s) const;

  /// @}

};

} // namespace gtsam

