/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Similarity3.h
 * @brief  Implementation of Similarity3 transform
 * @author Paul Drews
 */

#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Manifold.h>

namespace gtsam {

// Forward declarations
class Pose3;

/**
 * 3D similarity transform
 */
class Similarity3: public LieGroup<Similarity3, 7> {

  /// @name Pose Concept
  /// @{
  typedef Rot3 Rotation;
  typedef Point3 Translation;
  /// @}

private:
  Rot3 R_;
  Point3 t_;
  double s_;

public:

  /// @name Constructors
  /// @{

  /// Default constructor
  Similarity3();

  /// Construct pure scaling
  Similarity3(double s);

  /// Construct from GTSAM types
  Similarity3(const Rot3& R, const Point3& t, double s);

  /// Construct from Eigen types
  Similarity3(const Matrix3& R, const Vector3& t, double s);

  /// Construct from matrix [R t; 0 s^-1]
  Similarity3(const Matrix4& T);

  /// @}
  /// @name Testable
  /// @{

  /// Compare with tolerance
  bool equals(const Similarity3& sim, double tol) const;

  /// Exact equality
  bool operator==(const Similarity3& other) const;

  /// Print with optional string
  void print(const std::string& s) const;

  GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const Similarity3& p);

  /// @}
  /// @name Group
  /// @{

  /// Return an identity transform
  static Similarity3 identity();

  /// Composition
  Similarity3 operator*(const Similarity3& T) const;

  /// Return the inverse
  Similarity3 inverse() const;

  /// @}
  /// @name Group action on Point3
  /// @{

  /// Action on a point p is s*(R*p+t)
  Point3 transform_from(const Point3& p, //
      OptionalJacobian<3, 7> H1 = boost::none, //
      OptionalJacobian<3, 3> H2 = boost::none) const;

  /** syntactic sugar for transform_from */
  Point3 operator*(const Point3& p) const;

  /// @}
  /// @name Lie Group
  /// @{

  /** Log map at the identity
   * \f$ [R_x,R_y,R_z, t_x, t_y, t_z, \lambda] \f$
   */
  static Vector7 Logmap(const Similarity3& s, //
      OptionalJacobian<7, 7> Hm = boost::none);

  /** Exponential map at the identity
   */
  static Similarity3 Expmap(const Vector7& v, //
      OptionalJacobian<7, 7> Hm = boost::none);

  /// Chart at the origin
  struct ChartAtOrigin {
    static Similarity3 Retract(const Vector7& v, ChartJacobian H = boost::none) {
      return Similarity3::Expmap(v, H);
    }
    static Vector7 Local(const Similarity3& other, ChartJacobian H = boost::none) {
      return Similarity3::Logmap(other, H);
    }
  };

  using LieGroup<Similarity3, 7>::inverse;

  /**
   * wedge for Similarity3:
   * @param xi 7-dim twist (w,u,lambda) where
   * @return 4*4 element of Lie algebra that can be exponentiated
   * TODO(frank): rename to Hat, make part of traits
   */
  static Matrix4 wedge(const Vector7& xi);

  /// Project from one tangent space to another
  Matrix7 AdjointMap() const;

  /// @}
  /// @name Standard interface
  /// @{

  /// Calculate 4*4 matrix group equivalent
  const Matrix4 matrix() const;

  /// Return a GTSAM rotation
  const Rot3& rotation() const {
    return R_;
  }

  /// Return a GTSAM translation
  const Point3& translation() const {
    return t_;
  }

  /// Return the scale
  double scale() const {
    return s_;
  }

  /// Convert to a rigid body pose (R, s*t)
  /// TODO(frank): why is this here? Red flag! Definitely don't have it as a cast.
  operator Pose3() const;

  /// Dimensionality of tangent space = 7 DOF - used to autodetect sizes
  inline static size_t Dim() {
    return 7;
  }

  /// Dimensionality of tangent space = 7 DOF
  inline size_t dim() const {
    return 7;
  }

  /// @}
  /// @name Helper functions
  /// @{

  /// Calculate expmap and logmap coefficients.
private:
  static Matrix3 GetV(Vector3 w, double lambda);

  /// @}
};

template<>
inline Matrix wedge<Similarity3>(const Vector& xi) {
  return Similarity3::wedge(xi);
}

template<>
struct traits<Similarity3> : public internal::LieGroup<Similarity3> {};

template<>
struct traits<const Similarity3> : public internal::LieGroup<Similarity3> {};

} // namespace gtsam
