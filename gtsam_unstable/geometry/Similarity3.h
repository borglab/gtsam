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

  /** Pose Concept requirements */
  typedef Rot3 Rotation;
  typedef Point3 Translation;

private:
  Rotation R_;
  Translation t_;
  double s_;

public:

  /// @name Constructors
  /// @{

  Similarity3();

  /// Construct pure scaling
  Similarity3(double s);

  /// Construct from GTSAM types
  Similarity3(const Rotation& R, const Translation& t, double s);

  /// Construct from Eigen types
  Similarity3(const Matrix3& R, const Vector3& t, double s);

  /// @}
  /// @name Testable
  /// @{

  /// Compare with tolerance
  bool equals(const Similarity3& sim, double tol) const;

  /// Compare with standard tolerance
  bool operator==(const Similarity3& other) const;

  /// Print with optional string
  void print(const std::string& s) const;

  GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const Similarity3& p);

  /// @}
  /// @name Group
  /// @{

  /// Return an identity transform
  static Similarity3 identity();

  /// Return the inverse
  Similarity3 inverse() const;

  Translation transform_from(const Translation& p) const;

  /** syntactic sugar for transform_from */
  inline Translation operator*(const Translation& p) const;

  Similarity3 operator*(const Similarity3& T) const;

  /// @}
  /// @name Standard interface
  /// @{

  /// Return a GTSAM rotation
  const Rotation& rotation() const {
    return R_;
  };

  /// Return a GTSAM translation
  const Translation& translation() const {
    return t_;
  };

  /// Return the scale
  double scale() const {
    return s_;
  };

  /// Convert to a rigid body pose
  operator Pose3() const;

  /// Dimensionality of tangent space = 7 DOF - used to autodetect sizes
  inline static size_t Dim() {
    return 7;
  };

  /// Dimensionality of tangent space = 7 DOF
  inline size_t dim() const {
    return 7;
  };

  /// @}
  /// @name Chart
  /// @{

  /// Update Similarity transform via 7-dim vector in tangent space
  struct ChartAtOrigin {
    static Similarity3 Retract(const Vector7& v,  ChartJacobian H = boost::none);

  /// 7-dimensional vector v in tangent space that makes other = this->retract(v)
  static Vector7 Local(const Similarity3& other,  ChartJacobian H = boost::none);
  };

  /// Project from one tangent space to another
  Matrix7 AdjointMap() const;

  /// @}
  /// @name Stubs
  /// @{

  /// Not currently implemented, required because this is a lie group
  static Vector7 Logmap(const Similarity3& s, OptionalJacobian<7, 7> Hm = boost::none);
  static Similarity3 Expmap(const Vector7& v, OptionalJacobian<7, 7> Hm = boost::none);

  using LieGroup<Similarity3, 7>::inverse; // version with derivative
};

template<>
struct traits<Similarity3> : public internal::LieGroup<Similarity3> {};
}
