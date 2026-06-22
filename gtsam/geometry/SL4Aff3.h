/**
 * @file  Aff3.h
 * @brief Affine Group (Aff(3, R)) factor
 * @author: Emmanuel Larralde
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/MatrixLieGroup.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/config.h>
#include <gtsam/dllexport.h>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <gtsam/base/MatrixSerialization.h>
#endif

#include <string>

using Aff3Jacobian = gtsam::OptionalJacobian<12, 12>;

using Matrix12x12 = Eigen::Matrix<double, 12, 12>;
using Matrix16x16 = Eigen::Matrix<double, 16, 16>;

namespace gtsam {
// NOTE(hlim): Strictly speaking, it should be expressed as Aff(3, ℝ),
// but for simplicity, we omit ℝ, assuming our target is over the real numbers.
// And the variable `aff3` represents Aff(3, ℝ).
class GTSAM_EXPORT Aff3 : public MatrixLieGroup<Aff3, 12, 4> {
 public:
  static const size_t dimension = 12;

 protected:
  Matrix44 T_;

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor initializes at origin
  Aff3() : T_(Matrix44::Identity()) {}

  /// Copy constructor
  Aff3(const Matrix44& pose);

  Aff3(const Aff3& pose) = default;

  Aff3& operator=(const Aff3& pose) = default;

  /** print with optional string */
  void print(const std::string& s = "") const;

  /** assert equality up to a tolerance */
  bool equals(const Aff3& aff3, double tol = 1e-9) const;

  /** convert to 4*4 matrix */
  inline const Matrix44& matrix() const { return T_; }

  /// @}
  /// @name Group
  /// @{

  /// identity for group operation
  static Aff3 Identity() { return Aff3(); }

  /// inverse transformation
  Aff3 inverse() const { return Aff3(T_.inverse()); }

  /// Group operation
  Aff3 operator*(const Aff3& other) const { return Aff3(T_ * other.T_); }

  /// @}
  /// @name Lie Group
  /// @{

  // compose and between provided by LieGroup

  /// Adjoint representation of the tangent space
  Matrix12x12 AdjointMap() const;

  /// Version with derivative version added by LieGroup
  using LieGroup<Aff3, 12>::inverse;

  /// Exponential map at identity - create an element from canonical coordinates
  static Aff3 Expmap(const Vector& xi, Aff3Jacobian H = {});

  /// Log map at identity - return the canonical coordinates of this element
  static Vector Logmap(const Aff3& p, Aff3Jacobian H = {});

  // Chart at origin
  struct GTSAM_EXPORT ChartAtOrigin {
    static Aff3 Retract(const Vector12& xi, ChartJacobian Hxi = {});
    static Vector12 Local(const Aff3& pose, ChartJacobian Hpose = {});
  };

  // retract and localCoordinates provided by LieGroup

  /// @}
  /// @name Matrix Lie Group
  /// @{

  using LieAlgebra = Matrix44;
  
  static Matrix44 Hat(const Vector& xi);
  static Vector Vee(const Matrix44& X);

  /// @}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(T_);
  }
#endif
};  // \class Aff3

template <>
struct traits<Aff3> : public internal::MatrixLieGroup<Aff3, 4> {};

template <>
struct traits<const Aff3> : public internal::MatrixLieGroup<Aff3, 4> {};

}  // namespace gtsam
