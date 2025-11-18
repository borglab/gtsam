/**
 * @file  SL4.h
 * @brief Projective Special Linear Group (SL(4, R)) factor
 * @author: Hyungtae Lim
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

using SL4Jacobian = gtsam::OptionalJacobian<15, 15>;

using Matrix15x15 = Eigen::Matrix<double, 15, 15>;
using Matrix16x16 = Eigen::Matrix<double, 16, 16>;

namespace gtsam {
// NOTE(hlim): Strictly speaking, it should be expressed as SL(4, ℝ),
// but for simplicity, we omit ℝ, assuming our target is over the real numbers.
// And the variable `sl4` represents SL(4, ℝ).
class GTSAM_EXPORT SL4 : public MatrixLieGroup<SL4, 15, 4> {
 public:
  static const size_t dimension = 15;

 protected:
  Matrix44 T_;

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor initializes at origin
  SL4() : T_(Matrix44::Identity()) {}

  /// Copy constructor
  SL4(const Matrix44& pose);

  SL4(const SL4& pose) = default;

  SL4& operator=(const SL4& pose) = default;

  /** print with optional string */
  void print(const std::string& s = "") const;

  /** assert equality up to a tolerance */
  bool equals(const SL4& sl4, double tol = 1e-9) const;

  /** convert to 4*4 matrix */
  inline const Matrix44& matrix() const { return T_; }

  /// @}
  /// @name Group
  /// @{

  /// identity for group operation
  static SL4 Identity() { return SL4(); }

  /// inverse transformation
  SL4 inverse() const { return SL4(T_.inverse()); }

  /// Group operation
  SL4 operator*(const SL4& other) const { return SL4(T_ * other.T_); }

  /// @}
  /// @name Lie Group
  /// @{

  // compose and between provided by LieGroup

  /// Adjoint representation of the tangent space
  Matrix15x15 AdjointMap() const;

  /// Version with derivative version added by LieGroup
  using LieGroup<SL4, 15>::inverse;

  /// Exponential map at identity - create an element from canonical coordinates
  static SL4 Expmap(const Vector& xi, SL4Jacobian H = {});

  /// Log map at identity - return the canonical coordinates of this element
  static Vector Logmap(const SL4& p, SL4Jacobian H = {});

  // Chart at origin
  struct GTSAM_EXPORT ChartAtOrigin {
    static SL4 Retract(const Vector15& xi, ChartJacobian Hxi = {});
    static Vector15 Local(const SL4& pose, ChartJacobian Hpose = {});
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
};  // \class SL4

template <>
struct traits<SL4> : public internal::MatrixLieGroup<SL4, 4> {};

template <>
struct traits<const SL4> : public internal::MatrixLieGroup<SL4, 4> {};

}  // namespace gtsam
