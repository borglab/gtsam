/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO3.h
 * @brief   3*3 matrix representation of SO(3)
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @author  Duy Nguyen Ta
 * @date    December 2014
 */

#pragma once

#include <gtsam/geometry/SOn.h>

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/dllexport.h>

#include <vector>

namespace gtsam {

using SO3 = SO<3>;

// Below are all declarations of SO<3> specializations.
// They are *defined* in SO3.cpp.

template <>
GTSAM_EXPORT
SO3 SO3::AxisAngle(const Vector3& axis, double theta);

template <>
GTSAM_EXPORT
SO3 SO3::ClosestTo(const Matrix3& M);

template <>
GTSAM_EXPORT
SO3 SO3::ChordalMean(const std::vector<SO3>& rotations);

template <>
GTSAM_EXPORT
Matrix3 SO3::Hat(const Vector3& xi);  ///< make skew symmetric matrix

template <>
GTSAM_EXPORT
Vector3 SO3::Vee(const Matrix3& X);  ///< inverse of Hat

/// Adjoint map
template <>
inline Matrix3 SO3::AdjointMap() const{ return matrix_; }

/**
 * Exponential map at identity - create a rotation from canonical coordinates
 * \f$ [R_x,R_y,R_z] \f$ using Rodrigues' formula
 */
template <>
GTSAM_EXPORT
SO3 SO3::Expmap(const Vector3& omega, ChartJacobian H);

/// Derivative of Expmap
template <>
GTSAM_EXPORT
Matrix3 SO3::ExpmapDerivative(const Vector3& omega);

/**
 * Log map at identity - returns the canonical coordinates
 * \f$ [R_x,R_y,R_z] \f$ of this rotation
 */
template <>
GTSAM_EXPORT
Vector3 SO3::Logmap(const SO3& R, ChartJacobian H);

/// Derivative of Logmap
template <>
GTSAM_EXPORT
Matrix3 SO3::LogmapDerivative(const Vector3& omega);

// Chart at origin for SO3 is *not* Cayley but actual Expmap/Logmap
template <>
GTSAM_EXPORT
SO3 SO3::ChartAtOrigin::Retract(const Vector3& omega, ChartJacobian H);

template <>
GTSAM_EXPORT
Vector3 SO3::ChartAtOrigin::Local(const SO3& R, ChartJacobian H);

template <>
GTSAM_EXPORT
Vector9 SO3::vec(OptionalJacobian<9, 3> H) const;

#if GTSAM_ENABLE_BOOST_SERIALIZATION
template <class Archive>
/** Serialization function */
void serialize(Archive& ar, SO3& R, const unsigned int /*version*/) {
  Matrix3& M = R.matrix_;
  ar& boost::serialization::make_nvp("R11", M(0, 0));
  ar& boost::serialization::make_nvp("R12", M(0, 1));
  ar& boost::serialization::make_nvp("R13", M(0, 2));
  ar& boost::serialization::make_nvp("R21", M(1, 0));
  ar& boost::serialization::make_nvp("R22", M(1, 1));
  ar& boost::serialization::make_nvp("R23", M(1, 2));
  ar& boost::serialization::make_nvp("R31", M(2, 0));
  ar& boost::serialization::make_nvp("R32", M(2, 1));
  ar& boost::serialization::make_nvp("R33", M(2, 2));
}
#endif

namespace so3 {

/**
 * Compose general matrix with an SO(3) element.
 * We only provide the 9*9 derivative in the first argument M.
 */
GTSAM_EXPORT Matrix3 compose(const Matrix3& M, const SO3& R,
                             OptionalJacobian<9, 9> H = {});

/// (constant) Jacobian of compose wrpt M
GTSAM_EXPORT Matrix99 Dcompose(const SO3& R);

/**
 * Opaque evaluation context at ω: caches W, WW, θ, θ², nearZero/nearPi,
 * Lazily computes D, G, dB, dC, dG on demand.
 * Math is based on Ethan Eade's elegant Lie group document, at
 * https://www.ethaneade.org/lie.pdf, and the Kernel idea in doc/Jacobians.md
 */
struct GTSAM_EXPORT Local {
  /// Tolerance for near zero (theta^2)
  static constexpr double kNearZeroThresholdSq = 1e-6;
  /// Tolerance for near pi (delta^2 = (pi - theta)^2)
  static constexpr double kNearPiThresholdSq = 1e-6;

  Vector3 omega;         ///< The rotation vector.
  double theta2;         ///< The squared norm of the rotation vector (theta^2).
  double theta;          ///< The norm of the rotation vector (theta).
  Matrix3 W;             ///< The skew-symmetric matrix for the rotation vector.
  Matrix3 WW;            ///< The square of the skew-symmetric matrix (W * W).
  bool nearZero{false};  ///< Flag indicating if theta is near zero.
  bool nearPi{false};    ///< Flag indicating if theta is near pi.
  double A, B, C;        ///< Ethan's A,B,C coefficients

  /// Constructor with element of Lie algebra so(3)
  explicit Local(const Vector3& omega,
                 double nearZeroThresholdSq = kNearZeroThresholdSq,
                 double nearPiThresholdSq = kNearPiThresholdSq);

  // Exponential map via Rodrigues formula: I + A(θ) W + B(θ) WW
  Matrix3 expmap() const;

  // Jacobian kernel J_[l/r] = I +/0 B W + C WW  (left/right).
  struct Kernel Jacobian() const &;

  // Specialized kernel for inverse Jacobian, stable even for |omega| > π
  struct InvJKernel InvJacobian() const &;  // I +/- 1/2 W + D WW

  // Gamma kernel: Γ_[l/r] = 0.5 I ± C W + G WW (left/right).
  struct Kernel Gamma() const &;

  // access to (lazily evaluated) coefficients
  double D() const;
  double E() const;
  double dB() const;
  double dC() const;
  double dE() const;

 protected:
  mutable std::optional<double> D_, E_;         ///< D-E are lazily computed.
  mutable std::optional<double> dB_, dC_, dE_;  ///< Radial derivatives c(θ)'/θ
};

/**
 * Kernel: M(ω) = a I + b W + c W^2 with radial derivatives db,dc for Fréchet.
 * Right variants flip b→-b, db→-db (no recompute of W/WW).
 * Keep a pointer to Local: Kernel methods above return a const & to prevent
 * having a pointer to a temporary.
 */
struct GTSAM_EXPORT Kernel {
  const Local* S;
  double a{0}, b{0}, c{0}, db{0}, dc{0};  // left-specialization form

  Matrix3 left() const;   // a I + b W + c WW
  Matrix3 right() const;  // a I - b W + c WW

  Vector3 applyLeft(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                    OptionalJacobian<3, 3> Hv = {}) const;
  Vector3 applyRight(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                     OptionalJacobian<3, 3> Hv = {}) const;

  /// Fréchet derivative of left-kernel M(ω) in the direction X ∈ so(3)
  /// L_M(Ω)[X] = b X + c (Ω X + X Ω) + s (db Ω + dc Ω²), with s = -½ tr(Ω X)
  Matrix3 frechet(const Matrix3& X) const;
  /// Apply Fréchet derivative to vector (left specialization)
  Matrix3 applyFrechet(const Vector3& v) const;
};

// Stable inverse Jacobian kernel
struct GTSAM_EXPORT InvJKernel {
  const Local* S;
  Kernel J;  // holds the forward kernel

  Matrix3 left() const;
  Matrix3 right() const;

  Vector3 applyLeft(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                    OptionalJacobian<3, 3> Hv = {}) const;
  Vector3 applyRight(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                     OptionalJacobian<3, 3> Hv = {}) const;
};

/// y + alpha * x  (functional)
Kernel axpy(double alpha, const Kernel& X, const Kernel& Y);

// Blend K = α X + (1-α) Y with radial derivative (·)'/θ via dalpha
Kernel blend(double alpha, double dalpha, const Kernel& X, const Kernel& Y);

/// @deprecated: use so3::Local
struct GTSAM_EXPORT ExpmapFunctor : public Local {
  explicit ExpmapFunctor(const Vector3& omega);
  ExpmapFunctor(double nearZeroThresholdSq, const Vector3& axis);
  ExpmapFunctor(const Vector3& axis, double angle);
};

/// @deprecated: use so3::Local
struct GTSAM_EXPORT DexpFunctor : public ExpmapFunctor {
  using J33 = OptionalJacobian<3, 3>;
  explicit DexpFunctor(const Vector3& omega, double nearZeroThresholdSq = 1e-6,
                       double nearPiThresholdSq = 1e-6);
  Matrix3 rightJacobian() const;
  Matrix3 leftJacobian() const;
  Matrix3 rightJacobianInverse() const;
  Matrix3 leftJacobianInverse() const;
  Vector3 applyRightJacobian(const Vector3& v, J33 H1 = {}, J33 H2 = {}) const;
  Vector3 applyRightJacobianInverse(const Vector3& v, J33 H1 = {},
                                    J33 H2 = {}) const;
  Vector3 applyLeftJacobian(const Vector3& v, J33 H1 = {}, J33 H2 = {}) const;
  Vector3 applyLeftJacobianInverse(const Vector3& v, J33 H1 = {},
                                   J33 H2 = {}) const;
  inline Matrix3 dexp() const { return rightJacobian(); }
  inline Matrix3 invDexp() const { return rightJacobianInverse(); }
};
}  //  namespace so3

/*
 * Define the traits. internal::MatrixLieGroup provides both Lie group and Testable
 */

template <>
struct traits<SO3> : public internal::MatrixLieGroup<SO3, 3> {};

template <>
struct traits<const SO3> : public internal::MatrixLieGroup<SO3, 3> {};

}  // end namespace gtsam
