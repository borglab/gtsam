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

// Below are two functors that allow for saving computation when exponential map
// and its derivatives are needed at the same location in so<3>. The second
// functor also implements dedicated methods to apply dexp and/or inv(dexp).

/// Functor implementing Exponential map
/// Math is based on Ethan Eade's elegant Lie group document, at
/// https://www.ethaneade.org/lie.pdf.
struct GTSAM_EXPORT ExpmapFunctor {
  const double theta2, theta;
  const Matrix3 W, WW;
  bool nearZero{ false };

  // Ethan Eade's constants:
  double A;  // A = sin(theta) / theta
  double B;  // B = (1 - cos(theta))

  /// Constructor with element of Lie algebra so(3)
  explicit ExpmapFunctor(const Vector3& omega);

  /// Constructor with threshold (advanced)
  ExpmapFunctor(double nearZeroThresholdSq, const Vector3& axis);

  /// Constructor with axis-angle
  ExpmapFunctor(const Vector3& axis, double angle);

  /// Rodrigues formula
  Matrix3 expmap() const;

protected:
  void init(double nearZeroThresholdSq);
};

struct DexpFunctor; // forward declaration

// Bound kernel: M(ω) = a I + b W + c W^2  with radial derivatives (db, dc) for
// the Fréchet derivative. Holds a const reference to the functor providing
// Ω = [ω]× and Ω².
struct GTSAM_EXPORT ABCKernel {
  const DexpFunctor& S;
  double a{0.0}, b{0.0}, c{0.0};
  double db{0.0}, dc{0.0};  // db = b'(θ)/θ, dc = c'(θ)/θ

  /// M(ω) as a 3x3 matrix
  Matrix3 matrix() const;

  /// Apply: y = M(ω) v
  Vector3 apply(const Vector3& v,  //
                OptionalJacobian<3, 3> H1 = {},
                OptionalJacobian<3, 3> H2 = {}) const;

  /// operator*: y = M(ω) v
  Vector3 operator*(const Vector3& v) const { return apply(v); }

  /// Fréchet derivative of M(ω) in the direction X \in so(3)
  /// L_M(Ω)[X] = b X + c (Ω X + X Ω) + s (db Ω + dc Ω²),
  /// with s = -½ tr(Ω X)
  Matrix3 frechet(const Matrix3& X) const;

  /// Apply Fréchet derivative to vector
  /// Should match H1 derivative of apply
  Matrix3 applyFrechet(const Vector3& v) const;
};

/// Functor that implements Exponential map *and* its derivatives
/// Math extends Ethan theme of elegant a + bW + cWW expressions.
/// See https://www.ethaneade.org/lie.pdf expmap (82) and left Jacobian (83).
struct GTSAM_EXPORT DexpFunctor : public ExpmapFunctor {
  const Vector3 omega;

  // Ethan's C constant used in Jacobians
  double C;  // (1 - A) / theta^2

  // Radial derivatives:
  double dB; // (A - 2B) / theta2 = B'(θ)/θ
  double dC;  // (B - 3C) / theta2 = C'(θ)/θ

  // Factories (right/left Jacobians and Γ kernels)
  ABCKernel Jr() const { return ABCKernel{*this, 1.0, -B, C, -dB, dC}; }
  ABCKernel Jl() const { return ABCKernel{*this, 1.0, +B, C, +dB, dC}; }

  // Constant used in inverse Jacobians
  double D;  // (1 - A/2B) / theta^2

  /// Constructor with element of Lie algebra so(3)
  explicit DexpFunctor(const Vector3& omega);

  /// Constructor with custom thresholds (advanced)
  explicit DexpFunctor(const Vector3& omega, double nearZeroThresholdSq,
                       double nearPiThresholdSq);

  // NOTE(luca): Right Jacobian for Exponential map in SO(3) - equation
  // (10.86) and following equations in G.S. Chirikjian, "Stochastic Models,
  // Information Theory, and Lie Groups", Volume 2, 2008.
  //   Expmap(xi + dxi) \approx Expmap(xi) * Expmap(dexp * dxi)
  // This maps a perturbation dxi=(w,v) in the tangent space to
  // a perturbation on the manifold Expmap(rightJacobian() * xi)
  Matrix3 rightJacobian() const { return I_3x3 - B * W + C * WW; }

  // Compute the left Jacobian for Exponential map in SO(3)
  Matrix3 leftJacobian() const { return I_3x3 + B * W + C * WW; }

  /// Inverse of right Jacobian
  /// For |omega|>pi uses rightJacobian().inverse(), as unstable beyond pi!
  Matrix3 rightJacobianInverse() const;

  // Inverse of left Jacobian
  /// For |omega|>pi uses leftJacobian().inverse(), as unstable beyond pi!
  Matrix3 leftJacobianInverse() const;

  /// Multiplies with rightJacobian(), with optional derivatives
  Vector3 applyRightJacobian(const Vector3& v, OptionalJacobian<3, 3> H1 = {},
                             OptionalJacobian<3, 3> H2 = {}) const {
    return Jr().apply(v, H1, H2);
  }

  /// Multiplies with rightJacobian().inverse(), with optional derivatives
  Vector3 applyRightJacobianInverse(const Vector3& v,
                                    OptionalJacobian<3, 3> H1 = {},
                                    OptionalJacobian<3, 3> H2 = {}) const;

  /// Multiplies with leftJacobian(), with optional derivatives
  Vector3 applyLeftJacobian(const Vector3& v, OptionalJacobian<3, 3> H1 = {},
                            OptionalJacobian<3, 3> H2 = {}) const {
    return Jl().apply(v, H1, H2);
  }

  /// Multiplies with leftJacobianInverse(), with optional derivatives
  Vector3 applyLeftJacobianInverse(const Vector3& v,
                                   OptionalJacobian<3, 3> H1 = {},
                                   OptionalJacobian<3, 3> H2 = {}) const;

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
  /// @deprecated: use rightJacobian
  inline Matrix3 dexp() const { return rightJacobian(); }

  /// @deprecated: use rightJacobianInverse
  inline Matrix3 invDexp() const { return rightJacobianInverse(); }
#endif
};

/// Functor that calculates/applies Gamma = 0.5*I + C*W + G*WW, used in Gal3
struct GTSAM_EXPORT GammaFunctor : public DexpFunctor {
  double G;   // (1 - 2B) / (2 * theta^2)
  double dG;  // -(dB + 2G) / theta2 = G'(θ)/θ
  explicit GammaFunctor(const Vector3& omega, double nearZeroThresholdSq,
                        double nearPiThresholdSq);
  explicit GammaFunctor(const Vector3& omega);

  ABCKernel Gr() const { return ABCKernel{*this, 0.5, -C, G, -dC, dG}; }
  ABCKernel Gl() const { return ABCKernel{*this, 0.5, +C, G, +dC, dG}; }
  Matrix3 rightGamma() const { return Gr().matrix(); }
  Matrix3 leftGamma() const { return Gl().matrix(); }
  /// Apply Gamma to vector v, with optional derivatives
  Vector3 applyRightGamma(const Vector3& v, OptionalJacobian<3, 3> H1 = {},
                          OptionalJacobian<3, 3> H2 = {}) const {
    return Gr().apply(v, H1, H2);
  }
  Vector3 applyLeftGamma(const Vector3& v, OptionalJacobian<3, 3> H1 = {},
                         OptionalJacobian<3, 3> H2 = {}) const {
    return Gl().apply(v, H1, H2);
  }
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
