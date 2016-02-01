/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO3.cpp
 * @brief   3*3 matrix representation of SO(3)
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @author  Duy Nguyen Ta
 * @date    December 2014
 */

#include <gtsam/geometry/SO3.h>
#include <gtsam/base/concepts.h>
#include <cmath>
#include <limits>

namespace gtsam {

/* ************************************************************************* */
// Functor implementing Exponential map
struct ExpmapImpl {
  const double theta2;
  Matrix3 W, K, KK;
  bool nearZero;
  double theta, sin_theta, one_minus_cos;  // only defined if !nearZero

  void init() {
    nearZero = (theta2 <= std::numeric_limits<double>::epsilon());
    if (nearZero) return;
    theta = std::sqrt(theta2);  // rotation angle
    sin_theta = std::sin(theta);
    const double s2 = std::sin(theta / 2.0);
    one_minus_cos = 2.0 * s2 * s2;  // numerically better than [1 - cos(theta)]
  }

  // Constructor with element of Lie algebra so(3)
  ExpmapImpl(const Vector3& omega) : theta2(omega.dot(omega)) {
    const double wx = omega.x(), wy = omega.y(), wz = omega.z();
    W << 0.0, -wz, +wy, +wz, 0.0, -wx, -wy, +wx, 0.0;
    init();
    if (!nearZero) {
      theta = std::sqrt(theta2);
      K = W / theta;
      KK = K * K;
    }
  }

  // Constructor with axis-angle
  ExpmapImpl(const Vector3& axis, double angle) : theta2(angle * angle) {
    const double ax = axis.x(), ay = axis.y(), az = axis.z();
    K << 0.0, -az, +ay, +az, 0.0, -ax, -ay, +ax, 0.0;
    W = K * angle;
    init();
    if (!nearZero) {
      theta = angle;
      KK = K * K;
    }
  }

  // Rodrgues formula
  SO3 expmap() const {
    if (nearZero)
      return I_3x3 + W;
    else
      return I_3x3 + sin_theta * K + one_minus_cos * K * K;
  }
};

/* ************************************************************************* */
SO3 SO3::AxisAngle(const Vector3& axis, double theta) {
  return ExpmapImpl(axis, theta).expmap();
}

/* ************************************************************************* */
// Functor that implements Exponential map *and* its derivatives
struct DexpImpl : ExpmapImpl {
  const Vector3 omega;
  double a, b;  // constants used in dexp and applyDexp

  // Constructor with element of Lie algebra so(3)
  DexpImpl(const Vector3& omega) : ExpmapImpl(omega), omega(omega) {
    if (nearZero) return;
    a = one_minus_cos / theta;
    b = 1.0 - sin_theta / theta;
  }

  // NOTE(luca): Right Jacobian for Exponential map in SO(3) - equation
  // (10.86) and following equations in G.S. Chirikjian, "Stochastic Models,
  // Information Theory, and Lie Groups", Volume 2, 2008.
  //   expmap(omega + v) \approx expmap(omega) * expmap(dexp * v)
  // This maps a perturbation v in the tangent space to
  // a perturbation on the manifold Expmap(dexp * v) */
  SO3 dexp() const {
    if (nearZero)
      return I_3x3 - 0.5 * W;
    else
      return I_3x3 - a * K + b * KK;
  }

  // Just multiplies with dexp()
  Vector3 applyDexp(const Vector3& v, OptionalJacobian<3, 3> H1,
                    OptionalJacobian<3, 3> H2) const {
    if (nearZero) {
      if (H1) *H1 = 0.5 * skewSymmetric(v);
      if (H2) *H2 = I_3x3;
      return v - 0.5 * omega.cross(v);
    }
    const Vector3 Kv = omega.cross(v / theta);
    const Vector3 KKv = omega.cross(Kv / theta);
    if (H1) {
      // TODO(frank): Iserles hints that there should be a form I + c*K + d*KK
      const Matrix3 T = skewSymmetric(v / theta);
      const double Da = (sin_theta - 2.0 * a) / theta2;
      const double Db = (one_minus_cos - 3.0 * b) / theta2;
      *H1 = (-Da * Kv + Db * KKv) * omega.transpose() + a * T -
            skewSymmetric(Kv * b / theta) - b * K * T;
    }
    if (H2) *H2 = dexp();
    return v - a * Kv + b * KKv;
  }
};

/* ************************************************************************* */
SO3 SO3::Expmap(const Vector3& omega, ChartJacobian H) {
  if (H) {
    DexpImpl impl(omega);
    *H = impl.dexp();
    return impl.expmap();
  } else
    return ExpmapImpl(omega).expmap();
}

Matrix3 SO3::ExpmapDerivative(const Vector3& omega) {
  return DexpImpl(omega).dexp();
}

Vector3 SO3::ApplyExpmapDerivative(const Vector3& omega, const Vector3& v,
                                   OptionalJacobian<3, 3> H1,
                                   OptionalJacobian<3, 3> H2) {
  return DexpImpl(omega).applyDexp(v, H1, H2);
}

/* ************************************************************************* */
Vector3 SO3::Logmap(const SO3& R, ChartJacobian H) {
  using std::sqrt;
  using std::sin;

  // note switch to base 1
  const double& R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  const double& R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  const double& R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

  // Get trace(R)
  const double tr = R.trace();

  Vector3 omega;

  // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
  // we do something special
  if (std::abs(tr + 1.0) < 1e-10) {
    if (std::abs(R33 + 1.0) > 1e-10)
      omega = (M_PI / sqrt(2.0 + 2.0 * R33)) * Vector3(R13, R23, 1.0 + R33);
    else if (std::abs(R22 + 1.0) > 1e-10)
      omega = (M_PI / sqrt(2.0 + 2.0 * R22)) * Vector3(R12, 1.0 + R22, R32);
    else
      // if(std::abs(R.r1_.x()+1.0) > 1e-10)  This is implicit
      omega = (M_PI / sqrt(2.0 + 2.0 * R11)) * Vector3(1.0 + R11, R21, R31);
  } else {
    double magnitude;
    const double tr_3 = tr - 3.0; // always negative
    if (tr_3 < -1e-7) {
      double theta = acos((tr - 1.0) / 2.0);
      magnitude = theta / (2.0 * sin(theta));
    } else {
      // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      magnitude = 0.5 - tr_3 * tr_3 / 12.0;
    }
    omega = magnitude * Vector3(R32 - R23, R13 - R31, R21 - R12);
  }

  if(H) *H = LogmapDerivative(omega);
  return omega;
}

/* ************************************************************************* */
Matrix3 SO3::LogmapDerivative(const Vector3& omega) {
  using std::cos;
  using std::sin;

  double theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<double>::epsilon()) return I_3x3;
  double theta = std::sqrt(theta2);  // rotation angle
  /** Right Jacobian for Log map in SO(3) - equation (10.86) and following equations in
   * G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
   * logmap( Rhat * expmap(omega) ) \approx logmap( Rhat ) + Jrinv * omega
   * where Jrinv = LogmapDerivative(omega);
   * This maps a perturbation on the manifold (expmap(omega))
   * to a perturbation in the tangent space (Jrinv * omega)
   */
  const Matrix3 W = skewSymmetric(omega); // element of Lie algebra so(3): W = omega^
  return I_3x3 + 0.5 * W +
         (1 / (theta * theta) - (1 + cos(theta)) / (2 * theta * sin(theta))) *
             W * W;
}

/* ************************************************************************* */

} // end namespace gtsam

