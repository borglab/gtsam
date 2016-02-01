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
// Functor that helps implement Exponential map and its derivatives
struct ExpmapImpl {
  const Vector3 omega;
  const double theta2;
  Matrix3 W;
  bool nearZero;
  double theta, s1, s2, c_1;

  // omega: element of Lie algebra so(3): W = omega^, normalized by normx
  ExpmapImpl(const Vector3& omega) : omega(omega), theta2(omega.dot(omega)) {
    const double wx = omega.x(), wy = omega.y(), wz = omega.z();
    W << 0.0, -wz, +wy, +wz, 0.0, -wx, -wy, +wx, 0.0;  // Skew[omega]
    nearZero = (theta2 <= std::numeric_limits<double>::epsilon());
    if (!nearZero) {
      theta = std::sqrt(theta2);  // rotation angle
      s1 = std::sin(theta) / theta;
      s2 = std::sin(theta / 2.0);
      c_1 = 2.0 * s2 * s2;  // numerically better behaved than [1 - cos(theta)]
    }
  }

  SO3 operator()() const {
    if (nearZero)
      return I_3x3 + W;
    else
      return I_3x3 + s1 * W + c_1 * W * W / theta2;
  }

  // NOTE(luca): Right Jacobian for Exponential map in SO(3) - equation
  // (10.86) and following equations in G.S. Chirikjian, "Stochastic Models,
  // Information Theory, and Lie Groups", Volume 2, 2008.
  //   expmap(omega + v) \approx expmap(omega) * expmap(dexp * v)
  // This maps a perturbation v in the tangent space to
  // a perturbation on the manifold Expmap(dexp * v) */
  SO3 dexp() const {
    if (nearZero) {
      return I_3x3 - 0.5 * W;
    } else {
      const double a = c_1 / theta2;
      const double b = (1.0 - s1) / theta2;
      return I_3x3 - a * W + b * W * W;
    }
  }

  // Just multiplies with dexp()
  Vector3 applyDexp(const Vector3& v, OptionalJacobian<3, 3> H1 = boost::none,
                    OptionalJacobian<3, 3> H2 = boost::none) const {
    if (nearZero) {
      if (H1) *H1 = 0.5 * skewSymmetric(v);
      if (H2) *H2 = I_3x3;
      return v;
    } else {
      const double a = c_1 / theta2;
      const double b = (1.0 - s1) / theta2;
      Matrix3 dexp = I_3x3 - a * W + b * W * W;
      if (H1) {
        const Vector3 Wv = omega.cross(v);
        const Vector3 WWv = omega.cross(Wv);
        const Matrix3 T = skewSymmetric(v);
        const double Da = (s1 - 2.0 * a) / theta2;
        const double Db = (3.0 * s1 - std::cos(theta) - 2.0) / theta2 / theta2;
        *H1 = (-Da * Wv + Db * WWv) * omega.transpose() + a * T -
              b * skewSymmetric(Wv) - b * W * T;
      }
      if (H2) *H2 = dexp;
      return dexp * v;
    }
  }
};

SO3 SO3::AxisAngle(const Vector3& axis, double theta) {
  return ExpmapImpl(axis*theta)();
}

SO3 SO3::Expmap(const Vector3& omega, ChartJacobian H) {
  ExpmapImpl impl(omega);
  if (H) *H = impl.dexp();
  return impl();
}

Matrix3 SO3::ExpmapDerivative(const Vector3& omega) {
  return ExpmapImpl(omega).dexp();
}

Vector3 SO3::ApplyExpmapDerivative(const Vector3& omega, const Vector3& v,
                                   OptionalJacobian<3, 3> H1,
                                   OptionalJacobian<3, 3> H2) {
  return ExpmapImpl(omega).applyDexp(v, H1, H2);
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
  double tr = R.trace();

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
    double tr_3 = tr - 3.0; // always negative
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
#ifdef DUY_VERSION
  /// Follow Iserles05an, B11, pg 147, with a sign change in the second term (left version)
  Matrix3 X = skewSymmetric(omega);
  Matrix3 X2 = X*X;
  double vi = theta/2.0;
  double s2 = (theta*tan(M_PI_2-vi) - 2)/(2*theta*theta);
  return I_3x3 + 0.5*X - s2*X2;
#else // Luca's version
  /** Right Jacobian for Log map in SO(3) - equation (10.86) and following equations in
   * G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
   * logmap( Rhat * expmap(omega) ) \approx logmap( Rhat ) + Jrinv * omega
   * where Jrinv = LogmapDerivative(omega);
   * This maps a perturbation on the manifold (expmap(omega))
   * to a perturbation in the tangent space (Jrinv * omega)
   */
  const Matrix3 X = skewSymmetric(omega); // element of Lie algebra so(3): X = omega^
  return I_3x3 + 0.5 * X
      + (1 / (theta * theta) - (1 + cos(theta)) / (2 * theta * sin(theta))) * X
          * X;
#endif
}

/* ************************************************************************* */

} // end namespace gtsam

