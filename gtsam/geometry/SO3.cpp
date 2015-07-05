/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO3.cpp
 * @brief   3*3 matrix representation o SO(3)
 * @author  Frank Dellaert
 * @date    December 2014
 */

#include <gtsam/geometry/SO3.h>
#include <gtsam/base/concepts.h>
#include <cmath>
#include <limits>

namespace gtsam {

/* ************************************************************************* */
// Near zero, we just have I + skew(omega)
static SO3 FirstOrder(const Vector3& omega) {
  Matrix3 R;
  R(0, 0) =  1.;
  R(1, 0) =  omega.z();
  R(2, 0) = -omega.y();
  R(0, 1) = -omega.z();
  R(1, 1) =  1.;
  R(2, 1) =  omega.x();
  R(0, 2) =  omega.y();
  R(1, 2) = -omega.x();
  R(2, 2) =  1.;
  return R;
}

SO3 SO3::AxisAngle(const Vector3& axis, double theta) {
  if (theta*theta > std::numeric_limits<double>::epsilon()) {
    using std::cos;
    using std::sin;

    // get components of axis \omega, where is a unit vector
    const double& wx = axis.x(), wy = axis.y(), wz = axis.z();

    const double costheta = cos(theta), sintheta = sin(theta), c_1 = 1 - costheta;
    const double wx_sintheta = wx * sintheta, wy_sintheta = wy * sintheta,
                 wz_sintheta = wz * sintheta;

    const double C00 = c_1 * wx * wx, C01 = c_1 * wx * wy, C02 = c_1 * wx * wz;
    const double C11 = c_1 * wy * wy, C12 = c_1 * wy * wz;
    const double C22 = c_1 * wz * wz;

    Matrix3 R;
    R(0, 0) =     costheta + C00;
    R(1, 0) =  wz_sintheta + C01;
    R(2, 0) = -wy_sintheta + C02;
    R(0, 1) = -wz_sintheta + C01;
    R(1, 1) =     costheta + C11;
    R(2, 1) =  wx_sintheta + C12;
    R(0, 2) =  wy_sintheta + C02;
    R(1, 2) = -wx_sintheta + C12;
    R(2, 2) =     costheta + C22;
    return R;
  } else {
    return FirstOrder(axis*theta);
  }

}

/// simply convert omega to axis/angle representation
SO3 SO3::Expmap(const Vector3& omega, ChartJacobian H) {
  if (H) *H = ExpmapDerivative(omega);

  double theta2 = omega.dot(omega);
  if (theta2 > std::numeric_limits<double>::epsilon()) {
    double theta = std::sqrt(theta2);
    return AxisAngle(omega / theta, theta);
  } else {
    return FirstOrder(omega);
  }
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
Matrix3 SO3::ExpmapDerivative(const Vector3& omega)    {
  using std::cos;
  using std::sin;

  double theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<double>::epsilon()) return I_3x3;
  double theta = std::sqrt(theta2);  // rotation angle
#ifdef DUY_VERSION
  /// Follow Iserles05an, B10, pg 147, with a sign change in the second term (left version)
  Matrix3 X = skewSymmetric(omega);
  Matrix3 X2 = X*X;
  double vi = theta/2.0;
  double s1 = sin(vi)/vi;
  double s2 = (theta - sin(theta))/(theta*theta*theta);
  return I_3x3 - 0.5*s1*s1*X + s2*X2;
#else // Luca's version
  /**
   * Right Jacobian for Exponential map in SO(3) - equation (10.86) and following equations in
   * G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
   * expmap(thetahat + omega) \approx expmap(thetahat) * expmap(Jr * omega)
   * where Jr = ExpmapDerivative(thetahat);
   * This maps a perturbation in the tangent space (omega) to
   * a perturbation on the manifold (expmap(Jr * omega))
   */
  // element of Lie algebra so(3): X = omega^, normalized by normx
  const Matrix3 Y = skewSymmetric(omega) / theta;
  return I_3x3 - ((1 - cos(theta)) / (theta)) * Y
      + (1 - sin(theta) / theta) * Y * Y; // right Jacobian
#endif
}

/* ************************************************************************* */
Matrix3 SO3::LogmapDerivative(const Vector3& omega)    {
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

