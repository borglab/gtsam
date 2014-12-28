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

namespace gtsam {

SO3 Rodrigues(const double& theta, const Vector3& axis) {
  using std::cos;
  using std::sin;

  // get components of axis \omega
  double wx = axis(0), wy = axis(1), wz = axis(2);

  double c = cos(theta), s = sin(theta), c_1 = 1 - c;
  double wwTxx = wx * wx, wwTyy = wy * wy, wwTzz = wz * wz;
  double swx = wx * s, swy = wy * s, swz = wz * s;

  double C00 = c_1 * wwTxx, C01 = c_1 * wx * wy, C02 = c_1 * wx * wz;
  double C11 = c_1 * wwTyy, C12 = c_1 * wy * wz;
  double C22 = c_1 * wwTzz;

  Matrix3 R;
  R << c + C00, -swz + C01, swy + C02, //
  swz + C01, c + C11, -swx + C12, //
  -swy + C02, swx + C12, c + C22;

  return R;
}

/// simply convert omega to axis/angle representation
SO3 SO3::Expmap(const Eigen::Ref<const Vector3>& omega,
    ChartJacobian H) {

  if (H)
    CONCEPT_NOT_IMPLEMENTED;

  if (omega.isZero())
    return SO3::Identity();
  else {
    double angle = omega.norm();
    return Rodrigues(angle, omega / angle);
  }
}

Vector3 SO3::Logmap(const SO3& R, ChartJacobian H) {
  using std::sqrt;
  using std::sin;

  if (H)
    CONCEPT_NOT_IMPLEMENTED;

  // note switch to base 1
  const double& R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  const double& R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  const double& R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

  // Get trace(R)
  double tr = R.trace();

  // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
  // we do something special
  if (std::abs(tr + 1.0) < 1e-10) {
    if (std::abs(R33 + 1.0) > 1e-10)
      return (M_PI / sqrt(2.0 + 2.0 * R33)) * Vector3(R13, R23, 1.0 + R33);
    else if (std::abs(R22 + 1.0) > 1e-10)
      return (M_PI / sqrt(2.0 + 2.0 * R22)) * Vector3(R12, 1.0 + R22, R32);
    else
      // if(std::abs(R.r1_.x()+1.0) > 1e-10)  This is implicit
      return (M_PI / sqrt(2.0 + 2.0 * R11)) * Vector3(1.0 + R11, R21, R31);
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
    return magnitude * Vector3(R32 - R23, R13 - R31, R21 - R12);
  }
}

} // end namespace gtsam

