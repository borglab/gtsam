/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Quaternion.h
 * @brief  Lie Group wrapper for Eigen Quaternions
 * @author Frank Dellaert
 **/

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/concepts.h>
#include <gtsam/geometry/SO3.h> // Logmap/Expmap derivatives
#include <limits>
#include <iostream>

#define QUATERNION_TYPE Eigen::Quaternion<_Scalar,_Options>

namespace gtsam {

// Define traits
template<typename _Scalar, int _Options>
struct traits<QUATERNION_TYPE> {
  typedef QUATERNION_TYPE ManifoldType;
  typedef QUATERNION_TYPE Q;

  typedef lie_group_tag structure_category;
  typedef multiplicative_group_tag group_flavor;

  /// @name Group traits
  /// @{
  static Q Identity() {
    return Q::Identity();
  }

  /// @}
  /// @name Basic manifold traits
  /// @{
  enum {
    dimension = 3
  };
  typedef OptionalJacobian<3, 3> ChartJacobian;
  typedef Eigen::Matrix<_Scalar, 3, 1, _Options, 3, 1> TangentVector;

  /// @}
  /// @name Lie group traits
  /// @{
  static Q Compose(const Q &g, const Q & h,
      ChartJacobian Hg = boost::none, ChartJacobian Hh = boost::none) {
    if (Hg) *Hg = h.toRotationMatrix().transpose();
    if (Hh) *Hh = I_3x3;
    return g * h;
  }

  static Q Between(const Q &g, const Q & h,
      ChartJacobian Hg = boost::none, ChartJacobian Hh = boost::none) {
    Q d = g.inverse() * h;
    if (Hg) *Hg = -d.toRotationMatrix().transpose();
    if (Hh) *Hh = I_3x3;
    return d;
  }

  static Q Inverse(const Q &g,
      ChartJacobian H = boost::none) {
    if (H) *H = -g.toRotationMatrix();
    return g.inverse();
  }

  /// Exponential map, using the inlined code from Eigen's conversion from axis/angle
  static Q Expmap(const Eigen::Ref<const TangentVector>& omega,
                  ChartJacobian H = boost::none) {
    using std::cos;
    using std::sin;
    if (H) *H = SO3::ExpmapDerivative(omega.template cast<double>());
    _Scalar theta2 = omega.dot(omega);
    if (theta2 > std::numeric_limits<_Scalar>::epsilon()) {
      _Scalar theta = std::sqrt(theta2);
      _Scalar ha = _Scalar(0.5) * theta;
      Vector3 vec = (sin(ha) / theta) * omega;
      return Q(cos(ha), vec.x(), vec.y(), vec.z());
    } else {
      // first order approximation sin(theta/2)/theta = 0.5
      Vector3 vec = _Scalar(0.5) * omega;
      return Q(1.0, vec.x(), vec.y(), vec.z());
    }
  }

  /// We use our own Logmap, as there is a slight bug in Eigen
  static TangentVector Logmap(const Q& q, ChartJacobian H = boost::none) {
    using std::acos;
    using std::sqrt;

    // define these compile time constants to avoid std::abs:
    static const double twoPi = 2.0 * M_PI, NearlyOne = 1.0 - 1e-10,
    NearlyNegativeOne = -1.0 + 1e-10;

    TangentVector omega;

    const _Scalar qw = q.w();
    // See Quaternion-Logmap.nb in doc for Taylor expansions
    if (qw > NearlyOne) {
      // Taylor expansion of (angle / s) at 1
      // (2 + 2 * (1-qw) / 3) * q.vec();
      omega = ( 8. / 3. - 2. / 3. * qw) * q.vec();
    } else if (qw < NearlyNegativeOne) {
      // Taylor expansion of (angle / s) at -1
      // (-2 - 2 * (1 + qw) / 3) * q.vec();
      omega = (-8. / 3. - 2. / 3. * qw) * q.vec();
    } else {
      // Normal, away from zero case
      _Scalar angle = 2 * acos(qw), s = sqrt(1 - qw * qw);
      // Important:  convert to [-pi,pi] to keep error continuous
      if (angle > M_PI)
      angle -= twoPi;
      else if (angle < -M_PI)
      angle += twoPi;
      omega = (angle / s) * q.vec();
    }

    if(H) *H = SO3::LogmapDerivative(omega.template cast<double>());
    return omega;
  }

  /// @}
  /// @name Manifold traits
  /// @{

  static TangentVector Local(const Q& g, const Q& h,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    Q b = Between(g, h, H1, H2);
    Matrix3 D_v_b;
    TangentVector v = Logmap(b, (H1 || H2) ? &D_v_b : 0);
    if (H1) *H1 = D_v_b * (*H1);
    if (H2) *H2 = D_v_b * (*H2);
    return v;
  }

  static Q Retract(const Q& g, const TangentVector& v,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    Matrix3 D_h_v;
    Q b = Expmap(v,H2 ? &D_h_v : 0);
    Q h = Compose(g, b, H1, H2);
    if (H2) *H2 = (*H2) * D_h_v;
    return h;
  }

  /// @}
  /// @name Testable
  /// @{
  static void Print(const Q& q, const std::string& str = "") {
    if (str.size() == 0)
    std::cout << "Eigen::Quaternion: ";
    else
    std::cout << str << " ";
    std::cout << q.vec().transpose() << std::endl;
  }
  static bool Equals(const Q& q1, const Q& q2, double tol = 1e-8) {
    return Between(q1, q2).vec().array().abs().maxCoeff() < tol;
  }
  /// @}
};

typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;

} // \namespace gtsam

