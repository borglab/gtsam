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
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/concepts.h>
#include <gtsam/geometry/SO3.h>  // Logmap/Expmap derivatives

#include <iostream>
#include <limits>

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
  inline constexpr static auto dimension = 3;
  static int GetDimension(const Q& /* g */) { return 3; }
  typedef OptionalJacobian<3, 3> ChartJacobian;
  typedef Eigen::Matrix<_Scalar, 3, 1, _Options, 3, 1> TangentVector;

  /// @}
  /// @name Lie group traits
  /// @{
  static Q Compose(const Q &g, const Q & h,
      ChartJacobian Hg = {}, ChartJacobian Hh = {}) {
    if (Hg) *Hg = h.toRotationMatrix().transpose();
    if (Hh) *Hh = I_3x3;
    return g * h;
  }

  static Q Between(const Q &g, const Q & h,
      ChartJacobian Hg = {}, ChartJacobian Hh = {}) {
    Q d = g.inverse() * h;
    if (Hg) *Hg = -d.toRotationMatrix().transpose();
    if (Hh) *Hh = I_3x3;
    return d;
  }

  static Q Inverse(const Q &g,
      ChartJacobian H = {}) {
    if (H) *H = -g.toRotationMatrix();
    return g.inverse();
  }

  /// Exponential map, using the inlined code from Eigen's conversion from axis/angle
  static Q Expmap(const Eigen::Ref<const TangentVector>& omega,
                  ChartJacobian H = {}) {
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
  static TangentVector Logmap(const Q& q, ChartJacobian H = {}) {
    using std::atan;

    // theta = 2 * atan2(|v|, w),  omega = theta * v / |v|
    // (C. Hertzberg et al., Information Fusion, 2011), written with atan()
    // because it is measurably cheaper than atan2() and w >= 0 is arranged
    // below anyway. Both theta and v/|v| are invariant to a positive rescaling
    // of (w, v), so this stays correct when q is not of unit norm -- which
    // matters because Rot3Q builds its quaternion straight from a rotation
    // matrix supplied by the caller and never renormalizes.
    //
    // This also removes the need for the two Taylor branches the previous
    // acos-based form required: atan is accurate as |v| -> 0, so the only
    // degenerate case left is |v| == 0 exactly, i.e. the identity.
    const _Scalar qw = q.w();
    const _Scalar n = q.vec().norm();

    TangentVector omega;
    if (n == _Scalar(0)) {
      omega = TangentVector::Zero();
    } else {
      // Branch exactly as the previous implementation did, so that the sign
      // chosen at qw == 0 (an exact pi rotation, where both signs are valid
      // logs) is unchanged. Note !(qw > 0) rather than qw < 0.
      const bool negate = !(qw > _Scalar(0));
      const _Scalar w = negate ? -qw : qw;
      const _Scalar theta = (w > n) ? _Scalar(2) * atan(n / w)
                                    : _Scalar(M_PI) - _Scalar(2) * atan(w / n);
      omega = (negate ? -theta / n : theta / n) * q.vec();
    }

    if (H) *H = SO3::LogmapDerivative(omega.template cast<double>());
    return omega;
  }



  static Matrix3 AdjointMap(const Q &g) {
    return g.toRotationMatrix();
  }

  using LieAlgebra = Matrix3;

  static Matrix3 Hat(const Vector3& v) {
    return SO3::Hat(v);
  }
  
  static Vector3 Vee(const Matrix3& X) {
    return SO3::Vee(X);
  }

  static Vector9 Vec(const Q& q, OptionalJacobian<9, 3> H = {}) {
    return SO3(q.toRotationMatrix()).SO3::vec(H);
  }

  /// @}
  /// @name Manifold traits
  /// @{

  static TangentVector Local(const Q& g, const Q& h,
      ChartJacobian H1 = {}, ChartJacobian H2 = {}) {
    Q b = Between(g, h, H1, H2);
    Matrix3 D_v_b;
    TangentVector v = Logmap(b, (H1 || H2) ? &D_v_b : 0);
    if (H1) *H1 = D_v_b * (*H1);
    if (H2) *H2 = D_v_b * (*H2);
    return v;
  }

  static Q Retract(const Q& g, const TangentVector& v,
      ChartJacobian H1 = {}, ChartJacobian H2 = {}) {
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
