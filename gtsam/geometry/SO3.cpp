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

#include <Eigen/SVD>

#include <cmath>
#include <limits>
#include <iostream>

namespace gtsam {

/* ************************************************************************* */
namespace so3 {

Matrix99 Dcompose(const SO3& R) {
  Matrix99 H;
  H << I_3x3 * R(0, 0), I_3x3 * R(1, 0), I_3x3 * R(2, 0),  //
      I_3x3 * R(0, 1), I_3x3 * R(1, 1), I_3x3 * R(2, 1),   //
      I_3x3 * R(0, 2), I_3x3 * R(1, 2), I_3x3 * R(2, 2);
  return H;
}

Matrix3 compose(const Matrix3& M, const SO3& R, OptionalJacobian<9, 9> H) {
  Matrix3 MR = M * R.matrix();
  if (H) *H = Dcompose(R);
  return MR;
}

void ExpmapFunctor::init(bool nearZeroApprox) {
  nearZero = nearZeroApprox || (theta2 <= std::numeric_limits<double>::epsilon());
  if (!nearZero) {
    sin_theta = std::sin(theta);
    const double s2 = std::sin(theta / 2.0);
    one_minus_cos = 2.0 * s2 * s2;  // numerically better than [1 - cos(theta)]
  }
}

ExpmapFunctor::ExpmapFunctor(const Vector3& omega, bool nearZeroApprox)
    : theta2(omega.dot(omega)), theta(std::sqrt(theta2)) {
  const double wx = omega.x(), wy = omega.y(), wz = omega.z();
  W << 0.0, -wz, +wy, +wz, 0.0, -wx, -wy, +wx, 0.0;
  init(nearZeroApprox);
  if (!nearZero) {
    K = W / theta;
    KK = K * K;
  }
}

ExpmapFunctor::ExpmapFunctor(const Vector3& axis, double angle, bool nearZeroApprox)
    : theta2(angle * angle), theta(angle) {
  const double ax = axis.x(), ay = axis.y(), az = axis.z();
  K << 0.0, -az, +ay, +az, 0.0, -ax, -ay, +ax, 0.0;
  W = K * angle;
  init(nearZeroApprox);
  if (!nearZero) {
    KK = K * K;
  }
}

SO3 ExpmapFunctor::expmap() const {
  if (nearZero)
    return I_3x3 + W;
  else
    return I_3x3 + sin_theta * K + one_minus_cos * KK;
}

DexpFunctor::DexpFunctor(const Vector3& omega, bool nearZeroApprox)
    : ExpmapFunctor(omega, nearZeroApprox), omega(omega) {
  if (nearZero)
    dexp_ = I_3x3 - 0.5 * W;
  else {
    a = one_minus_cos / theta;
    b = 1.0 - sin_theta / theta;
    dexp_ = I_3x3 - a * K + b * KK;
  }
}

Vector3 DexpFunctor::applyDexp(const Vector3& v, OptionalJacobian<3, 3> H1,
                               OptionalJacobian<3, 3> H2) const {
  if (H1) {
    if (nearZero) {
      *H1 = 0.5 * skewSymmetric(v);
    } else {
      // TODO(frank): Iserles hints that there should be a form I + c*K + d*KK
      const Vector3 Kv = K * v;
      const double Da = (sin_theta - 2.0 * a) / theta2;
      const double Db = (one_minus_cos - 3.0 * b) / theta2;
      *H1 = (Db * K - Da * I_3x3) * Kv * omega.transpose() -
            skewSymmetric(Kv * b / theta) +
            (a * I_3x3 - b * K) * skewSymmetric(v / theta);
    }
  }
  if (H2) *H2 = dexp_;
  return dexp_ * v;
}

Vector3 DexpFunctor::applyInvDexp(const Vector3& v, OptionalJacobian<3, 3> H1,
                                  OptionalJacobian<3, 3> H2) const {
  const Matrix3 invDexp = dexp_.inverse();
  const Vector3 c = invDexp * v;
  if (H1) {
    Matrix3 D_dexpv_omega;
    applyDexp(c, D_dexpv_omega);  // get derivative H of forward mapping
    *H1 = -invDexp* D_dexpv_omega;
  }
  if (H2) *H2 = invDexp;
  return c;
}

}  // namespace so3

/* ************************************************************************* */
SO3 SO3::AxisAngle(const Vector3& axis, double theta) {
  return so3::ExpmapFunctor(axis, theta).expmap();
}

/* ************************************************************************* */
SO3 SO3::ClosestTo(const Matrix3& M) {
  Eigen::JacobiSVD<Matrix3> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& U = svd.matrixU();
  const auto& V = svd.matrixV();
  const double det = (U * V.transpose()).determinant();
  return U * Vector3(1, 1, det).asDiagonal() * V.transpose();
}

/* ************************************************************************* */
SO3 SO3::ChordalMean(const std::vector<SO3>& rotations) {
  //  See Hartley13ijcv:
  //  Cost function C(R) = \sum sqr(|R-R_i|_F)
  // Closed form solution = ClosestTo(C_e), where C_e = \sum R_i !!!!
  Matrix3 C_e {Z_3x3};
  for (const auto& R_i : rotations) {
    C_e += R_i;
  }
  return ClosestTo(C_e);
}

/* ************************************************************************* */
void SO3::print(const std::string& s) const {
   std::cout << s << *this << std::endl;
 }

//******************************************************************************
 Matrix3 SO3::Hat(const Vector3& xi) { return skewSymmetric(xi); }

 /* ************************************************************************* */
 Vector3 SO3::Vee(const Matrix3& X) {
   Vector3 xi;
   xi(0) = -X(1, 2);
   xi(1) = X(0, 2);
   xi(2) = -X(0, 1);
   return xi;
}

/* ************************************************************************* */
SO3 SO3::Expmap(const Vector3& omega, ChartJacobian H) {
  if (H) {
    so3::DexpFunctor impl(omega);
    *H = impl.dexp();
    return impl.expmap();
  } else
    return so3::ExpmapFunctor(omega).expmap();
}

Matrix3 SO3::ExpmapDerivative(const Vector3& omega) {
  return so3::DexpFunctor(omega).dexp();
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
static Vector9 vec(const SO3& R) { return Eigen::Map<const Vector9>(R.data()); }

static const std::vector<const Matrix3> G({SO3::Hat(Vector3::Unit(0)),
                                           SO3::Hat(Vector3::Unit(1)),
                                           SO3::Hat(Vector3::Unit(2))});

static const Matrix93 P =
    (Matrix93() << vec(G[0]), vec(G[1]), vec(G[2])).finished();

/* ************************************************************************* */
Vector9 SO3::vec(OptionalJacobian<9, 3> H) const {
  const SO3& R = *this;
  if (H) {
    // As Luca calculated (for SO4), this is (I3 \oplus R) * P
    *H << R * P.block<3, 3>(0, 0), R * P.block<3, 3>(3, 0),
        R * P.block<3, 3>(6, 0);
  }
  return gtsam::vec(R);
};

/* ************************************************************************* */

} // end namespace gtsam

