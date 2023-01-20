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

#include <gtsam/base/concepts.h>
#include <gtsam/geometry/SO3.h>

#include <Eigen/SVD>

#include <cmath>
#include <iostream>
#include <limits>

namespace gtsam {

//******************************************************************************
namespace so3 {

GTSAM_EXPORT Matrix99 Dcompose(const SO3& Q) {
  Matrix99 H;
  auto R = Q.matrix();
  H << I_3x3 * R(0, 0), I_3x3 * R(1, 0), I_3x3 * R(2, 0),  //
      I_3x3 * R(0, 1), I_3x3 * R(1, 1), I_3x3 * R(2, 1),   //
      I_3x3 * R(0, 2), I_3x3 * R(1, 2), I_3x3 * R(2, 2);
  return H;
}

GTSAM_EXPORT Matrix3 compose(const Matrix3& M, const SO3& R, OptionalJacobian<9, 9> H) {
  Matrix3 MR = M * R.matrix();
  if (H) *H = Dcompose(R);
  return MR;
}

void ExpmapFunctor::init(bool nearZeroApprox) {
  nearZero =
      nearZeroApprox || (theta2 <= std::numeric_limits<double>::epsilon());
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

ExpmapFunctor::ExpmapFunctor(const Vector3& axis, double angle,
                             bool nearZeroApprox)
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
    return SO3(I_3x3 + W);
  else
    return SO3(I_3x3 + sin_theta * K + one_minus_cos * KK);
}

DexpFunctor::DexpFunctor(const Vector3& omega, bool nearZeroApprox)
    : ExpmapFunctor(omega, nearZeroApprox), omega(omega) {
  if (nearZero) {
    dexp_ = I_3x3 - 0.5 * W;
  } else {
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
    *H1 = -invDexp * D_dexpv_omega;
  }
  if (H2) *H2 = invDexp;
  return c;
}

}  // namespace so3

//******************************************************************************
template <>
GTSAM_EXPORT
SO3 SO3::AxisAngle(const Vector3& axis, double theta) {
  return so3::ExpmapFunctor(axis, theta).expmap();
}

//******************************************************************************
template <>
GTSAM_EXPORT
SO3 SO3::ClosestTo(const Matrix3& M) {
  Eigen::JacobiSVD<Matrix3> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto& U = svd.matrixU();
  const auto& V = svd.matrixV();
  const double det = (U * V.transpose()).determinant();
  return SO3(U * Vector3(1, 1, det).asDiagonal() * V.transpose());
}

//******************************************************************************
template <>
GTSAM_EXPORT
SO3 SO3::ChordalMean(const std::vector<SO3>& rotations) {
  // See Hartley13ijcv:
  // Cost function C(R) = \sum sqr(|R-R_i|_F)
  // Closed form solution = ClosestTo(C_e), where C_e = \sum R_i !!!!
  Matrix3 C_e{Z_3x3};
  for (const auto& R_i : rotations) {
    C_e += R_i.matrix();
  }
  return ClosestTo(C_e);
}

//******************************************************************************
template <>
GTSAM_EXPORT
Matrix3 SO3::Hat(const Vector3& xi) {
  // skew symmetric matrix X = xi^
  Matrix3 Y = Z_3x3;
  Y(0, 1) = -xi(2);
  Y(0, 2) = +xi(1);
  Y(1, 2) = -xi(0);
  return Y - Y.transpose();
}

//******************************************************************************
template <>
GTSAM_EXPORT
Vector3 SO3::Vee(const Matrix3& X) {
  Vector3 xi;
  xi(0) = -X(1, 2);
  xi(1) = +X(0, 2);
  xi(2) = -X(0, 1);
  return xi;
}

//******************************************************************************
template <>
GTSAM_EXPORT
Matrix3 SO3::AdjointMap() const {
  return matrix_;
}

//******************************************************************************
template <>
GTSAM_EXPORT
SO3 SO3::Expmap(const Vector3& omega, ChartJacobian H) {
  if (H) {
    so3::DexpFunctor impl(omega);
    *H = impl.dexp();
    return impl.expmap();
  } else {
    return so3::ExpmapFunctor(omega).expmap();
  }
}

template <>
GTSAM_EXPORT
Matrix3 SO3::ExpmapDerivative(const Vector3& omega) {
  return so3::DexpFunctor(omega).dexp();
}

//******************************************************************************
/* Right Jacobian for Log map in SO(3) - equation (10.86) and following
 equations in G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie
 Groups", Volume 2, 2008.

   logmap( Rhat * expmap(omega) ) \approx logmap(Rhat) + Jrinv * omega

 where Jrinv = LogmapDerivative(omega). This maps a perturbation on the
 manifold (expmap(omega)) to a perturbation in the tangent space (Jrinv *
 omega)
 */
template <>
GTSAM_EXPORT
Matrix3 SO3::LogmapDerivative(const Vector3& omega) {
  using std::cos;
  using std::sin;

  double theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<double>::epsilon()) return I_3x3;
  double theta = std::sqrt(theta2);  // rotation angle

  // element of Lie algebra so(3): W = omega^
  const Matrix3 W = Hat(omega);
  return I_3x3 + 0.5 * W +
         (1 / (theta * theta) - (1 + cos(theta)) / (2 * theta * sin(theta))) *
             W * W;
}

//******************************************************************************
template <>
GTSAM_EXPORT
Vector3 SO3::Logmap(const SO3& Q, ChartJacobian H) {
  using std::sin;
  using std::sqrt;

  // note switch to base 1
  const Matrix3& R = Q.matrix();
  const double &R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  const double &R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  const double &R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

  // Get trace(R)
  const double tr = R.trace();

  Vector3 omega;

  // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
  // we do something special
  if (tr + 1.0 < 1e-3) {
    if (R33 > R22 && R33 > R11) {
      // R33 is the largest diagonal, a=3, b=1, c=2
      const double W = R21 - R12;
      const double Q1 = 2.0 + 2.0 * R33;
      const double Q2 = R31 + R13;
      const double Q3 = R23 + R32;
      const double r = sqrt(Q1);
      const double one_over_r = 1 / r;
      const double norm = sqrt(Q1*Q1 + Q2*Q2 + Q3*Q3 + W*W);
      const double sgn_w = W < 0 ? -1.0 : 1.0;
      const double mag = M_PI - (2 * sgn_w * W) / norm;
      const double scale = 0.5 * one_over_r * mag;
      omega = sgn_w * scale * Vector3(Q2, Q3, Q1);
    } else if (R22 > R11) {
      // R22 is the largest diagonal, a=2, b=3, c=1
      const double W = R13 - R31;
      const double Q1 = 2.0 + 2.0 * R22;
      const double Q2 = R23 + R32;
      const double Q3 = R12 + R21;
      const double r = sqrt(Q1);
      const double one_over_r = 1 / r;
      const double norm = sqrt(Q1*Q1 + Q2*Q2 + Q3*Q3 + W*W);
      const double sgn_w = W < 0 ? -1.0 : 1.0;
      const double mag = M_PI - (2 * sgn_w * W) / norm;
      const double scale = 0.5 * one_over_r * mag;
      omega = sgn_w * scale * Vector3(Q3, Q1, Q2);
    } else {
      // R11 is the largest diagonal, a=1, b=2, c=3
      const double W = R32 - R23;
      const double Q1 = 2.0 + 2.0 * R11;
      const double Q2 = R12 + R21;
      const double Q3 = R31 + R13;
      const double r = sqrt(Q1);
      const double one_over_r = 1 / r;
      const double norm = sqrt(Q1*Q1 + Q2*Q2 + Q3*Q3 + W*W);
      const double sgn_w = W < 0 ? -1.0 : 1.0;
      const double mag = M_PI - (2 * sgn_w * W) / norm;
      const double scale = 0.5 * one_over_r * mag;
      omega = sgn_w * scale * Vector3(Q1, Q2, Q3);
    }
  } else {
    double magnitude;
    const double tr_3 = tr - 3.0; // could be non-negative if the matrix is off orthogonal
    if (tr_3 < -1e-6) {
      // this is the normal case -1 < trace < 3
      double theta = acos((tr - 1.0) / 2.0);
      magnitude = theta / (2.0 * sin(theta));
    } else {
      // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      // see https://github.com/borglab/gtsam/issues/746 for details
      magnitude = 0.5 - tr_3 / 12.0 + tr_3*tr_3/60.0;
    }
    omega = magnitude * Vector3(R32 - R23, R13 - R31, R21 - R12);
  }

  if (H) *H = LogmapDerivative(omega);
  return omega;
}

//******************************************************************************
// Chart at origin for SO3 is *not* Cayley but actual Expmap/Logmap

template <>
GTSAM_EXPORT
SO3 SO3::ChartAtOrigin::Retract(const Vector3& omega, ChartJacobian H) {
  return Expmap(omega, H);
}

template <>
GTSAM_EXPORT
Vector3 SO3::ChartAtOrigin::Local(const SO3& R, ChartJacobian H) {
  return Logmap(R, H);
}

//******************************************************************************
// local vectorize
static Vector9 vec3(const Matrix3& R) {
  return Eigen::Map<const Vector9>(R.data());
}

// so<3> generators
static std::vector<Matrix3> G3({SO3::Hat(Vector3::Unit(0)),
                                SO3::Hat(Vector3::Unit(1)),
                                SO3::Hat(Vector3::Unit(2))});

// vectorized generators
static const Matrix93 P3 =
    (Matrix93() << vec3(G3[0]), vec3(G3[1]), vec3(G3[2])).finished();

//******************************************************************************
template <>
GTSAM_EXPORT
Vector9 SO3::vec(OptionalJacobian<9, 3> H) const {
  const Matrix3& R = matrix_;
  if (H) {
    // As Luca calculated (for SO4), this is (I3 \oplus R) * P3
    *H << R * P3.block<3, 3>(0, 0), R * P3.block<3, 3>(3, 0),
        R * P3.block<3, 3>(6, 0);
  }
  return gtsam::vec3(R);
}
//******************************************************************************

}  // end namespace gtsam
