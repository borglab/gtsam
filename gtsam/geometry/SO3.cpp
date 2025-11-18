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

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/concepts.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SO3.h>

#include <Eigen/SVD>
#include <cmath>
#include <limits>

namespace gtsam {

//******************************************************************************
namespace so3 {

static constexpr double one_6th = 1.0 / 6.0;
static constexpr double one_12th = 1.0 / 12.0;
static constexpr double one_24th = 1.0 / 24.0;
static constexpr double one_60th = 1.0 / 60.0;
static constexpr double one_120th = 1.0 / 120.0;
static constexpr double one_180th = 1.0 / 180.0;
static constexpr double one_360th = 1.0 / 360.0;
static constexpr double one_720th = 1.0 / 720.0;
static constexpr double one_1260th = 1.0 / 1260.0;

static constexpr double kPi_inv = 1.0 / M_PI;
static constexpr double kPi2 = M_PI * M_PI;
static constexpr double k1_Pi2 = 1.0 / kPi2;
static constexpr double kPi3 = M_PI * kPi2;
static constexpr double k1_Pi3 = 1.0 / kPi3;
static constexpr double k2_Pi3 = 2.0 * k1_Pi3;
static constexpr double k1_4Pi = 0.25 * kPi_inv; // 1/(4*pi)

// --- Thresholds ---
// Tolerance for near zero (theta^2)
static constexpr double kNearZeroThresholdSq = 1e-6;
// Tolerance for near pi (delta^2 = (pi - theta)^2)
static constexpr double kNearPiThresholdSq = 1e-6;

GTSAM_EXPORT Matrix99 Dcompose(const SO3& Q) {
  Matrix99 H;
  auto R = Q.matrix();
  H << I_3x3 * R(0, 0), I_3x3 * R(1, 0), I_3x3 * R(2, 0),  //
      I_3x3 * R(0, 1), I_3x3 * R(1, 1), I_3x3 * R(2, 1),   //
      I_3x3 * R(0, 2), I_3x3 * R(1, 2), I_3x3 * R(2, 2);
  return H;
}

GTSAM_EXPORT Matrix3 compose(const Matrix3& M, const SO3& R,
                             OptionalJacobian<9, 9> H) {
  Matrix3 MR = M * R.matrix();
  if (H) *H = Dcompose(R);
  return MR;
}

void ExpmapFunctor::init(double nearZeroThresholdSq) {
  nearZero = (theta2 <= nearZeroThresholdSq);

  if (!nearZero) {
    // General case: Use standard stable formulas for A and B
    const double sin_theta = std::sin(theta);
    A = sin_theta / theta;
    const double s2 = std::sin(theta / 2.0);
    const double one_minus_cos =
        2.0 * s2 * s2;  // numerically better than [1 - cos(theta)]
    B = one_minus_cos / theta2;
  } else {
    // Taylor expansion at 0 for A, B (Order theta^2)
    A = 1.0 - theta2 * one_6th;
    B = 0.5 - theta2 * one_24th;
  }
}

ExpmapFunctor::ExpmapFunctor(const Vector3& omega) :ExpmapFunctor(kNearZeroThresholdSq, omega) {}

ExpmapFunctor::ExpmapFunctor(double nearZeroThresholdSq, const Vector3& omega)
    : theta2(omega.dot(omega)),
      theta(std::sqrt(theta2)),
      W(skewSymmetric(omega)),
      WW(W * W) {
  init(nearZeroThresholdSq);
}

ExpmapFunctor::ExpmapFunctor(const Vector3& axis, double angle)
    : theta2(angle * angle),
      theta(angle),
      W(skewSymmetric(axis * angle)),
      WW(W * W) {
  init(kNearZeroThresholdSq);
}


Matrix3 ExpmapFunctor::expmap() const { return I_3x3 + A * W + B * WW; }

DexpFunctor::DexpFunctor(const Vector3& omega, double nearZeroThresholdSq, double nearPiThresholdSq)
  : ExpmapFunctor(nearZeroThresholdSq, omega), omega(omega) {
  // General case or nearPi: Use standard stable formulas first
  const double delta = M_PI > theta ? M_PI - theta : 0.0;
  const double delta2 = delta * delta;
  nearPi = (delta2 < nearPiThresholdSq);
}

DexpFunctor::DexpFunctor(const Vector3& omega)
    : DexpFunctor(omega, kNearZeroThresholdSq, kNearPiThresholdSq) {}

double DexpFunctor::C() const {
  if (!C_.has_value()) {
    // Usually stable, even near pi (1-0)/pi^2
    C_ = !nearZero ? (1.0 - A) / theta2 : (one_6th - theta2 * one_120th);
  }
  return C_.value();
}

double DexpFunctor::D() const {
  if (!D_.has_value()) {
    D_ = !nearZero ? (nearPi ? (k1_Pi2 + (k2_Pi3 - k1_4Pi) * (M_PI - theta))
                             : ((1.0 - A / (2.0 * B)) / theta2))
                   : (one_12th + theta2 * one_720th);
  }
  return D_.value();
}

double DexpFunctor::E() const {
  if (!E_.has_value()) {
    E_ = !nearZero ? ((1.0 - 2.0 * B) / (2.0 * theta2))
                   : (one_24th - theta2 * one_720th);
  }
  return E_.value();
}

double DexpFunctor::dA() const {
  if (!dA_.has_value()) {
    // Identity: dA = A′/θ = C − B (valid for all θ, with our near-zero series)
    dA_ = C() - B;
  }
  return dA_.value();
}

double DexpFunctor::dB() const {
  if (!dB_.has_value()) {
    dB_ =
        !nearZero ? ((A - 2.0 * B) / theta2) : (-one_12th + theta2 * one_180th);
  }
  return dB_.value();
}

double DexpFunctor::dC() const {
  if (!dC_.has_value()) {
    dC_ = !nearZero ? ((B - 3.0 * C()) / theta2)
                    : (-one_60th + theta2 * one_1260th);
  }
  return dC_.value();
}

double DexpFunctor::dE() const {
  if (!dE_.has_value()) {
    dE_ = !nearZero ? (-(dB() + 2.0 * E()) / theta2) : (-one_360th);
  }
  return dE_.value();
}

// --- Kernels ---
Kernel DexpFunctor::Rodrigues() const& { return Kernel{this, 1.0, A, B, dA(), dB()}; }
Kernel DexpFunctor::Jacobian() const& { return Kernel{this, 1.0, B, C(), dB(), dC()}; }
InvJKernel DexpFunctor::InvJacobian() const& { return InvJKernel{this, Jacobian()}; }
Kernel DexpFunctor::Gamma() const& { return Kernel{this, 0.5, C(), E(), dC(), dE()}; }

// --- If you only need Jacobians, not apply ---
Matrix3 DexpFunctor::rightJacobian() const { return I_3x3 - B * W + C() * WW; }
Matrix3 DexpFunctor::leftJacobian() const { return I_3x3 + B * W + C() * WW; }

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
Matrix3 DexpFunctor::rightJacobianInverse() const {
  return InvJacobian().right();
}
Matrix3 DexpFunctor::leftJacobianInverse() const {
  return InvJacobian().left();
}
Vector3 DexpFunctor::applyRightJacobian(const Vector3& v,
                                        OptionalJacobian<3, 3> H1,
                                        OptionalJacobian<3, 3> H2) const {
  return Jacobian().applyRight(v, H1, H2);
}
Vector3 DexpFunctor::applyLeftJacobian(const Vector3& v,
                                       OptionalJacobian<3, 3> H1,
                                       OptionalJacobian<3, 3> H2) const {
  return Jacobian().applyLeft(v, H1, H2);
}
Vector3 DexpFunctor::applyRightJacobianInverse(
    const Vector3& v, OptionalJacobian<3, 3> H1,
    OptionalJacobian<3, 3> H2) const {
  return InvJacobian().applyRight(v, H1, H2);
}
Vector3 DexpFunctor::applyLeftJacobianInverse(const Vector3& v,
                                              OptionalJacobian<3, 3> H1,
                                              OptionalJacobian<3, 3> H2) const {
  return InvJacobian().applyLeft(v, H1, H2);
}
#endif

}  // namespace so3

//******************************************************************************
template <>
GTSAM_EXPORT
SO3 SO3::AxisAngle(const Vector3& axis, double theta) {
  return SO3(so3::ExpmapFunctor(axis, theta).expmap());
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
  return skewSymmetric(xi);
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
SO3 SO3::Expmap(const Vector3& omega, ChartJacobian H) {
  so3::DexpFunctor local(omega);
  if (H) *H = local.rightJacobian();
  return SO3(local.expmap());
}

template <>
GTSAM_EXPORT
Matrix3 SO3::ExpmapDerivative(const Vector3& omega) {
  return so3::DexpFunctor(omega).rightJacobian();
}

//******************************************************************************
template <>
GTSAM_EXPORT
Matrix3 SO3::LogmapDerivative(const Vector3& omega) {
  return so3::DexpFunctor(omega).InvJacobian().right();
}

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
    if (tr_3 < -so3::kNearZeroThresholdSq) {
      // this is the normal case -1 < trace < 3
      double theta = acos((tr - 1.0) / 2.0);
      magnitude = theta / (2.0 * sin(theta));
    } else {
      // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      // see https://github.com/borglab/gtsam/issues/746 for details
      magnitude = 0.5 - tr_3 * (1.0 / 12.0) + tr_3 * tr_3 * (1.0 / 60.0);
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
template <>
GTSAM_EXPORT
Vector9 SO3::vec(OptionalJacobian<9, 3> H) const {
  const Matrix3& R = matrix_;
  if (H) {
    H->setZero();
    H->block<3, 1>(0, 1) = -R.col(2);
    H->block<3, 1>(0, 2) = R.col(1);
    H->block<3, 1>(3, 0) = R.col(2);
    H->block<3, 1>(3, 2) = -R.col(0);
    H->block<3, 1>(6, 0) = -R.col(1);
    H->block<3, 1>(6, 1) = R.col(0);
  }
  return Eigen::Map<const Vector9>(R.data());
}
//******************************************************************************

}  // end namespace gtsam
