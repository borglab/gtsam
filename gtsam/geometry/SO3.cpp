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
#include <memory>
#include <optional>

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
static constexpr double k1_4Pi = 0.25 * kPi_inv;  // 1/(4*pi)

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

// --- Local::Cache and Local methods ---
struct Local::Cache {
  Vector3 omega;  ///< The rotation vector.
  double theta2;  ///< The squared norm of the rotation vector (theta^2).
  double theta;   ///< The norm of the rotation vector (theta).
  Matrix3 W;      ///< The skew-symmetric matrix for the rotation vector.
  Matrix3 WW;     ///< The square of the skew-symmetric matrix (W * W).
  bool nearZero;  ///< Flag indicating if theta is near zero.
  bool nearPi;    ///< Flag indicating if theta is near pi.

  // Lazy coefficient cache using optionals
  mutable std::optional<double> A_, B_, C_, D_, E_;  ///< Coefficients
  mutable std::optional<double> dB_, dC_, dE_;       ///< Derivatives

  Cache(const Vector3& w, double nz, double np)
      : omega(w),
        theta2(w.dot(w)),
        theta(std::sqrt(theta2)),
        W(skewSymmetric(w)),
        WW(W * W),
        nearZero(theta2 <= nz) {
    const double delta = M_PI > theta ? (M_PI - theta) : 0.0;
    const double delta2 = delta * delta;
    nearPi = (delta2 < np);
  }

  double A() const {
    if (!A_) {
      A_ = !nearZero ? (std::sin(theta) / theta) : (1.0 - theta2 * one_6th);
    }
    return *A_;
  }

  double B() const {
    if (!B_) {
      if (!nearZero) {
        const double sinHalfTheta = std::sin(0.5 * theta);
        B_ = (2.0 * sinHalfTheta * sinHalfTheta) / theta2;
      } else {
        B_ = 0.5 - theta2 * one_24th;
      }
    }
    return *B_;
  }

  double C() const {
    if (!C_) {
      C_ = !nearZero ? ((1.0 - A()) / theta2) : (one_6th - theta2 * one_120th);
    }
    return *C_;
  }

  double D() const {
    if (!D_) {
      D_ = !nearZero ? (nearPi ? (k1_Pi2 + (k2_Pi3 - k1_4Pi) * (M_PI - theta))
                               : ((1.0 - A() / (2.0 * B())) / theta2))
                     : (one_12th + theta2 * one_720th);
    }
    return *D_;
  }

  double E() const {
    if (!E_) {
      E_ = !nearZero ? ((1.0 - 2.0 * B()) / (2.0 * theta2))
                     : (one_24th - theta2 * one_720th);
    }
    return *E_;
  }

  double dB() const {
    if (!dB_) {
      dB_ = !nearZero ? ((A() - 2.0 * B()) / theta2)
                      : (-one_12th + theta2 * one_180th);
    }
    return *dB_;
  }

  double dC() const {
    if (!dC_) {
      dC_ = !nearZero ? ((B() - 3.0 * C()) / theta2)
                      : (-one_60th + theta2 * one_1260th);
    }
    return *dC_;
  }

  double dE() const {
    if (!dE_) {
      dE_ = !nearZero ? (-(dB() + 2.0 * E()) / theta2) : (-one_360th);
    }
    return *dE_;
  }
};

Local::Local(const Vector3& omega, double nearZeroThresholdSq,
             double nearPiThresholdSq)
    : p_(std::make_shared<Cache>(omega, nearZeroThresholdSq,
                                 nearPiThresholdSq)) {}

const Matrix3& Local::W() const { return p_->W; }
const Matrix3& Local::WW() const { return p_->WW; }
double Local::theta() const { return p_->theta; }
double Local::theta2() const { return p_->theta2; }
bool Local::nearZero() const { return p_->nearZero; }

double Local::A() const { return p_->A(); }
double Local::B() const { return p_->B(); }
double Local::C() const { return p_->C(); }
double Local::D() const { return p_->D(); }
double Local::E() const { return p_->E(); }

// --- Kernel matrices (out-of-line to keep header tight) ---
Matrix3 Kernel::left() const {
  // left: a I + b W + c W^2
  return a * I_3x3 + b * S->W + c * S->WW;
}
Matrix3 Kernel::right() const {
  // right: a I - b W + c W^2
  return a * I_3x3 - b * S->W + c * S->WW;
}

Vector3 Kernel::applyLeft(const Vector3& v, OptionalJacobian<3, 3> Hw,
                          OptionalJacobian<3, 3> Hv) const {
  const Vector3 Wv = S->W * v;
  const Vector3 WWv = S->WW * v;
  if (Hw) {
    // Closed-form ∂/∂ω without doubleCross Jacobian:
    const auto& w = S->omega;
    const auto wt = w.transpose();
    const auto vt = v.transpose();
    const auto sI = w.dot(v) * I_3x3;
    *Hw =
        -b * skewSymmetric(v) +           // d(Wv)/dω = -[v]×
        c * (w * vt + sI - 2 * v * wt) +  // d(WWv)/dω = ω vᵀ + (ω·v) I - 2 v ωᵀ
        (db * Wv + dc * WWv) * wt;        // radial derivative terms
  }
  if (Hv) *Hv = left();  // ∂y/∂v = a I + b W + c W²
  return a * v + b * Wv + c * WWv;
}

Vector3 Kernel::applyRight(const Vector3& v, OptionalJacobian<3, 3> Hw,
                           OptionalJacobian<3, 3> Hv) const {
  // Implement by flipping b and db and reusing left machinery
  Kernel tmp{S, a, -b, c, -db, dc};
  return tmp.applyLeft(v, Hw, Hv);
}

Matrix3 Kernel::frechet(const Matrix3& X) const {
  const Matrix3& W = S->W;
  const Matrix3& WW = S->WW;
  const double s = -0.5 * (W * X).trace();
  return b * X + c * (W * X + X * W) + s * (db * W + dc * WW);
}

Matrix3 Kernel::applyFrechet(const Vector3& v) const {
  Matrix3 H;
  H.col(0) = frechet(skewSymmetric(Vector3::UnitX())) * v;  // δω = e_x
  H.col(1) = frechet(skewSymmetric(Vector3::UnitY())) * v;  // δω = e_y
  H.col(2) = frechet(skewSymmetric(Vector3::UnitZ())) * v;  // δω = e_z
  return H;
}

// --- InvJKernel matrices (closed form; π-stable via Local::InvJacobian()
// selection) ---
Matrix3 InvJKernel::left() const {
  if (S->theta > M_PI) return J.left().inverse();
  return I_3x3 - 0.5 * S->W + S->D() * S->WW;
}
Matrix3 InvJKernel::right() const {
  if (S->theta > M_PI) return J.right().inverse();
  return I_3x3 + 0.5 * S->W + S->D() * S->WW;
}

Vector3 InvJKernel::applyLeft(const Vector3& v, OptionalJacobian<3, 3> Hw,
                              OptionalJacobian<3, 3> Hv) const {
  const Matrix3 Linv = left();
  const Vector3 c = Linv * v;
  if (Hw) {
    Matrix3 Hf;
    J.applyLeft(c, Hf);  // derivative of forward mapping at c
    *Hw = -Linv * Hf;    // chain rule for inverse
  }
  if (Hv) *Hv = Linv;
  return c;
}

Vector3 InvJKernel::applyRight(const Vector3& v, OptionalJacobian<3, 3> Hw,
                               OptionalJacobian<3, 3> Hv) const {
  const Matrix3 Rinv = right();
  const Vector3 c = Rinv * v;
  if (Hw) {
    Matrix3 Hf;
    J.applyRight(c, Hf);  // derivative of forward right-mapping at c
    *Hw = -Rinv * Hf;     // chain rule for inverse
  }
  if (Hv) *Hv = Rinv;
  return c;
}

// --- Thresholds (class statics) ---
constexpr double Local::kNearZeroThresholdSq;
constexpr double Local::kNearPiThresholdSq;

// --- Local high-level helpers ---
Matrix3 Local::expmap() const {
  // Exp(ω) = I + A W + B W^2
  return I_3x3 + p_->A() * p_->W + p_->B() * p_->WW;
}

Kernel Local::Jacobian() const {
  // J_l/r share same coefficients; right flips b internally
  return Kernel{p_, 1.0, p_->B(), p_->C(), p_->dB(), p_->dC()};
}

InvJKernel Local::InvJacobian() const {
  // Algebraic inverse kernel (matrices only)
  return InvJKernel{p_, this->Jacobian()};
}

Kernel Local::Gamma() const {
  // Gamma = 1/2 I + C W + G W^2 (left); right flips b internally
  return Kernel{p_, 0.5, p_->C(), p_->E(), p_->dC(), p_->dE()};
}

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
// --- Backward-compatible functors (deprecated shims) ---
ExpmapFunctor::ExpmapFunctor(const Vector3& omega)
    : ExpmapFunctor(kNearZeroThresholdSq, omega) {}
ExpmapFunctor::ExpmapFunctor(double nz, const Vector3& omega)
    : Local(omega, nz) {}
ExpmapFunctor::ExpmapFunctor(const Vector3& axis, double angle)
    : Local(axis * angle) {}
DexpFunctor::DexpFunctor(const Vector3& omega, double nz, double np)
    : Local(omega, nz, np) {}
Matrix3 DexpFunctor::rightJacobian() const { return Jacobian().right(); }
Matrix3 DexpFunctor::leftJacobian() const { return Jacobian().left(); }
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
  return Expmap(axis * theta);
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
  so3::Local local(omega);
  if (H) *H = local.Jacobian().right();
  return SO3(local.expmap());
}

template <>
GTSAM_EXPORT
Matrix3 SO3::ExpmapDerivative(const Vector3& omega) {
  return so3::Local(omega).Jacobian().right();
}

//******************************************************************************
template <>
GTSAM_EXPORT
Matrix3 SO3::LogmapDerivative(const Vector3& omega) {
  return so3::Local(omega).InvJacobian().right();
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
    if (tr_3 < -so3::Local::kNearZeroThresholdSq) {
      // this is the normal case -1 < trace < 3
      double theta = acos((tr - 1.0) / 2.0);
      magnitude = theta / (2.0 * sin(theta));
    } else {
      // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      // see https://github.com/borglab/gtsam/issues/746 for details
      magnitude = 0.5 - tr_3 * so3::one_12th + tr_3 * tr_3 * so3::one_60th;
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
