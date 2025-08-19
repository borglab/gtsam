/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Kernel.cpp
 * @brief   Specialized kernels for SO(3)
 * @author  Frank Dellaert
 * @date    August 2025
 */

#include <gtsam/geometry/Kernel.h>

#include <cmath>

namespace gtsam {
namespace so3 {

// Lightweight 3x3 inverse (adjugate / determinant), avoids Eigen's LU
// machinery. Assumes M is invertible. Used to keep link-time/code-size small on
// GCC/Ubuntu.
static inline Matrix3 inverse3x3(const Matrix3& M) {
  const double a = M(0, 0), b = M(0, 1), c = M(0, 2);
  const double d = M(1, 0), e = M(1, 1), f = M(1, 2);
  const double g = M(2, 0), h = M(2, 1), i = M(2, 2);

  const double A = (e * i - f * h);
  const double B = -(d * i - f * g);
  const double C = (d * h - e * g);
  const double D = -(b * i - c * h);
  const double E = (a * i - c * g);
  const double F = -(a * h - b * g);
  const double G = (b * f - c * e);
  const double H = -(a * f - c * d);
  const double I = (a * e - b * d);

  const double det = a * A + b * B + c * C;
  const double inv_det = 1.0 / det;

  Matrix3 adjT;  // transpose of cofactor matrix = adjugate^T
  adjT << A, D, G, B, E, H, C, F, I;
  return inv_det * adjT;
}

// --- Kernel matrices (out-of-line to keep header tight) ---
Matrix3 Kernel::left() const {
  // left: a I + b Ω + c Ω²
  return a * I_3x3 + b * S->W + c * S->WW;
}
Matrix3 Kernel::right() const {
  // right: a I - b Ω + c Ω²
  return a * I_3x3 - b * S->W + c * S->WW;
}

Vector3 Kernel::applyLeft(const Vector3& v, OptionalJacobian<3, 3> Hw,
                          OptionalJacobian<3, 3> Hv) const {
  const Vector3 Wv = S->omega.cross(v);
  const Vector3 WWv = S->omega.cross(Wv);
  if (Hw) {
    // Closed-form ∂/∂ω without doubleCross Jacobian:
    const auto& w = S->omega;
    const auto wt = w.transpose();
    const auto vt = v.transpose();
    const auto sI = w.dot(v) * I_3x3;
    *Hw = -b * skewSymmetric(v) +           // d(Ω v)/dω = -[v]×
          c * (w * vt + sI - 2 * v * wt) +  // d(Ω² v)/dω = ωvᵀ + (ω·v)I - 2vωᵀ
          (db * Wv + dc * WWv) * wt;        // radial derivative terms
  }
  if (Hv) *Hv = left();  // ∂y/∂v = a I + b Ω + c Ω²
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
  if (S->theta > M_PI) {
    // Above π the closed-form coefficients can suffer cancellation; invert
    // numerically but with a tiny, fixed-cost 3x3 adjugate instead of Eigen's
    // LU to avoid link-time bloat.
    return inverse3x3(J.left());
  }
  // Closed-form inverse in the {I, Ω, Ω²} basis
  return I_3x3 - 0.5 * S->W + S->D() * S->WW;
}
Matrix3 InvJKernel::right() const {
  if (S->theta > M_PI) {
    return inverse3x3(J.right());
  }
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

Kernel axpy(double alpha, const Kernel& X, const Kernel& Y) {
  return Kernel{Y.S,
                Y.a + alpha * X.a,
                Y.b + alpha * X.b,
                Y.c + alpha * X.c,
                Y.db + alpha * X.db,
                Y.dc + alpha * X.dc};
}

Kernel blend(double alpha, double dalpha, const Kernel& X, const Kernel& Y) {
  const double beta = 1.0 - alpha;
  // L has db=dc=0; derivative comes from α only
  return Kernel{
      Y.S,
      alpha * X.a + beta * Y.a,
      alpha * X.b + beta * Y.b,
      alpha * X.c + beta * Y.c,
      dalpha * (X.b - Y.b) + beta * Y.db,  // db
      dalpha * (X.c - Y.c) + beta * Y.dc   // dc
  };
}

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
static constexpr double k1_4Pi = 0.25 * kPi_inv;  // 1/(4·π)

Local::Local(const Vector3& omega, double nz, double np)
    : omega(omega),
      theta2(omega.dot(omega)),
      theta(std::sqrt(theta2)),
      W(skewSymmetric(omega)),
      WW(W * W),
      nearZero(theta2 <= nz) {
  const double delta = M_PI > theta ? (M_PI - theta) : 0.0;
  const double delta2 = delta * delta;
  nearPi = (delta2 < np);
  if (!nearZero) {
    A = std::sin(theta) / theta;
    const double sinHalfTheta = std::sin(0.5 * theta);
    B = (2.0 * sinHalfTheta * sinHalfTheta) / theta2;
    C = (1.0 - A) / theta2;
  } else {
    A = 1.0 - theta2 * one_6th;
    B = 0.5 - theta2 * one_24th;
    C = one_6th - theta2 * one_120th;
  }
}

double Local::D() const {
  if (std::isnan(D_)) {
    D_ = !nearZero ? (nearPi ? (k1_Pi2 + (k2_Pi3 - k1_4Pi) * (M_PI - theta))
                             : ((1.0 - A / (2.0 * B)) / theta2))
                   : (one_12th + theta2 * one_720th);
  }
  return D_;
}

double Local::E() const {
  if (std::isnan(E_)) {
    E_ = !nearZero ? ((1.0 - 2.0 * B) / (2.0 * theta2))
                   : (one_24th - theta2 * one_720th);
  }
  return E_;
}

double Local::dA() const {
  if (std::isnan(dA_)) {
    // Identity: dA = A′/θ = C − B (valid for all θ, with our near-zero series)
    dA_ = C - B;
  }
  return dA_;
}

double Local::dB() const {
  if (std::isnan(dB_)) {
    dB_ =
        !nearZero ? ((A - 2.0 * B) / theta2) : (-one_12th + theta2 * one_180th);
  }
  return dB_;
}

double Local::dC() const {
  if (std::isnan(dC_)) {
    dC_ = !nearZero ? ((B - 3.0 * C) / theta2)
                    : (-one_60th + theta2 * one_1260th);
  }
  return dC_;
}

double Local::dE() const {
  if (std::isnan(dE_)) {
    dE_ = !nearZero ? (-(dB() + 2.0 * E()) / theta2) : (-one_360th);
  }
  return dE_;
}

// --- Kernels ---
Kernel Local::Rodrigues() const& { return Kernel{this, 1.0, A, B, dA(), dB()}; }
Kernel Local::Jacobian() const& { return Kernel{this, 1.0, B, C, dB(), dC()}; }
InvJKernel Local::InvJacobian() const& { return InvJKernel{this, Jacobian()}; }
Kernel Local::Gamma() const& { return Kernel{this, 0.5, C, E(), dC(), dE()}; }

// --- Backward-compatible functors (deprecated shims) ---
ExpmapFunctor::ExpmapFunctor(const Vector3& omega)
    : ExpmapFunctor(kNearZeroThresholdSq, omega) {}
ExpmapFunctor::ExpmapFunctor(double nz, const Vector3& omega)
    : Local(omega, nz) {}
ExpmapFunctor::ExpmapFunctor(const Vector3& axis, double angle)
    : Local(axis * angle) {}
DexpFunctor::DexpFunctor(const Vector3& omega, double nz, double np)
    : ExpmapFunctor(nz, omega) {
  const double delta = M_PI > theta ? (M_PI - theta) : 0.0;
  const double delta2 = delta * delta;
  nearPi = (delta2 < np);
}
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

}  // namespace so3
}  // namespace gtsam
