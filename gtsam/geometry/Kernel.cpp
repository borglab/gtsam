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
#include <gtsam/geometry/SO3.h>

namespace gtsam {
namespace so3 {

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
  const Vector3 Wv = S->omega.cross(v);
  const Vector3 WWv = S->omega.cross(Wv);
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

}  // namespace so3
}  // namespace gtsam
