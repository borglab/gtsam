/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Kernel.h
 * @brief   Specialized kernels for SO(3)
 * @author  Frank Dellaert
 * @date    August 2025
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/dllexport.h>

#include <limits>

namespace gtsam {
namespace so3 {

struct DexpFunctor;

/**
 * Kernel: M(ω) = a I + b Ω + c Ω² with radial derivatives db,dc for Fréchet.
 * Right variants flip b→-b, db→-db (no recompute of Ω/Ω²).
 * Keep a pointer to Local: Kernel methods above return a const & to prevent
 * having a pointer to a temporary.
 */
struct GTSAM_EXPORT Kernel {
  const DexpFunctor* S;
  double a{0}, b{0}, c{0}, db{0}, dc{0};  // left-specialization form

  Matrix3 left() const;   // a I + b Ω + c Ω²
  Matrix3 right() const;  // a I - b Ω + c Ω²

  Vector3 applyLeft(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                    OptionalJacobian<3, 3> Hv = {}) const;
  Vector3 applyRight(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                     OptionalJacobian<3, 3> Hv = {}) const;

  /// Fréchet derivative of left-kernel M(ω) in the direction X ∈ so(3)
  /// L_M(Ω)[X] = b X + c (Ω X + X Ω) + s (db Ω + dc Ω²), with s = -½ tr(Ω X)
  Matrix3 frechet(const Matrix3& X) const;
  /// Apply Fréchet derivative to vector (left specialization)
  Matrix3 applyFrechet(const Vector3& v) const;
};

// Stable inverse Jacobian kernel
struct GTSAM_EXPORT InvJKernel {
  const DexpFunctor* S;
  Kernel J;  // holds the forward kernel

  Matrix3 left() const;
  Matrix3 right() const;

  Vector3 applyLeft(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                    OptionalJacobian<3, 3> Hv = {}) const;
  Vector3 applyRight(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                     OptionalJacobian<3, 3> Hv = {}) const;
};

/// y + alpha * x  (functional)
GTSAM_EXPORT Kernel axpy(double alpha, const Kernel& X, const Kernel& Y);

// Blend K = α X + (1-α) Y with radial derivative (·)'/θ via dalpha
GTSAM_EXPORT Kernel blend(double alpha, double dalpha, const Kernel& X,
                          const Kernel& Y);

}  // namespace so3
}  // namespace gtsam