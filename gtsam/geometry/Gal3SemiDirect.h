/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Gal3SemiDirect.h
 * @date June, 2026
 * @author Alessandro Fornasier
 * @brief Galilean group Gal(3) = SE(3) ⋉ ℝ⁴ built from the semidirect
 * ProductLieGroup. The SE(3) factor is (rotation, velocity-boost); the ℝ⁴
 * normal subgroup is (position, time). Equivalent to the native gtsam::Gal3.
 */

#pragma once

#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

/**
 * @brief SE(3) acting on ℝ⁴=(position p, time s): φ((R,ν),[p;s]) = [R·p+ν·s; s],
 * the 4×4 homogeneous matrix M(g)=[[R,ν],[0,1]] times [p;s]. Here the Pose3
 * translation slot holds the velocity-boost ν.
 */
struct SE3Vector4Action
    : public GroupAction<SE3Vector4Action, Pose3, Vector4> {
  static constexpr ActionType type = ActionType::Left;

  Vector4 operator()(const Pose3& g, const Vector4& h,
                     OptionalJacobian<4, 6> Hg = {},
                     OptionalJacobian<4, 4> Hh = {}) const {
    const Rot3& R = g.rotation();
    const Point3 nu = g.translation();  // velocity-boost
    const Vector3 p = h.head<3>();
    const double s = h(3);

    Matrix3 D_Rp_R, D_Rp_p;
    const Point3 Rp =
        R.rotate(p, Hg ? &D_Rp_R : nullptr, Hh ? &D_Rp_p : nullptr);

    Vector4 out;
    out << (Rp + s * nu), s;

    if (Hg) {
      // g·Exp([ω;ρ]) ≈ (R·Exp(ω̂), ν + R·ρ): ∂/∂ω = -R·p̂ (=D_Rp_R), ∂/∂ρ = s·R.
      Hg->setZero();
      Hg->topLeftCorner<3, 3>() = D_Rp_R;
      Hg->block<3, 3>(0, 3) = s * R.matrix();
    }
    if (Hh) {
      // ∂φ/∂h = M(g) = [[R, ν],[0,1]].
      Hh->setZero();
      Hh->topLeftCorner<3, 3>() = D_Rp_p;  // ∂/∂p = R
      Hh->block<3, 1>(0, 3) = nu;          // ∂/∂s = ν
      (*Hh)(3, 3) = 1.0;
    }
    return out;
  }

  /// Generator A_Φ([ω;ρ]) = 4×4 se(3) hat = [[ω̂, ρ],[0,0]].
  static Matrix4 generator(const Vector6& u) {
    Matrix4 A = Matrix4::Zero();
    A.topLeftCorner<3, 3>() = skewSymmetric(u.head<3>());
    A.block<3, 1>(0, 3) = u.tail<3>();
    return A;
  }
};

/// Gal(3) reconstructed from the semidirect product (dimension 10).
using SemidirectGal3 = ProductLieGroup<Pose3, Vector4, SE3Vector4Action>;

}  // namespace gtsam
