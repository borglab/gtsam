/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file TangentLieGroup.h
 * @date June, 2026
 * @author Alessandro Fornasier
 * @brief Tangent group TG = G ⋉ 𝔤: the semidirect product of a Lie group G with
 * its own Lie algebra under the adjoint action φ(g,ξ)=Ad_g·ξ.
 *
 * GTSAM's Expmap derivative is the RIGHT Jacobian (D_G = φ₁(−ad)). For the
 * tangent group the action's transport kernel collapses onto the adjoint
 * kernel: φ₁(ad_u) = J_l^G(u). Built on the semidirect ProductLieGroup, so the
 * coupled off-diagonal Q-block of the chart Jacobian is produced automatically.
 */

#pragma once

#include <gtsam/base/ProductLieGroup.h>

namespace gtsam {

/**
 * @brief Adjoint action of a Lie group G on its Lie algebra 𝔤 ≅ ℝⁿ.
 *
 *   φ(g, ξ) = Ad_g · ξ ,   generator(u) = ad_u = G::adjointMap(u).
 *
 * Requires G to expose a static adjointMap (the algebra adjoint). Uses the
 * single Ad_g (spatial) convention.
 */
template <typename G>
struct AdjointAction
    : public GroupAction<AdjointAction<G>, G,
                         typename traits<G>::TangentVector> {
  static constexpr ActionType type = ActionType::Left;
  using TangentVector = typename traits<G>::TangentVector;
  static constexpr int n = traits<G>::dimension;

  TangentVector operator()(const G& g, const TangentVector& xi,
                           OptionalJacobian<n, n> Hg = {},
                           OptionalJacobian<n, n> Hxi = {}) const {
    const typename traits<G>::Jacobian Ad = traits<G>::AdjointMap(g);
    if (Hxi) *Hxi = Ad;                       // ∂(Ad_g ξ)/∂ξ = Ad_g
    if (Hg) *Hg = -(Ad * G::adjointMap(xi));  // ∂/∂g (right) = -Ad_g · ad_ξ
    return Ad * xi;
  }

  /// Infinitesimal generator A_Φ(u) = ad_u.
  static Eigen::Matrix<double, n, n> generator(const TangentVector& u) {
    return G::adjointMap(u);
  }
};

/**
 * @brief Tangent Lie group TG = G ⋉ 𝔤 (dimension 2·dim(G)).
 *
 * Element is a pair: .first ∈ G, .second ∈ 𝔤 ≅ ℝⁿ (the algebra vector).
 * Group law: (g₁,ξ₁)·(g₂,ξ₂) = (g₁g₂, ξ₁ + Ad_{g₁}ξ₂).
 *
 * Example: using TSE3 = TangentLieGroup<Pose3>;  // dimension 12
 */
template <typename G>
using TangentLieGroup =
    ProductLieGroup<G, typename traits<G>::TangentVector, AdjointAction<G>>;

}  // namespace gtsam
