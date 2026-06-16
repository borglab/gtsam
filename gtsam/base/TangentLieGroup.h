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
 * This class builds on the semidirect ProductLieGroup.
 */

#pragma once

#include <gtsam/base/ProductLieGroup.h>

namespace gtsam {

/**
 * @brief Adjoint action of a Lie group G on its Lie algebra 𝔤 ≅ ℝⁿ.
 *
 *   φ(g, ξ) = Ad_g · ξ ,
 *
 * where generator(u) = ad_u = G::adjointMap(u).
 *
 * Requires G to expose a static adjointMap.
 */
template <typename G>
struct AdjointAction : public GroupAction<AdjointAction<G>, G,
                                          typename traits<G>::TangentVector> {
  static constexpr ActionType type = ActionType::Left;
  using TangentVector = typename traits<G>::TangentVector;
  static constexpr int n = traits<G>::dimension;

  TangentVector operator()(const G& g, const TangentVector& xi,
                           OptionalJacobian<n, n> Hg = {},
                           OptionalJacobian<n, n> Hxi = {}) const {
    const typename traits<G>::Jacobian Ad = traits<G>::AdjointMap(g);
    if (Hxi) *Hxi = Ad;                       // ∂(Ad_g ξ)/∂ξ = Ad_g
    if (Hg) *Hg = -(Ad * G::adjointMap(xi));  // ∂/∂g = -Ad_g · ad_ξ
    return Ad * xi;
  }

  /// Infinitesimal generator Aφ(u) = ad_u.
  static Eigen::Matrix<double, n, n> generator(const TangentVector& u) {
    return G::adjointMap(u);
  }
};

/**
 * @brief Tangent Lie group TG = G ⋉ 𝔤 (dimension 2·dim(G)).
 *
 * Element is a pair: .first ∈ G, .second ∈ 𝔤 ≅ ℝⁿ.
 * Group law: (g₁,ξ₁)·(g₂,ξ₂) = (g₁g₂, ξ₁ + Ad_{g₁}ξ₂).
 *
 * Example: using TGSE3 = TangentLieGroup<Pose3>;  // dimension 12
 */
template <typename G>
using TangentLieGroup =
    ProductLieGroup<G, typename traits<G>::TangentVector, AdjointAction<G>>;

}  // namespace gtsam
