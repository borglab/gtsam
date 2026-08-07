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
 * @author Frank Dellaert
 * @brief Tangent group TG = G ⋉ 𝔤 under the adjoint action.
 */

#pragma once

#include <gtsam/base/GroupAction.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Testable.h>

#include <iostream>
#include <string>
#include <type_traits>
#include <utility>

namespace gtsam {
namespace internal {

/**
 * Enforces the static algebra-adjoint requirement of TangentLieGroup and
 * AdjointAction below. For example, Pose3 satisfies this through
 * Pose3::adjointMap().
 */
template <typename T, typename = void>
struct TangentLieGroupHasAdjointMap : std::false_type {};
template <typename T>
struct TangentLieGroupHasAdjointMap<
    T, std::void_t<decltype(T::adjointMap(
           std::declval<const typename traits<T>::TangentVector&>()))>>
    : std::true_type {};

/**
 * Optional closed-form kernels used by `TangentLieGroup::Expmap()` and its
 * private `rightJacobian()` helper. Rot3 specializes this trait in Rot3.h to
 * reuse the same SO(3) kernel as Pose3; other groups use the structured
 * augmented exponential in TangentLieGroup-inl.h.
 */
template <typename G>
struct TangentLieGroupJacobian {
  static constexpr bool available = false;
  static constexpr bool expmapAvailable = false;
};

}  // namespace internal

/** The adjoint action phi(g,v) = Ad_g v, also useful independently. */
template <typename G>
struct AdjointAction : public GroupAction<AdjointAction<G>, G,
                                          typename traits<G>::TangentVector> {
  static constexpr ActionType type = ActionType::Left;
  using TangentVector = typename traits<G>::TangentVector;
  static constexpr int n = traits<G>::dimension;
  static_assert(internal::TangentLieGroupHasAdjointMap<G>::value,
                "AdjointAction<G> requires G::adjointMap(TangentVector)");

  TangentVector operator()(const G& g, const TangentVector& v,
                           OptionalJacobian<n, n> Hg = {},
                           OptionalJacobian<n, n> Hv = {}) const {
    const typename traits<G>::Jacobian Ad = traits<G>::AdjointMap(g);
    if (Hv) *Hv = Ad;
    if (Hg) *Hg = -(Ad * G::adjointMap(v));
    return Ad * v;
  }

  static typename traits<G>::Jacobian generator(const TangentVector& u) {
    return G::adjointMap(u);
  }
};

/**
 * @brief Tangent Lie group TG = G ⋉ 𝔤, with dimension 2 dim(G).
 *
 * Elements are pairs (g,v), with product
 *   (g1,v1)(g2,v2) = (g1 g2, v1 + Ad_g1 v2).
 * This dedicated class exposes the tangent structure directly, allowing its
 * repeated-block Jacobians and adjoints to be assembled without a generic
 * semidirect-action layer. Standard Lie-group operations and their Jacobians
 * are inherited from `LieGroup`; this class supplies the tangent-group law,
 * charts, adjoints, and specialized kernels.
 */
template <typename G>
class TangentLieGroup
    : public std::pair<G, typename traits<G>::TangentVector>,
      public LieGroup<TangentLieGroup<G>,
                      internal::dimensionProduct(2, traits<G>::dimension)> {
  GTSAM_CONCEPT_ASSERT(IsLieGroup<G>);
  GTSAM_CONCEPT_ASSERT(IsTestable<G>);
  static_assert(traits<G>::dimension != Eigen::Dynamic,
                "TangentLieGroup requires a fixed-dimensional base group");
  static_assert(internal::TangentLieGroupHasAdjointMap<G>::value,
                "TangentLieGroup requires G::adjointMap(TangentVector)");

 private:
  inline constexpr static int n = traits<G>::dimension;

 public:
  using This = TangentLieGroup<G>;
  using Base = std::pair<G, typename traits<G>::TangentVector>;
  using LieBase = LieGroup<This, internal::dimensionProduct(2, n)>;
  using LieBase::Dim;
  using LieBase::dimension;
  using BaseTangent = typename traits<G>::TangentVector;
  using BaseJacobian = typename traits<G>::Jacobian;
  using TangentVector = typename LieBase::TangentVector;
  using Jacobian = typename LieBase::Jacobian;
  using ChartJacobian = typename LieBase::ChartJacobian;
  using ChartAtOrigin = internal::ChartAtIdentity<This, dimension>;
  using SplitJacobian = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;
  using group_flavor = multiplicative_group_tag;

  using LieBase::between;
  using LieBase::compose;
  using LieBase::expmap;
  using LieBase::inverse;
  using LieBase::logmap;

  TangentLieGroup() : Base(traits<G>::Identity(), BaseTangent::Zero()) {}
  TangentLieGroup(const G& g, const BaseTangent& v) : Base(g, v) {}
  TangentLieGroup(const Base& base) : Base(base) {}

  static TangentLieGroup Identity() { return TangentLieGroup(); }
  constexpr size_t dim() const { return dimension; }

  TangentLieGroup operator*(const TangentLieGroup& other) const;
  TangentLieGroup inverse() const;
  TangentLieGroup retract(const TangentVector& xi, ChartJacobian H1 = {},
                          ChartJacobian H2 = {}) const;
  TangentVector localCoordinates(const TangentLieGroup& other,
                                 ChartJacobian H1 = {},
                                 ChartJacobian H2 = {}) const;

  static TangentLieGroup Expmap(const TangentVector& xi, ChartJacobian H = {});
  static TangentLieGroup Expmap(const Eigen::Ref<const BaseTangent>& u,
                                const Eigen::Ref<const BaseTangent>& v,
                                SplitJacobian H1 = {}, SplitJacobian H2 = {});
  static TangentVector Logmap(const TangentLieGroup& p, ChartJacobian H = {});
  static TangentVector LocalCoordinates(const TangentLieGroup& p,
                                        ChartJacobian H = {}) {
    return Logmap(p, H);
  }

  Jacobian AdjointMap() const;
  static Jacobian adjointMap(const TangentVector& xi);

  void print(const std::string& s = "") const;
  bool equals(const TangentLieGroup& other, double tol = 1e-9) const {
    return traits<G>::Equals(this->first, other.first, tol) &&
           traits<BaseTangent>::Equals(this->second, other.second, tol);
  }

 private:
  static std::pair<BaseTangent, BaseTangent> split(const TangentVector& xi);
  static TangentVector join(const BaseTangent& u, const BaseTangent& v);
  static Jacobian rightJacobian(const TangentVector& xi);
};

template <typename G>
struct traits<TangentLieGroup<G>> : internal::LieGroup<TangentLieGroup<G>> {};

}  // namespace gtsam

#include <gtsam/base/TangentLieGroup-inl.h>
