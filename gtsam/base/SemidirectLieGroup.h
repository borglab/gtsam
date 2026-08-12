/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SemidirectLieGroup.h
 * @date August, 2026
 * @author Frank Dellaert
 * @author Rohan Bansal
 * @author Alessandro Fornasier
 * @brief A Lie group formed from a left action of G on a vector space H.
 */

#pragma once

#include <gtsam/base/GroupAction.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Testable.h>

#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace gtsam {
namespace internal {

/**
 * Recognizes the vector-space second factor required by SemidirectLieGroup.
 * Consulted by the class-level compatibility assertion below; for example,
 * Vector3 is accepted while Pose3 is not.
 */
template <typename T>
struct SemidirectLieGroupIsVector : std::false_type {};
template <int N>
struct SemidirectLieGroupIsVector<Eigen::Matrix<double, N, 1>>
    : std::true_type {};

/**
 * Detects the optional static algebra adjoint on the base group.
 * `SemidirectLieGroup::Expmap()` and `Logmap()` use the complete right-
 * Jacobian kernel when this is true, and their directional Fréchet fallback
 * otherwise.
 */
template <typename T, typename = void>
struct SemidirectLieGroupHasAdjointMap : std::false_type {};
template <typename T>
struct SemidirectLieGroupHasAdjointMap<
    T, std::void_t<decltype(T::adjointMap(
           std::declval<const typename traits<T>::TangentVector&>()))>>
    : std::true_type {};

/**
 * Validates the action contract used by SemidirectLieGroup's group law and
 * matrix-function kernels. The class-level assertion below points failures at
 * the declaration rather than at a later Expmap instantiation.
 */
template <typename Action, typename G, typename H, typename = void>
struct IsCompatibleSemidirectAction : std::false_type {};
template <typename Action, typename G, typename H>
struct IsCompatibleSemidirectAction<
    Action, G, H,
    std::void_t<decltype(Action::type),
                decltype(Action::generator(
                    std::declval<const typename traits<G>::TangentVector&>()))>>
    : std::bool_constant<std::is_base_of_v<GroupAction<Action, G, H>, Action> &&
                         std::is_default_constructible_v<Action> &&
                         Action::type == ActionType::Left> {};

}  // namespace internal

/**
 * @brief Left semidirect product G ⋉ H induced by Action.
 *
 * The group law is
 *   (g1,h1)(g2,h2) = (g1 g2, h1 Action(g1,h2)).
 * H must be a fixed-size Eigen vector, and Action must be a default-
 * constructible left GroupAction with an infinitesimal generator.
 * Standard Lie-group operations and their Jacobians are inherited from
 * `LieGroup`; this class supplies the semidirect law, action-specific charts,
 * adjoints, and augmented-matrix exponential/logarithmic kernels.
 */
template <typename G, typename H, typename Action>
class SemidirectLieGroup
    : public std::pair<G, H>,
      public LieGroup<SemidirectLieGroup<G, H, Action>,
                      internal::dimensionSum(traits<G>::dimension,
                                             traits<H>::dimension)> {
  GTSAM_CONCEPT_ASSERT(IsLieGroup<G>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<H>);
  GTSAM_CONCEPT_ASSERT(IsTestable<G>);
  GTSAM_CONCEPT_ASSERT(IsTestable<H>);

 public:
  using This = SemidirectLieGroup<G, H, Action>;
  using Base = std::pair<G, H>;
  using LieBase = LieGroup<This, internal::dimensionSum(traits<G>::dimension,
                                                        traits<H>::dimension)>;

 protected:
  inline constexpr static int n = traits<G>::dimension;
  inline constexpr static int m = traits<H>::dimension;
  inline constexpr static bool firstDynamic = n == Eigen::Dynamic;
  static_assert(internal::IsCompatibleSemidirectAction<Action, G, H>::value &&
                    internal::SemidirectLieGroupIsVector<H>::value &&
                    m != Eigen::Dynamic,
                "SemidirectLieGroup requires a default-constructible left "
                "GroupAction, a fixed-size Eigen vector H, and "
                "Action::generator(u)");

 public:
  using LieBase::Dim;
  using LieBase::dimension;
  using TangentVector = typename LieBase::TangentVector;
  using ChartJacobian = typename LieBase::ChartJacobian;
  using Jacobian = typename LieBase::Jacobian;
  using ChartAtOrigin = internal::ChartAtIdentity<This, dimension>;
  using Jacobian1 = typename traits<G>::Jacobian;
  using Jacobian2 = typename traits<H>::Jacobian;
  using ActionJacobianG =
      std::conditional_t<firstDynamic, Matrix, Eigen::Matrix<double, m, n>>;

  using group_flavor = multiplicative_group_tag;

  SemidirectLieGroup() : Base(defaultIdentity<G>(), traits<H>::Identity()) {}
  SemidirectLieGroup(const G& g, const H& h) : Base(g, h) {}
  SemidirectLieGroup(const Base& base) : Base(base) {}

  static SemidirectLieGroup Identity() { return SemidirectLieGroup(); }
  size_t dim() const { return firstDim() + secondDim(); }

  SemidirectLieGroup operator*(const SemidirectLieGroup& other) const;
  SemidirectLieGroup inverse() const;
  using LieBase::between;
  using LieBase::compose;
  using LieBase::expmap;
  using LieBase::inverse;
  using LieBase::logmap;

  SemidirectLieGroup retract(const TangentVector& v, ChartJacobian H1 = {},
                             ChartJacobian H2 = {}) const;
  TangentVector localCoordinates(const SemidirectLieGroup& other,
                                 ChartJacobian H1 = {},
                                 ChartJacobian H2 = {}) const;
  static SemidirectLieGroup Expmap(const TangentVector& xi,
                                   ChartJacobian D = {});
  static SemidirectLieGroup Expmap(
      const Eigen::Ref<const typename traits<G>::TangentVector>& u,
      const Eigen::Ref<const typename traits<H>::TangentVector>& v,
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = {},
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = {});
  static TangentVector Logmap(const SemidirectLieGroup& p,
                              ChartJacobian D = {});
  static TangentVector LocalCoordinates(const SemidirectLieGroup& p,
                                        ChartJacobian D = {}) {
    return Logmap(p, D);
  }
  Jacobian AdjointMap() const;
  static Jacobian adjointMap(const TangentVector& xi);

  void print(const std::string& s = "") const;
  bool equals(const SemidirectLieGroup& other, double tol = 1e-9) const {
    return traits<G>::Equals(this->first, other.first, tol) &&
           traits<H>::Equals(this->second, other.second, tol);
  }

 private:
  template <typename T>
  static T defaultIdentity();
  size_t firstDim() const { return traits<G>::GetDimension(this->first); }
  size_t secondDim() const { return traits<H>::GetDimension(this->second); }
  void checkMatchingDimensions(const SemidirectLieGroup& other,
                               const char* operation) const;

  template <typename T, int D = traits<T>::dimension>
  static typename traits<T>::TangentVector tangentSegment(
      const TangentVector& xi, size_t start, size_t d);
  static TangentVector makeTangentVector(
      const typename traits<G>::TangentVector& u,
      const typename traits<H>::TangentVector& v, size_t d1, size_t d2);
  static Jacobian zeroJacobian(size_t d);
  static Jacobian identityJacobian(size_t d);

  struct Phi1KernelResult {
    Jacobian2 phi0;
    Jacobian2 phi1;
  };
  static Phi1KernelResult phi1Kernel(const Jacobian2& A);
  static typename traits<H>::TangentVector phi1FrechetAction(
      const Jacobian2& A, const Jacobian2& B,
      const typename traits<H>::TangentVector& v);
  static Jacobian rightJacobian(const TangentVector& xi);
};

template <typename G, typename H, typename Action>
struct traits<SemidirectLieGroup<G, H, Action>>
    : internal::LieGroup<SemidirectLieGroup<G, H, Action>> {};

}  // namespace gtsam

#include <gtsam/base/SemidirectLieGroup-inl.h>
