/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ActionProductLieGroup.h
 * @date April, 2026
 * @author Frank Dellaert
 * @brief Product Lie group parameterized by a left group action
 */

#pragma once

#include <gtsam/base/GroupAction.h>
#include <gtsam/base/ProductLieGroup.h>

#include <type_traits>

namespace gtsam {

template <typename G, typename H>
struct DirectProductAction;

namespace action_product_lie_group_detail {

template <typename Action, typename G, typename H, typename = void>
struct IsStatelessLeftActionPolicy : std::false_type {};

template <typename Action, typename G, typename H>
struct IsStatelessLeftActionPolicy<Action, G, H,
                                   std::void_t<decltype(Action::type)>>
    : std::bool_constant<
          std::is_empty_v<Action> && std::is_default_constructible_v<Action> &&
          std::is_base_of_v<GroupAction<Action, G, H>, Action> &&
          Action::type == ActionType::Left &&
          std::is_invocable_r_v<
              H, const Action&, const G&, const H&,
              OptionalJacobian<traits<H>::dimension, traits<G>::dimension>,
              OptionalJacobian<traits<H>::dimension, traits<H>::dimension>>> {
};

template <typename G, typename H>
using ProductChartJacobian = std::conditional_t<
    traits<G>::dimension == Eigen::Dynamic ||
        traits<H>::dimension == Eigen::Dynamic,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>,
    OptionalJacobian<traits<G>::dimension + traits<H>::dimension,
                     traits<G>::dimension + traits<H>::dimension>>;

template <typename G, typename H>
using ProductTangentVector = std::conditional_t<
    traits<G>::dimension == Eigen::Dynamic ||
        traits<H>::dimension == Eigen::Dynamic,
    Vector,
    Eigen::Matrix<double, traits<G>::dimension + traits<H>::dimension, 1>>;

template <typename G, typename H>
using ProductJacobian = std::conditional_t<
    traits<G>::dimension == Eigen::Dynamic ||
        traits<H>::dimension == Eigen::Dynamic,
    Matrix,
    Eigen::Matrix<double, traits<G>::dimension + traits<H>::dimension,
                  traits<G>::dimension + traits<H>::dimension>>;

template <typename Product, typename Action, typename G, typename H,
          typename = void>
struct HasSemidirectExpmap : std::false_type {};

template <typename Product, typename Action, typename G, typename H>
struct HasSemidirectExpmap<
    Product, Action, G, H,
    std::void_t<decltype(Action::template Expmap<Product>(
        std::declval<const typename traits<G>::TangentVector&>(),
        std::declval<const typename traits<H>::TangentVector&>(),
        std::declval<OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>>(),
        std::declval<OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>>()))>>
    : std::bool_constant<std::is_convertible_v<
          decltype(Action::template Expmap<Product>(
              std::declval<const typename traits<G>::TangentVector&>(),
              std::declval<const typename traits<H>::TangentVector&>(),
              std::declval<OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>>(),
              std::declval<OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>>())),
          Product>> {};

template <typename Product, typename Action, typename G, typename H,
          typename = void>
struct HasSemidirectLogmap : std::false_type {};

template <typename Product, typename Action, typename G, typename H>
struct HasSemidirectLogmap<
    Product, Action, G, H,
    std::void_t<decltype(Action::template Logmap<Product>(
        std::declval<const Product&>(),
        std::declval<ProductChartJacobian<G, H>>()))>>
    : std::bool_constant<std::is_convertible_v<
          decltype(Action::template Logmap<Product>(
              std::declval<const Product&>(),
              std::declval<ProductChartJacobian<G, H>>())),
          ProductTangentVector<G, H>>> {};

template <typename Product, typename Action, typename G, typename H,
          typename = void>
struct HasSemidirectAdjointMap : std::false_type {};

template <typename Product, typename Action, typename G, typename H>
struct HasSemidirectAdjointMap<
    Product, Action, G, H,
    std::void_t<decltype(Action::template AdjointMap<Product>(
        std::declval<const Product&>()))>>
    : std::bool_constant<std::is_convertible_v<
          decltype(Action::template AdjointMap<Product>(
              std::declval<const Product&>())),
          ProductJacobian<G, H>>> {};

}  // namespace action_product_lie_group_detail

template <typename G, typename H>
struct DirectProductAction
    : public GroupAction<DirectProductAction<G, H>, G, H> {
  static constexpr ActionType type = ActionType::Left;

  H operator()(const G& g, const H& h,
               OptionalJacobian<traits<H>::dimension, traits<G>::dimension> Hg =
                   {},
               OptionalJacobian<traits<H>::dimension, traits<H>::dimension> Hh =
                   {}) const {
    if (Hg) {
      if constexpr (traits<H>::dimension == Eigen::Dynamic ||
                    traits<G>::dimension == Eigen::Dynamic) {
        Hg->setZero(static_cast<Eigen::Index>(traits<H>::GetDimension(h)),
                    static_cast<Eigen::Index>(traits<G>::GetDimension(g)));
      } else {
        Hg->setZero();
      }
    }
    if (Hh) {
      if constexpr (traits<H>::dimension == Eigen::Dynamic) {
        const Eigen::Index hDim =
            static_cast<Eigen::Index>(traits<H>::GetDimension(h));
        Hh->setIdentity(hDim, hDim);
      } else {
        Hh->setIdentity();
      }
    }
    return h;
  }
};

/**
 * @brief Product Lie group extended with an optional left action.
 * If Action is omitted the group reduces to the direct product G x H.
 * If Action is provided the group becomes the semidirect product G ⋉ H.
 */
template <typename G, typename H,
          typename Action = DirectProductAction<G, H>>
class ActionProductLieGroup : public ProductLieGroup<G, H> {
  GTSAM_CONCEPT_ASSERT(IsLieGroup<G>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<H>);
  GTSAM_CONCEPT_ASSERT(IsTestable<G>);
  GTSAM_CONCEPT_ASSERT(IsTestable<H>);

 public:
  using This = ActionProductLieGroup<G, H, Action>;
  using FirstFactor = G;
  using SecondFactor = H;
  using ActionPolicy = Action;
  using Base = ProductLieGroup<G, H>;
  using PairBase = typename Base::Base;
  using typename Base::ChartJacobian;
  using typename Base::Jacobian;
  using typename Base::Jacobian1;
  using typename Base::Jacobian2;
  using typename Base::TangentVector;

 protected:
  inline constexpr static int dimension1 = Base::dimension1;
  inline constexpr static int dimension2 = Base::dimension2;
  inline constexpr static bool firstDynamic = Base::firstDynamic;
  inline constexpr static bool secondDynamic = Base::secondDynamic;
  inline constexpr static bool isDirectProduct =
      std::is_same_v<Action, DirectProductAction<G, H>>;

 public:
  inline constexpr static int dimension = Base::dimension;
  inline constexpr static int manifoldDimension = dimension;
  using DynamicJacobian = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;

  static_assert(action_product_lie_group_detail::IsStatelessLeftActionPolicy<
                    Action, G, H>::value,
                "ActionProductLieGroup action must be a stateless left "
                "GroupAction policy on H.");

  typedef multiplicative_group_tag group_flavor;

  ActionProductLieGroup() : Base() {}
  ActionProductLieGroup(const G& g, const H& h) : Base(g, h) {}
  ActionProductLieGroup(const PairBase& base) : Base(base) {}
  ActionProductLieGroup(const Base& base) : Base(base) {}

  static This Identity() { return This(); }
  static constexpr int Dim() { return manifoldDimension; }
  size_t dim() const { return Base::dim(); }

  This operator*(const This& other) const;
  This inverse() const;

  This compose(const This& g) const { return (*this) * g; }
  This between(const This& g) const { return this->inverse() * g; }

  This retract(const TangentVector& v, ChartJacobian H1 = {},
               ChartJacobian H2 = {}) const;
  TangentVector localCoordinates(const This& g, ChartJacobian H1 = {},
                                 ChartJacobian H2 = {}) const;

  This compose(const This& other, ChartJacobian H1,
               ChartJacobian H2 = {}) const;
  This between(const This& other, ChartJacobian H1,
               ChartJacobian H2 = {}) const;
  This inverse(ChartJacobian D) const;

  static This Expmap(const TangentVector& v, ChartJacobian Hv = {});
  static This Expmap(
      const Eigen::Ref<const typename traits<G>::TangentVector>& v1,
      const Eigen::Ref<const typename traits<H>::TangentVector>& v2,
      DynamicJacobian H1 = {}, DynamicJacobian H2 = {});
  static TangentVector Logmap(const This& p, ChartJacobian Hp = {});
  static TangentVector LocalCoordinates(const This& p, ChartJacobian Hp = {}) {
    return Logmap(p, Hp);
  }

  This expmap(const TangentVector& v) const;
  TangentVector logmap(const This& g) const { return This::Logmap(between(g)); }
  Jacobian AdjointMap() const;

  void print(const std::string& s = "") const;
  bool equals(const This& other, double tol = 1e-9) const {
    return Base::equals(other, tol);
  }

 protected:
  static size_t combinedDimension(size_t d1, size_t d2) {
    return Base::combinedDimension(d1, d2);
  }

  template <typename T, int Dim = traits<T>::dimension>
  static typename traits<T>::TangentVector tangentSegment(
      const TangentVector& v, size_t start, size_t runtimeDimension) {
    return Base::template tangentSegment<T, Dim>(v, start, runtimeDimension);
  }

  static TangentVector makeTangentVector(
      const typename traits<G>::TangentVector& v1,
      const typename traits<H>::TangentVector& v2, size_t firstDimension,
      size_t secondDimension) {
    return Base::makeTangentVector(v1, v2, firstDimension, secondDimension);
  }

  static Jacobian zeroJacobian(size_t productDimension) {
    return Base::zeroJacobian(productDimension);
  }

  static Jacobian identityJacobian(size_t productDimension) {
    return Base::identityJacobian(productDimension);
  }

  void checkMatchingDimensions(const This& other, const char* operation) const;
};

template <typename G, typename H, typename Action>
struct traits<ActionProductLieGroup<G, H, Action>>
    : internal::LieGroup<ActionProductLieGroup<G, H, Action>> {};

}  // namespace gtsam

#include <gtsam/base/ActionProductLieGroup-inl.h>
