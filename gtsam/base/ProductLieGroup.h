/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ProductLieGroup.h
 * @date May, 2015
 * @author Frank Dellaert
 * @author Rohan Bansal
 * @author Alessandro Fornasier
 * @brief Group product of two Lie Groups
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Testable.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>  // pair
#include <vector>

namespace gtsam {

namespace internal {

/**
 * Detects vector-space Lie groups (Eigen column vectors, group law = addition).
 *
 * Used by `ProductLieGroup::adjointMap()` for abelian fixed- and dynamic-size
 * vector factors.
 * For example, both `Vector3` and dynamic `Vector` match this trait, so their
 * algebra-adjoint blocks are assembled as zero matrices.
 */
template <typename T>
struct ProductLieGroupIsVector : std::false_type {};
template <int N>
struct ProductLieGroupIsVector<Eigen::Matrix<double, N, 1>> : std::true_type {};

}  // namespace internal

/**
 * @brief Direct product Lie group G × H.
 *
 * Operations are componentwise and tangent vectors concatenate the G and H
 * coordinates. Use `SemidirectLieGroup` when the first factor acts on the
 * second, and `TangentLieGroup` for the adjoint-action tangent construction.
 * Standard Lie-group operations and their Jacobians are inherited from
 * `LieGroup`; this class supplies the direct-product law, component charts,
 * adjoints, and exponential/logarithmic kernels.
 */
template <typename G, typename H>
class ProductLieGroup
    : public std::pair<G, H>,
      public LieGroup<ProductLieGroup<G, H>,
                      internal::dimensionSum(traits<G>::dimension,
                                             traits<H>::dimension)> {
  GTSAM_CONCEPT_ASSERT(IsLieGroup<G>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<H>);
  GTSAM_CONCEPT_ASSERT(IsTestable<G>);
  GTSAM_CONCEPT_ASSERT(IsTestable<H>);

 public:
  using This = ProductLieGroup<G, H>;
  /// Base pair type
  using Base = std::pair<G, H>;
  using LieBase = LieGroup<This, internal::dimensionSum(traits<G>::dimension,
                                                        traits<H>::dimension)>;

 protected:
  /// Dimensions of the two subgroups
  inline constexpr static int n = traits<G>::dimension;
  inline constexpr static int m = traits<H>::dimension;
  inline constexpr static bool firstDynamic = n == Eigen::Dynamic;
  inline constexpr static bool secondDynamic = m == Eigen::Dynamic;

 public:
  using LieBase::Dim;
  using LieBase::dimension;
  using TangentVector = typename LieBase::TangentVector;
  using ChartJacobian = typename LieBase::ChartJacobian;
  using Jacobian = typename LieBase::Jacobian;
  using Jacobian1 = typename traits<G>::Jacobian;
  using Jacobian2 = typename traits<H>::Jacobian;

  /**
   * Component-wise chart at the product identity.
   *
   * For a dynamic product, `Identity()` cannot retain the runtime dimension of
   * a dynamic factor. `Retract` therefore obtains a correctly sized identity
   * by exponentiating a zero tangent before applying the component charts.
   * This is deliberately not the same as returning `Expmap(v)`: a component's
   * retract chart may differ from its exponential map (for example, Rot3 when
   * configured to use the Cayley chart).
   *
   * If both factors are dynamic, a single concatenated tangent does not reveal
   * their split; `Expmap` supplies the existing diagnostic for that case.
   * `Local` can always recover both dimensions from the supplied value.
   */
  struct ChartAtOrigin {
    static This Retract(const TangentVector& v, ChartJacobian Hv = {}) {
      if constexpr (dimension == Eigen::Dynamic) {
        const This identity = This::Expmap(TangentVector::Zero(v.size()));
        return identity.retract(v, {}, Hv);
      } else {
        return This::Identity().retract(v, {}, Hv);
      }
    }

    static TangentVector Local(const This& value, ChartJacobian Hv = {}) {
      if constexpr (dimension == Eigen::Dynamic) {
        const This identity(traits<G>::Between(value.first, value.first),
                            traits<H>::Between(value.second, value.second));
        return identity.localCoordinates(value, {}, Hv);
      } else {
        return This::Identity().localCoordinates(value, {}, Hv);
      }
    }
  };

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor yields identity
  ProductLieGroup() : Base(defaultIdentity<G>(), defaultIdentity<H>()) {}

  /// Construct from two subgroup elements
  ProductLieGroup(const G& g, const H& h) : Base(g, h) {}

  /// Construct from base pair
  ProductLieGroup(const Base& base) : Base(base) {}

  /// @}
  /// @name Group Operations
  /// @{

  typedef multiplicative_group_tag group_flavor;

  /// Identity element
  static ProductLieGroup Identity() { return ProductLieGroup(); }

  /// Group multiplication
  ProductLieGroup operator*(const ProductLieGroup& other) const;

  /// Group inverse
  ProductLieGroup inverse() const;

  using LieBase::between;
  using LieBase::compose;
  using LieBase::expmap;
  using LieBase::inverse;
  using LieBase::logmap;

  /// @}
  /// @name Manifold Operations
  /// @{

  /// Return manifold dimension
  size_t dim() const { return firstDim() + secondDim(); }

  /// Retract to manifold
  ProductLieGroup retract(const TangentVector& v, ChartJacobian H1 = {},
                          ChartJacobian H2 = {}) const;

  /// Local coordinates on manifold
  TangentVector localCoordinates(const ProductLieGroup& g,
                                 ChartJacobian H1 = {},
                                 ChartJacobian H2 = {}) const;

  /// @}
  /// @name Lie Group Operations
  /// @{

  /// Exponential map
  static ProductLieGroup Expmap(const TangentVector& v, ChartJacobian Hv = {});

  /// Exponential map from subgroup tangent vectors
  static ProductLieGroup Expmap(
      const Eigen::Ref<const typename traits<G>::TangentVector>& v1,
      const Eigen::Ref<const typename traits<H>::TangentVector>& v2,
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = {},
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = {});

  /// Logarithmic map
  static TangentVector Logmap(const ProductLieGroup& p, ChartJacobian Hp = {});

  /// Local coordinates (same as Logmap)
  static TangentVector LocalCoordinates(const ProductLieGroup& p,
                                        ChartJacobian Hp = {}) {
    return Logmap(p, Hp);
  }

  /// Adjoint map
  Jacobian AdjointMap() const;

  /**
   * Static Lie-algebra adjoint ad_xi.
   * Vector-space factors are treated as abelian; other factors must provide a
   * static adjointMap().
   */
  static Jacobian adjointMap(const TangentVector& xi);

  /// @}

 protected:
  /// Return default identity for fixed-size factors and a placeholder for
  /// dynamic ones.
  template <typename T>
  static T defaultIdentity();

  size_t firstDim() const { return traits<G>::GetDimension(this->first); }
  size_t secondDim() const { return traits<H>::GetDimension(this->second); }

  /// Extract a tangent segment for one factor.
  template <typename T, int Dim = traits<T>::dimension>
  static typename traits<T>::TangentVector tangentSegment(
      const TangentVector& v, size_t start, size_t d);

  /// Compute one factor's static algebra adjoint, including vector spaces.
  template <typename T>
  static typename traits<T>::Jacobian componentAdjointMap(
      const typename traits<T>::TangentVector& xi);

  /// Concatenate subgroup tangent vectors into the product tangent.
  static TangentVector makeTangentVector(
      const typename traits<G>::TangentVector& v1,
      const typename traits<H>::TangentVector& v2, size_t d1, size_t d2);

  /// Create a zero Jacobian with the requested runtime size.
  static Jacobian zeroJacobian(size_t d);

  /// Check that another product has matching runtime dimensions.
  void checkMatchingDimensions(const ProductLieGroup& other,
                               const char* operation) const;

 public:
  /// @name Testable interface
  /// @{
  void print(const std::string& s = "") const;

  bool equals(const ProductLieGroup& other, double tol = 1e-9) const {
    return traits<G>::Equals(this->first, other.first, tol) &&
           traits<H>::Equals(this->second, other.second, tol);
  }
  /// @}
};

/**
 * @brief Shared implementation for fixed-size and dynamic-count PowerLieGroup
 *
 * Standard Lie-group operations and their Jacobians are inherited from
 * `LieGroup`; this helper supplies the repeated-component storage traversal,
 * charts, exponential/logarithmic kernels, and adjoint assembly.
 */
template <typename T, int N>
struct PowerLieGroupJacobianStorage {
  /// Container type for per-component Jacobians.
  using type = std::array<T, N>;
};

template <typename T>
struct PowerLieGroupJacobianStorage<T, Eigen::Dynamic> {
  /// Container type for per-component Jacobians.
  using type = std::vector<T>;
};

template <typename G, int N, typename Derived>
class PowerLieGroupBase
    : public LieGroup<Derived,
                      internal::dimensionProduct(N, traits<G>::dimension)> {
 protected:
  static constexpr bool isDynamic = (N == Eigen::Dynamic);
  static constexpr int n = traits<G>::dimension;

 public:
  using LieBase =
      LieGroup<Derived, internal::dimensionProduct(N, traits<G>::dimension)>;
  typedef multiplicative_group_tag group_flavor;
  using LieBase::Dim;
  using LieBase::dimension;
  using TangentVector = typename LieBase::TangentVector;
  using ChartJacobian = typename LieBase::ChartJacobian;
  using Jacobian = typename LieBase::Jacobian;

  /**
   * Component-wise chart at the identity.
   *
   * The dynamic specialization infers its component count from the tangent
   * length before constructing a correctly sized identity. This deliberately
   * uses each component's retract/local chart, which need not equal its
   * Lie-group Expmap/Logmap chart (Pose2 is one such example).
   */
  struct ChartAtOrigin {
    static Derived Retract(const TangentVector& v, ChartJacobian H = {}) {
      if constexpr (isDynamic) {
        if (v.size() % n != 0) {
          throw std::invalid_argument(
              "PowerLieGroup::Retract tangent dimension must be divisible by "
              "base group dimension");
        }
        const size_t count =
            static_cast<size_t>(v.size() / static_cast<Eigen::Index>(n));
        return Derived(count).retract(v, {}, H);
      } else {
        return Derived::Identity().retract(v, {}, H);
      }
    }

    static TangentVector Local(const Derived& value, ChartJacobian H = {}) {
      if constexpr (isDynamic) {
        return Derived(value.size()).localCoordinates(value, {}, H);
      } else {
        return Derived::Identity().localCoordinates(value, {}, H);
      }
    }
  };

  using BaseJacobian = typename traits<G>::Jacobian;
  using JacobianStorage =
      typename PowerLieGroupJacobianStorage<BaseJacobian, N>::type;

 protected:
  /// Downcast to the derived storage type.
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  /// Downcast to the derived storage type.
  Derived& derived() { return static_cast<Derived&>(*this); }

  /// Total tangent dimension for a given component count.
  static size_t totalDimension(size_t count) {
    return count * static_cast<size_t>(n);
  }

  /// Starting offset of one component inside the concatenated tangent.
  static Eigen::Index offset(size_t i) {
    return static_cast<Eigen::Index>(i * static_cast<size_t>(n));
  }

  /// Runtime component count.
  size_t componentCount() const {
    if constexpr (isDynamic) {
      return derived().size();
    } else {
      return N;
    }
  }

  /// Validate tangent size for dynamic-count groups.
  static void checkDynamicTangentSize(const TangentVector& v, size_t count,
                                      const char* operation);

  /// Validate matching component counts for binary operations.
  void checkMatchingCounts(const Derived& other, const char* operation) const;

  /// Extract one component tangent from the concatenated tangent.
  static typename traits<G>::TangentVector tangentSegment(
      const TangentVector& v, size_t i);

  /// Create a result object with the requested component count.
  static Derived makeResult(size_t count);

  /// Create per-component Jacobian storage.
  static JacobianStorage makeJacobianStorage(size_t count);

  /// Write one component tangent into the concatenated tangent.
  static void assignTangentSegment(TangentVector& v, size_t i,
                                   const typename traits<G>::TangentVector& vi);

  /// Write one component block into a block-diagonal Jacobian.
  template <typename MatrixType>
  static void assignJacobianBlock(MatrixType& H, size_t i,
                                  const BaseJacobian& block);

  /// Assemble a block-diagonal Jacobian from per-component blocks.
  static void fillJacobianBlocks(ChartJacobian H,
                                 const JacobianStorage& jacobians,
                                 size_t count);

 public:
  /// Return manifold dimension
  size_t dim() const { return totalDimension(componentCount()); }

  /// Group multiplication
  Derived operator*(const Derived& other) const;

  /// Group inverse
  Derived inverse() const;

  using LieBase::between;
  using LieBase::compose;
  using LieBase::expmap;
  using LieBase::inverse;
  using LieBase::logmap;

  /// Retract to manifold
  Derived retract(const TangentVector& v, ChartJacobian H1 = {},
                  ChartJacobian H2 = {}) const;

  /// Local coordinates on manifold
  TangentVector localCoordinates(const Derived& g, ChartJacobian H1 = {},
                                 ChartJacobian H2 = {}) const;

  /// Exponential map
  static Derived Expmap(const TangentVector& v, ChartJacobian Hv = {});

  /// Logarithmic map
  static TangentVector Logmap(const Derived& p, ChartJacobian Hp = {});

  /// Local coordinates (same as Logmap)
  static TangentVector LocalCoordinates(const Derived& p,
                                        ChartJacobian Hp = {}) {
    return Logmap(p, Hp);
  }

  /// Adjoint map
  Jacobian AdjointMap() const;

  /// Print for debugging
  void print(const std::string& s = "") const;

  /// Equality with tolerance
  bool equals(const Derived& other, double tol = 1e-9) const;

 protected:
  /// Create a zero tangent with the requested runtime size.
  static TangentVector zeroTangent(size_t count);

  /// Create a zero Jacobian with the requested runtime size.
  static Jacobian zeroJacobian(size_t count);
};

/**
 * @brief Template to construct the N-fold power of a Lie group
 * Represents the group G^N = G x G x ... x G (N times)
 * Assumes Lie group structure for fixed-size G and fixed N >= 1
 */
template <typename G, int N>
class PowerLieGroup : public std::array<G, N>,
                      public PowerLieGroupBase<G, N, PowerLieGroup<G, N>> {
  static_assert(N >= 1, "PowerLieGroup requires N >= 1");
  GTSAM_CONCEPT_ASSERT(IsLieGroup<G>);
  GTSAM_CONCEPT_ASSERT(IsTestable<G>);
  static_assert(traits<G>::dimension != Eigen::Dynamic,
                "PowerLieGroup requires a fixed-size base group");

 public:
  /// Base array type
  typedef std::array<G, N> Base;
  typedef PowerLieGroupBase<G, N, PowerLieGroup> Helper;
  using Helper::Dim;
  using Helper::dimension;
  using typename Helper::BaseJacobian;
  using typename Helper::ChartJacobian;
  using typename Helper::Jacobian;
  using typename Helper::TangentVector;

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor yields identity
  PowerLieGroup() { this->fill(traits<G>::Identity()); }

  /// Construct from array of group elements
  PowerLieGroup(const Base& elements) : Base(elements) {}

  /// Construct from initializer list
  PowerLieGroup(const std::initializer_list<G>& elements);

  /// @}
  /// @name Group Operations
  /// @{

  /// Identity element
  static PowerLieGroup Identity() { return PowerLieGroup(); }

  /// @}
};

/**
 * @brief Dynamic-count specialization of PowerLieGroup
 * Represents G^N for runtime-sized N while keeping G fixed-size
 */
template <typename G>
class PowerLieGroup<G, Eigen::Dynamic>
    : public std::vector<G>,
      public PowerLieGroupBase<G, Eigen::Dynamic,
                               PowerLieGroup<G, Eigen::Dynamic>> {
  GTSAM_CONCEPT_ASSERT(IsLieGroup<G>);
  GTSAM_CONCEPT_ASSERT(IsTestable<G>);
  static_assert(traits<G>::dimension != Eigen::Dynamic,
                "PowerLieGroup requires a fixed-size base group");

 public:
  /// Base vector type
  typedef std::vector<G> Base;
  typedef PowerLieGroupBase<G, Eigen::Dynamic, PowerLieGroup> Helper;
  using Helper::Dim;
  using Helper::dimension;
  using typename Helper::BaseJacobian;
  using typename Helper::ChartJacobian;
  using typename Helper::Jacobian;
  using typename Helper::TangentVector;

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor yields a zero-length placeholder identity
  PowerLieGroup() = default;

  /// Construct a runtime-sized identity element
  explicit PowerLieGroup(size_t count) : Base(count, traits<G>::Identity()) {}

  /// Construct from vector of group elements
  PowerLieGroup(const Base& elements) : Base(elements) {}

  /// Construct from initializer list
  PowerLieGroup(const std::initializer_list<G>& elements) : Base(elements) {}

  /// @}
  /// @name Group Operations
  /// @{

  /// Identity element
  static PowerLieGroup Identity() { return PowerLieGroup(); }

  /// @}
};

/// Traits specialization for ProductLieGroup
template <typename G, typename H>
struct traits<ProductLieGroup<G, H>>
    : internal::LieGroup<ProductLieGroup<G, H>> {};

/// Traits specialization for PowerLieGroup
template <typename G, int N>
struct traits<PowerLieGroup<G, N>> : internal::LieGroup<PowerLieGroup<G, N>> {};

}  // namespace gtsam

#include <gtsam/base/ProductLieGroup-inl.h>
