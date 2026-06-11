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
#include <utility>  // pair
#include <vector>

namespace gtsam {

/**
 * @brief Template to construct the product Lie group of two other Lie groups
 * Assumes Lie group structure for G and H
 */
template <typename G, typename H>
class ProductLieGroup : public std::pair<G, H> {
  GTSAM_CONCEPT_ASSERT(IsLieGroup<G>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<H>);
  GTSAM_CONCEPT_ASSERT(IsTestable<G>);
  GTSAM_CONCEPT_ASSERT(IsTestable<H>);

 public:
  /// Base pair type
  typedef std::pair<G, H> Base;

 protected:
  /// Dimensions of the two subgroups
  inline constexpr static int dimension1 = traits<G>::dimension;
  inline constexpr static int dimension2 = traits<H>::dimension;
  inline constexpr static bool firstDynamic = dimension1 == Eigen::Dynamic;
  inline constexpr static bool secondDynamic = dimension2 == Eigen::Dynamic;

 public:
  /// Manifold dimension
  inline constexpr static int dimension =
      firstDynamic || secondDynamic ? Eigen::Dynamic : dimension1 + dimension2;

  /// Tangent vector type
  using TangentVector = std::conditional_t<dimension == Eigen::Dynamic, Vector,
                                           Eigen::Matrix<double, dimension, 1>>;

  /// Chart Jacobian type
  using ChartJacobian =
      std::conditional_t<dimension == Eigen::Dynamic,
                         OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>,
                         OptionalJacobian<dimension, dimension>>;

  /// Jacobian types for internal use
  using Jacobian =
      std::conditional_t<dimension == Eigen::Dynamic, Matrix,
                         Eigen::Matrix<double, dimension, dimension>>;
  using Jacobian1 = typename traits<G>::Jacobian;
  using Jacobian2 = typename traits<H>::Jacobian;

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

  /// Compose with another element (same as operator*)
  ProductLieGroup compose(const ProductLieGroup& g) const {
    return (*this) * g;
  }

  /// Calculate relative transformation
  ProductLieGroup between(const ProductLieGroup& g) const {
    return this->inverse() * g;
  }

  /// @}
  /// @name Manifold Operations
  /// @{

  /// Manifold dimension
  inline constexpr static int manifoldDimension = dimension;

  /// Return manifold dimension
  static constexpr int Dim() { return manifoldDimension; }

  /// Return manifold dimension
  size_t dim() const { return combinedDimension(firstDim(), secondDim()); }

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

  /// Compose with Jacobians
  ProductLieGroup compose(const ProductLieGroup& other, ChartJacobian H1,
                          ChartJacobian H2 = {}) const;

  /// Between with Jacobians
  ProductLieGroup between(const ProductLieGroup& other, ChartJacobian H1,
                          ChartJacobian H2 = {}) const;

  /// Inverse with Jacobian
  ProductLieGroup inverse(ChartJacobian D) const;

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

  /// Right multiplication by exponential map
  ProductLieGroup expmap(const TangentVector& v) const;

  /// Logarithmic map for relative transformation
  TangentVector logmap(const ProductLieGroup& g) const {
    return ProductLieGroup::Logmap(between(g));
  }

  /// Adjoint map
  Jacobian AdjointMap() const;

  /// @}

 protected:
  /// Return default identity for fixed-size factors and a placeholder for
  /// dynamic ones.
  template <typename T>
  static T defaultIdentity();

  size_t firstDim() const { return traits<G>::GetDimension(this->first); }
  size_t secondDim() const { return traits<H>::GetDimension(this->second); }

  static size_t combinedDimension(size_t d1, size_t d2) { return d1 + d2; }

  /// Extract a tangent segment for one factor.
  template <typename T, int Dim = traits<T>::dimension>
  static typename traits<T>::TangentVector tangentSegment(
      const TangentVector& v, size_t start, size_t runtimeDimension);

  /// Concatenate subgroup tangent vectors into the product tangent.
  static TangentVector makeTangentVector(
      const typename traits<G>::TangentVector& v1,
      const typename traits<H>::TangentVector& v2, size_t firstDimension,
      size_t secondDimension);

  /// Create a zero Jacobian with the requested runtime size.
  static Jacobian zeroJacobian(size_t productDimension);

  /// Create an identity Jacobian with the requested runtime size.
  static Jacobian identityJacobian(size_t productDimension);

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
class PowerLieGroupBase {
 protected:
  static constexpr bool isDynamic = (N == Eigen::Dynamic);
  static constexpr int baseDimension = traits<G>::dimension;

 public:
  typedef multiplicative_group_tag group_flavor;

  static constexpr int dimension =
      isDynamic ? Eigen::Dynamic : N * baseDimension;

  typedef std::conditional_t<isDynamic, Vector,
                             Eigen::Matrix<double, dimension, 1>>
      TangentVector;

  typedef std::conditional_t<isDynamic,
                             OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>,
                             OptionalJacobian<dimension, dimension>>
      ChartJacobian;

  typedef std::conditional_t<isDynamic, Matrix,
                             Eigen::Matrix<double, dimension, dimension>>
      Jacobian;

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
    return count * static_cast<size_t>(baseDimension);
  }

  /// Starting offset of one component inside the concatenated tangent.
  static Eigen::Index offset(size_t i) {
    return static_cast<Eigen::Index>(i * static_cast<size_t>(baseDimension));
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
  static constexpr int Dim() { return dimension; }

  /// Return manifold dimension
  size_t dim() const { return totalDimension(componentCount()); }

  /// Group multiplication
  Derived operator*(const Derived& other) const;

  /// Group inverse
  Derived inverse() const;

  /// Compose with another element (same as operator*)
  Derived compose(const Derived& g) const { return (*this) * g; }

  /// Calculate relative transformation
  Derived between(const Derived& g) const { return this->inverse() * g; }

  /// Retract to manifold
  Derived retract(const TangentVector& v, ChartJacobian H1 = {},
                  ChartJacobian H2 = {}) const;

  /// Local coordinates on manifold
  TangentVector localCoordinates(const Derived& g, ChartJacobian H1 = {},
                                 ChartJacobian H2 = {}) const;

  /// Compose with Jacobians
  Derived compose(const Derived& other, ChartJacobian H1,
                  ChartJacobian H2 = {}) const;

  /// Between with Jacobians
  Derived between(const Derived& other, ChartJacobian H1,
                  ChartJacobian H2 = {}) const;

  /// Inverse with Jacobian
  Derived inverse(ChartJacobian D) const;

  /// Exponential map
  static Derived Expmap(const TangentVector& v, ChartJacobian Hv = {});

  /// Logarithmic map
  static TangentVector Logmap(const Derived& p, ChartJacobian Hp = {});

  /// Local coordinates (same as Logmap)
  static TangentVector LocalCoordinates(const Derived& p,
                                        ChartJacobian Hp = {}) {
    return Logmap(p, Hp);
  }

  /// Right multiplication by exponential map
  Derived expmap(const TangentVector& v) const { return compose(Expmap(v)); }

  /// Logarithmic map for relative transformation
  TangentVector logmap(const Derived& g) const { return Logmap(between(g)); }

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

  /// Create an identity Jacobian with the requested runtime size.
  static Jacobian identityJacobian(size_t count);
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
  using typename Helper::BaseJacobian;
  using typename Helper::ChartJacobian;
  using typename Helper::Jacobian;
  using typename Helper::TangentVector;
  static constexpr int dimension = Helper::dimension;

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
  /// @name Manifold Operations
  /// @{

  /// Return manifold dimension
  static constexpr int Dim() { return dimension; }

  /// @}
  /// @name Lie Group Operations
  /// @{

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
  using typename Helper::BaseJacobian;
  using typename Helper::ChartJacobian;
  using typename Helper::Jacobian;
  using typename Helper::TangentVector;
  static constexpr int dimension = Helper::dimension;

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
  /// @name Manifold Operations
  /// @{

  /// Return manifold dimension
  static constexpr int Dim() { return dimension; }

  /// @}
  /// @name Lie Group Operations
  /// @{

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
