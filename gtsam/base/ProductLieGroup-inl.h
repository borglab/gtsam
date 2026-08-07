/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ProductLieGroup-inl.h
 * @date March, 2026
 * @author Frank Dellaert
 * @author Rohan Bansal
 * @author Alessandro Fornasier
 * @brief Internals for ProductLieGroup.h, not for general consumption
 */

#pragma once

namespace {

/**
 * Assign a sub-block using compile-time dimensions when available.
 *
 * Used by `ProductLieGroup`'s Jacobian and adjoint assembly paths below.
 * `PowerLieGroup` uses its separate `assignJacobianBlock()` helper.
 */
template <typename DstType, typename SrcType>
void assignBlock(const SrcType& source, size_t row, size_t column,
                 DstType* destination) {
  constexpr int rows = SrcType::RowsAtCompileTime;
  constexpr int columns = SrcType::ColsAtCompileTime;
  if constexpr (rows != Eigen::Dynamic && columns != Eigen::Dynamic) {
    destination->template block<rows, columns>(
        static_cast<int>(row), static_cast<int>(column)) = source;
  } else {
    destination->block(row, column, source.rows(), source.cols()) = source;
  }
}

}  // namespace

namespace gtsam {

template <typename G, typename H>
ProductLieGroup<G, H> ProductLieGroup<G, H>::operator*(
    const ProductLieGroup& other) const {
  checkMatchingDimensions(other, "operator*");
  return ProductLieGroup(traits<G>::Compose(this->first, other.first),
                         traits<H>::Compose(this->second, other.second));
}

template <typename G, typename H>
ProductLieGroup<G, H> ProductLieGroup<G, H>::inverse() const {
  return ProductLieGroup(traits<G>::Inverse(this->first),
                         traits<H>::Inverse(this->second));
}

template <typename G, typename H>
ProductLieGroup<G, H> ProductLieGroup<G, H>::retract(const TangentVector& v,
                                                     ChartJacobian H1,
                                                     ChartJacobian H2) const {
  const size_t d1 = firstDim();
  const size_t d2 = secondDim();
  const size_t d = d1 + d2;
  if (static_cast<size_t>(v.size()) != d) {
    throw std::invalid_argument(
        "ProductLieGroup::retract tangent dimension does not match product "
        "dimension");
  }
  Jacobian1 D_g_first = Jacobian1::Zero(d1, d1);
  Jacobian1 D_g_second = Jacobian1::Zero(d1, d1);
  Jacobian2 D_h_first = Jacobian2::Zero(d2, d2);
  Jacobian2 D_h_second = Jacobian2::Zero(d2, d2);
  G g =
      traits<G>::Retract(this->first, tangentSegment<G>(v, 0, d1),
                         H1 ? &D_g_first : nullptr, H2 ? &D_g_second : nullptr);
  H h =
      traits<H>::Retract(this->second, tangentSegment<H>(v, d1, d2),
                         H1 ? &D_h_first : nullptr, H2 ? &D_h_second : nullptr);
  if (H1) {
    *H1 = zeroJacobian(d);
    assignBlock(D_g_first, 0, 0, &*H1);
    assignBlock(D_h_first, d1, d1, &*H1);
  }
  if (H2) {
    *H2 = zeroJacobian(d);
    assignBlock(D_g_second, 0, 0, &*H2);
    assignBlock(D_h_second, d1, d1, &*H2);
  }
  return ProductLieGroup(g, h);
}

template <typename G, typename H>
typename ProductLieGroup<G, H>::TangentVector
ProductLieGroup<G, H>::localCoordinates(const ProductLieGroup& g,
                                        ChartJacobian H1,
                                        ChartJacobian H2) const {
  checkMatchingDimensions(g, "localCoordinates");
  const size_t d1 = firstDim();
  const size_t d2 = secondDim();
  const size_t d = d1 + d2;
  Jacobian1 D_g_first;
  Jacobian1 D_g_second;
  Jacobian2 D_h_first;
  Jacobian2 D_h_second;
  const auto v1 =
      traits<G>::Local(this->first, g.first, H1 ? &D_g_first : nullptr,
                       H2 ? &D_g_second : nullptr);
  const auto v2 =
      traits<H>::Local(this->second, g.second, H1 ? &D_h_first : nullptr,
                       H2 ? &D_h_second : nullptr);
  if (H1) {
    *H1 = zeroJacobian(d);
    assignBlock(D_g_first, 0, 0, &*H1);
    assignBlock(D_h_first, d1, d1, &*H1);
  }
  if (H2) {
    *H2 = zeroJacobian(d);
    assignBlock(D_g_second, 0, 0, &*H2);
    assignBlock(D_h_second, d1, d1, &*H2);
  }
  return makeTangentVector(v1, v2, d1, d2);
}

template <typename G, typename H>
ProductLieGroup<G, H> ProductLieGroup<G, H>::Expmap(const TangentVector& v,
                                                    ChartJacobian Hv) {
  size_t d1 = 0;
  size_t d2 = 0;
  if constexpr (firstDynamic && secondDynamic) {
    if (v.size() == 0) {
      if (Hv) *Hv = Matrix::Zero(0, 0);
      return ProductLieGroup();
    }
    throw std::invalid_argument(
        "ProductLieGroup::Expmap requires split tangent vectors when both "
        "factors are dynamic");
  } else if constexpr (firstDynamic) {
    if (v.size() < m) {
      throw std::invalid_argument(
          "ProductLieGroup::Expmap tangent dimension is too small for the "
          "fixed second factor");
    }
    d1 = static_cast<size_t>(v.size() - m);
    d2 = static_cast<size_t>(m);
  } else if constexpr (secondDynamic) {
    if (v.size() < n) {
      throw std::invalid_argument(
          "ProductLieGroup::Expmap tangent dimension is too small for the "
          "fixed first factor");
    }
    d1 = static_cast<size_t>(n);
    d2 = static_cast<size_t>(v.size() - n);
  } else {
    d1 = static_cast<size_t>(n);
    d2 = static_cast<size_t>(m);
  }
  if (static_cast<size_t>(v.size()) != d1 + d2) {
    throw std::invalid_argument(
        "ProductLieGroup::Expmap tangent dimension does not match product "
        "dimension");
  }

  Matrix D_g_first;
  Matrix D_h_second;
  const auto v1 = tangentSegment<G>(v, 0, d1);
  const auto v2 = tangentSegment<H>(v, d1, d2);
  ProductLieGroup result =
      Expmap(v1, v2,
             Hv ? OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>(D_g_first)
                : OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>(),
             Hv ? OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>(D_h_second)
                : OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>());
  if (Hv) {
    const size_t d = d1 + d2;
    *Hv = zeroJacobian(d);
    Hv->block(0, 0, d, d1) = D_g_first;
    Hv->block(0, d1, d, d2) = D_h_second;
  }
  return result;
}

template <typename G, typename H>
ProductLieGroup<G, H> ProductLieGroup<G, H>::Expmap(
    const Eigen::Ref<const typename traits<G>::TangentVector>& v1,
    const Eigen::Ref<const typename traits<H>::TangentVector>& v2,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2) {
  const size_t d1 = static_cast<size_t>(v1.size());
  const size_t d2 = static_cast<size_t>(v2.size());
  const size_t d = d1 + d2;
  Jacobian1 D_g_first;
  Jacobian2 D_h_second;
  G g = traits<G>::Expmap(v1, H1 ? &D_g_first : nullptr);
  H h = traits<H>::Expmap(v2, H2 ? &D_h_second : nullptr);
  if (H1) {
    *H1 = Matrix::Zero(d, d1);
    assignBlock(D_g_first, 0, 0, &*H1);
  }
  if (H2) {
    *H2 = Matrix::Zero(d, d2);
    assignBlock(D_h_second, d1, 0, &*H2);
  }
  return ProductLieGroup(g, h);
}

template <typename G, typename H>
typename ProductLieGroup<G, H>::TangentVector ProductLieGroup<G, H>::Logmap(
    const ProductLieGroup& p, ChartJacobian Hp) {
  const size_t d1 = p.firstDim();
  const size_t d2 = p.secondDim();
  const size_t d = d1 + d2;
  if (!Hp) {
    const auto v1 = traits<G>::Logmap(p.first);
    const auto v2 = traits<H>::Logmap(p.second);
    return makeTangentVector(v1, v2, d1, d2);
  }

  Jacobian1 D_g_first;
  Jacobian2 D_h_second;
  typename traits<G>::TangentVector v1 = traits<G>::Logmap(p.first, &D_g_first);
  typename traits<H>::TangentVector v2 =
      traits<H>::Logmap(p.second, &D_h_second);
  TangentVector v = makeTangentVector(v1, v2, d1, d2);
  *Hp = zeroJacobian(d);
  assignBlock(D_g_first, 0, 0, &*Hp);
  assignBlock(D_h_second, d1, d1, &*Hp);
  return v;
}

template <typename G, typename H>
typename ProductLieGroup<G, H>::Jacobian ProductLieGroup<G, H>::AdjointMap()
    const {
  // The two algebras are independent:
  //   Ad_(g,h) = diag(Ad_G(g), Ad_H(h)).
  const auto adjG = traits<G>::AdjointMap(this->first);
  const auto adjH = traits<H>::AdjointMap(this->second);
  const size_t d1 = static_cast<size_t>(adjG.rows());
  const size_t d2 = static_cast<size_t>(adjH.rows());
  Jacobian adj = zeroJacobian(d1 + d2);
  assignBlock(adjG, 0, 0, &adj);
  assignBlock(adjH, d1, d1, &adj);
  return adj;
}

template <typename G, typename H>
typename ProductLieGroup<G, H>::Jacobian ProductLieGroup<G, H>::adjointMap(
    const TangentVector& xi) {
  const size_t d = static_cast<size_t>(xi.size());
  size_t d1 = 0, d2 = 0;
  if constexpr (firstDynamic && secondDynamic) {
    if constexpr (internal::ProductLieGroupIsVector<G>::value &&
                  internal::ProductLieGroupIsVector<H>::value) {
      // Both factors are abelian, so the result is zero regardless of where
      // the concatenated tangent is split.
      return zeroJacobian(d);
    }
    throw std::invalid_argument(
        "ProductLieGroup::adjointMap cannot infer the tangent split when "
        "both factors are dynamic");
  } else if constexpr (firstDynamic) {
    if (xi.size() < m) {
      throw std::invalid_argument(
          "ProductLieGroup::adjointMap tangent dimension is too small for "
          "the fixed second factor");
    }
    d1 = static_cast<size_t>(xi.size() - m);
    d2 = static_cast<size_t>(m);
  } else if constexpr (secondDynamic) {
    if (xi.size() < n) {
      throw std::invalid_argument(
          "ProductLieGroup::adjointMap tangent dimension is too small for "
          "the fixed first factor");
    }
    d1 = static_cast<size_t>(n);
    d2 = static_cast<size_t>(xi.size() - n);
  } else {
    d1 = static_cast<size_t>(n);
    d2 = static_cast<size_t>(m);
  }
  if (d != d1 + d2) {
    throw std::invalid_argument(
        "ProductLieGroup::adjointMap tangent dimension does not match "
        "product dimension");
  }

  const auto a = tangentSegment<G>(xi, 0, d1);
  const auto b = tangentSegment<H>(xi, d1, d2);

  Jacobian ad = zeroJacobian(d);
  assignBlock(componentAdjointMap<G>(a), 0, 0, &ad);
  assignBlock(componentAdjointMap<H>(b), d1, d1, &ad);
  return ad;
}

template <typename G, typename H>
template <typename T>
T ProductLieGroup<G, H>::defaultIdentity() {
  if constexpr (traits<T>::dimension == Eigen::Dynamic) {
    return T();
  } else {
    return traits<T>::Identity();
  }
}

template <typename G, typename H>
template <typename T, int Dim>
typename traits<T>::TangentVector ProductLieGroup<G, H>::tangentSegment(
    const TangentVector& v, size_t start, size_t d) {
  const int startIndex = static_cast<int>(start);
  const int runtimeIndex = static_cast<int>(d);
  if constexpr (Dim == Eigen::Dynamic) {
    return v.segment(startIndex, runtimeIndex);
  } else {
    static_cast<void>(d);
    return v.template segment<Dim>(startIndex);
  }
}

template <typename G, typename H>
template <typename T>
typename traits<T>::Jacobian ProductLieGroup<G, H>::componentAdjointMap(
    const typename traits<T>::TangentVector& xi) {
  using ComponentJacobian = typename traits<T>::Jacobian;
  if constexpr (internal::ProductLieGroupIsVector<T>::value) {
    if constexpr (traits<T>::dimension == Eigen::Dynamic) {
      return ComponentJacobian::Zero(xi.size(), xi.size());
    } else {
      return ComponentJacobian::Zero();
    }
  } else {
    return T::adjointMap(xi);
  }
}

template <typename G, typename H>
typename ProductLieGroup<G, H>::TangentVector
ProductLieGroup<G, H>::makeTangentVector(
    const typename traits<G>::TangentVector& v1,
    const typename traits<H>::TangentVector& v2, size_t d1, size_t d2) {
  const int firstIndex = static_cast<int>(d1);
  const int secondIndex = static_cast<int>(d2);
  if constexpr (dimension == Eigen::Dynamic) {
    TangentVector v(d1 + d2);
    v.segment(0, firstIndex) = v1;
    v.segment(firstIndex, secondIndex) = v2;
    return v;
  } else {
    static_cast<void>(d1);
    static_cast<void>(d2);
    TangentVector v;
    v << v1, v2;
    return v;
  }
}

template <typename G, typename H>
typename ProductLieGroup<G, H>::Jacobian ProductLieGroup<G, H>::zeroJacobian(
    size_t d) {
  if constexpr (dimension == Eigen::Dynamic) {
    return Jacobian::Zero(d, d);
  } else {
    static_cast<void>(d);
    return Jacobian::Zero();
  }
}

template <typename G, typename H>
void ProductLieGroup<G, H>::checkMatchingDimensions(
    const ProductLieGroup& other, const char* operation) const {
  if (firstDim() != other.firstDim() || secondDim() != other.secondDim()) {
    throw std::invalid_argument(std::string("ProductLieGroup::") + operation +
                                " requires matching component dimensions");
  }
}

template <typename G, typename H>
void ProductLieGroup<G, H>::print(const std::string& s) const {
  std::cout << s << "ProductLieGroup" << std::endl;
  traits<G>::Print(this->first, "  first");
  traits<H>::Print(this->second, "  second");
}

template <typename G, int N, typename Derived>
void PowerLieGroupBase<G, N, Derived>::checkDynamicTangentSize(
    const TangentVector& v, size_t count, const char* operation) {
  if constexpr (isDynamic) {
    if (static_cast<size_t>(v.size()) != totalDimension(count)) {
      throw std::invalid_argument(
          std::string("PowerLieGroup::") + operation +
          " tangent dimension does not match group dimension");
    }
  } else {
    static_cast<void>(v);
    static_cast<void>(count);
    static_cast<void>(operation);
  }
}

template <typename G, int N, typename Derived>
void PowerLieGroupBase<G, N, Derived>::checkMatchingCounts(
    const Derived& other, const char* operation) const {
  if constexpr (isDynamic) {
    if (derived().size() != other.size()) {
      throw std::invalid_argument(std::string("PowerLieGroup::") + operation +
                                  " requires matching component counts");
    }
  } else {
    static_cast<void>(other);
    static_cast<void>(operation);
  }
}

template <typename G, int N, typename Derived>
typename traits<G>::TangentVector
PowerLieGroupBase<G, N, Derived>::tangentSegment(const TangentVector& v,
                                                 size_t i) {
  if constexpr (isDynamic) {
    return v.segment(offset(i), n);
  } else {
    return v.template segment<n>(i * n);
  }
}

template <typename G, int N, typename Derived>
Derived PowerLieGroupBase<G, N, Derived>::makeResult(size_t count) {
  if constexpr (isDynamic) {
    return Derived(count);
  } else {
    static_cast<void>(count);
    return Derived();
  }
}

template <typename G, int N, typename Derived>
typename PowerLieGroupBase<G, N, Derived>::JacobianStorage
PowerLieGroupBase<G, N, Derived>::makeJacobianStorage(size_t count) {
  if constexpr (isDynamic) {
    return JacobianStorage(count);
  } else {
    static_cast<void>(count);
    return JacobianStorage();
  }
}

template <typename G, int N, typename Derived>
void PowerLieGroupBase<G, N, Derived>::assignTangentSegment(
    TangentVector& v, size_t i, const typename traits<G>::TangentVector& vi) {
  if constexpr (isDynamic) {
    v.segment(offset(i), n) = vi;
  } else {
    v.template segment<n>(i * n) = vi;
  }
}

template <typename G, int N, typename Derived>
template <typename MatrixType>
void PowerLieGroupBase<G, N, Derived>::assignJacobianBlock(
    MatrixType& H, size_t i, const BaseJacobian& block) {
  if constexpr (isDynamic) {
    H.block(offset(i), offset(i), n, n) = block;
  } else {
    H.template block<n, n>(i * n, i * n) = block;
  }
}

template <typename G, int N, typename Derived>
void PowerLieGroupBase<G, N, Derived>::fillJacobianBlocks(
    ChartJacobian H, const JacobianStorage& jacobians, size_t count) {
  if (!H) return;
  *H = zeroJacobian(count);
  for (size_t i = 0; i < count; ++i) {
    assignJacobianBlock(*H, i, jacobians[i]);
  }
}

template <typename G, int N, typename Derived>
Derived PowerLieGroupBase<G, N, Derived>::operator*(
    const Derived& other) const {
  checkMatchingCounts(other, "operator*");
  Derived result = makeResult(componentCount());
  for (size_t i = 0; i < componentCount(); ++i) {
    result[i] = traits<G>::Compose(derived()[i], other[i]);
  }
  return result;
}

template <typename G, int N, typename Derived>
Derived PowerLieGroupBase<G, N, Derived>::inverse() const {
  Derived result = makeResult(componentCount());
  for (size_t i = 0; i < componentCount(); ++i) {
    result[i] = traits<G>::Inverse(derived()[i]);
  }
  return result;
}

template <typename G, int N, typename Derived>
Derived PowerLieGroupBase<G, N, Derived>::retract(const TangentVector& v,
                                                  ChartJacobian H1,
                                                  ChartJacobian H2) const {
  const size_t count = componentCount();
  checkDynamicTangentSize(v, count, "retract");
  JacobianStorage firstJacobians = makeJacobianStorage(count);
  JacobianStorage secondJacobians = makeJacobianStorage(count);
  Derived result = makeResult(count);
  for (size_t i = 0; i < count; ++i) {
    result[i] = traits<G>::Retract(derived()[i], tangentSegment(v, i),
                                   H1 ? &firstJacobians[i] : nullptr,
                                   H2 ? &secondJacobians[i] : nullptr);
  }
  fillJacobianBlocks(H1, firstJacobians, count);
  fillJacobianBlocks(H2, secondJacobians, count);
  return result;
}

template <typename G, int N, typename Derived>
typename PowerLieGroupBase<G, N, Derived>::TangentVector
PowerLieGroupBase<G, N, Derived>::localCoordinates(const Derived& g,
                                                   ChartJacobian H1,
                                                   ChartJacobian H2) const {
  checkMatchingCounts(g, "localCoordinates");
  const size_t count = componentCount();
  JacobianStorage firstJacobians = makeJacobianStorage(count);
  JacobianStorage secondJacobians = makeJacobianStorage(count);
  TangentVector v = zeroTangent(componentCount());
  for (size_t i = 0; i < count; ++i) {
    assignTangentSegment(
        v, i,
        traits<G>::Local(derived()[i], g[i], H1 ? &firstJacobians[i] : nullptr,
                         H2 ? &secondJacobians[i] : nullptr));
  }
  fillJacobianBlocks(H1, firstJacobians, count);
  fillJacobianBlocks(H2, secondJacobians, count);
  return v;
}

template <typename G, int N, typename Derived>
Derived PowerLieGroupBase<G, N, Derived>::Expmap(const TangentVector& v,
                                                 ChartJacobian Hv) {
  size_t count = 0;
  if constexpr (isDynamic) {
    if (v.size() % n != 0) {
      throw std::invalid_argument(
          "PowerLieGroup::Expmap tangent dimension must be divisible by base  "
          "group dimension");
    }
    count = static_cast<size_t>(v.size() / static_cast<Eigen::Index>(n));
  } else {
    count = N;
  }
  JacobianStorage jacobians = makeJacobianStorage(count);
  Derived result = makeResult(count);
  for (size_t i = 0; i < count; ++i) {
    result[i] =
        traits<G>::Expmap(tangentSegment(v, i), Hv ? &jacobians[i] : nullptr);
  }
  fillJacobianBlocks(Hv, jacobians, count);
  return result;
}

template <typename G, int N, typename Derived>
typename PowerLieGroupBase<G, N, Derived>::TangentVector
PowerLieGroupBase<G, N, Derived>::Logmap(const Derived& p, ChartJacobian Hp) {
  const size_t count = isDynamic ? p.size() : N;
  TangentVector v = zeroTangent(count);
  JacobianStorage jacobians = makeJacobianStorage(count);
  for (size_t i = 0; i < count; ++i) {
    assignTangentSegment(v, i,
                         traits<G>::Logmap(p[i], Hp ? &jacobians[i] : nullptr));
  }
  fillJacobianBlocks(Hp, jacobians, count);
  return v;
}

template <typename G, int N, typename Derived>
typename PowerLieGroupBase<G, N, Derived>::Jacobian
PowerLieGroupBase<G, N, Derived>::AdjointMap() const {
  Jacobian adj = zeroJacobian(componentCount());
  for (size_t i = 0; i < componentCount(); ++i) {
    assignJacobianBlock(adj, i, traits<G>::AdjointMap(derived()[i]));
  }
  return adj;
}

template <typename G, int N, typename Derived>
void PowerLieGroupBase<G, N, Derived>::print(const std::string& s) const {
  std::cout << s << "PowerLieGroup" << std::endl;
  for (size_t i = 0; i < componentCount(); ++i) {
    traits<G>::Print(derived()[i], "  component[" + std::to_string(i) + "]");
  }
}

template <typename G, int N, typename Derived>
bool PowerLieGroupBase<G, N, Derived>::equals(const Derived& other,
                                              double tol) const {
  if constexpr (isDynamic) {
    if (derived().size() != other.size()) {
      return false;
    }
  }
  for (size_t i = 0; i < componentCount(); ++i) {
    if (!traits<G>::Equals(derived()[i], other[i], tol)) {
      return false;
    }
  }
  return true;
}

template <typename G, int N, typename Derived>
typename PowerLieGroupBase<G, N, Derived>::TangentVector
PowerLieGroupBase<G, N, Derived>::zeroTangent(size_t count) {
  if constexpr (isDynamic) {
    return TangentVector::Zero(totalDimension(count));
  } else {
    static_cast<void>(count);
    return TangentVector::Zero();
  }
}

template <typename G, int N, typename Derived>
typename PowerLieGroupBase<G, N, Derived>::Jacobian
PowerLieGroupBase<G, N, Derived>::zeroJacobian(size_t count) {
  if constexpr (isDynamic) {
    return Jacobian::Zero(totalDimension(count), totalDimension(count));
  } else {
    static_cast<void>(count);
    return Jacobian::Zero();
  }
}

template <typename G, int N>
PowerLieGroup<G, N>::PowerLieGroup(const std::initializer_list<G>& elements) {
  if (elements.size() != N) {
    throw std::invalid_argument(
        "PowerLieGroup: initializer list size must equal N");
  }
  std::copy(elements.begin(), elements.end(), this->begin());
}

}  // namespace gtsam
