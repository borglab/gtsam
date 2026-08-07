/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file TangentLieGroup-inl.h
 * @date August, 2026
 * @author Alessandro Fornasier
 * @author Frank Dellaert
 * @brief Internal implementation of TangentLieGroup.
 */

#pragma once

#include <unsupported/Eigen/MatrixFunctions>

namespace gtsam {

template <typename G>
TangentLieGroup<G> TangentLieGroup<G>::operator*(
    const TangentLieGroup& other) const {
  const BaseJacobian Ad = traits<G>::AdjointMap(this->first);
  return {traits<G>::Compose(this->first, other.first),
          this->second + Ad * other.second};
}

template <typename G>
TangentLieGroup<G> TangentLieGroup<G>::inverse() const {
  const G gInverse = traits<G>::Inverse(this->first);
  return {gInverse, -traits<G>::AdjointMap(gInverse) * this->second};
}

template <typename G>
TangentLieGroup<G> TangentLieGroup<G>::retract(const TangentVector& xi,
                                               ChartJacobian H1,
                                               ChartJacobian H2) const {
  Jacobian Dexp;
  const TangentLieGroup delta = Expmap(xi, H2 ? &Dexp : nullptr);
  const TangentLieGroup result = compose(delta);
  if (H1) *H1 = delta.inverse().AdjointMap();
  if (H2) *H2 = Dexp;
  return result;
}

template <typename G>
typename TangentLieGroup<G>::TangentVector TangentLieGroup<G>::localCoordinates(
    const TangentLieGroup& other, ChartJacobian H1, ChartJacobian H2) const {
  const TangentLieGroup relative = between(other);
  Jacobian Dlog;
  const TangentVector xi =
      Logmap(relative, H1 || H2 ? ChartJacobian(&Dlog) : ChartJacobian());
  if (H1) *H1 = -Dlog * relative.inverse().AdjointMap();
  if (H2) *H2 = Dlog;
  return xi;
}

template <typename G>
TangentLieGroup<G> TangentLieGroup<G>::Expmap(const TangentVector& xi,
                                              ChartJacobian H) {
  const auto [u, v] = split(xi);
  Matrix D1, D2;
  const TangentLieGroup result =
      Expmap(u, v, H ? SplitJacobian(D1) : SplitJacobian(),
             H ? SplitJacobian(D2) : SplitJacobian());
  if (H) {
    H->leftCols(n) = D1;
    H->rightCols(n) = D2;
  }
  return result;
}

template <typename G>
TangentLieGroup<G> TangentLieGroup<G>::Expmap(
    const Eigen::Ref<const BaseTangent>& u,
    const Eigen::Ref<const BaseTangent>& v, SplitJacobian H1,
    SplitJacobian H2) {
  if constexpr (internal::TangentLieGroupJacobian<G>::expmapAvailable) {
    Jacobian derivative;
    const auto [g, transported] = internal::TangentLieGroupJacobian<G>::expmap(
        u, v, H1 || H2 ? &derivative : nullptr);
    if (H1) *H1 = derivative.leftCols(n);
    if (H2) *H2 = derivative.rightCols(n);
    return {g, transported};
  }

  BaseJacobian Dg;
  const G g = traits<G>::Expmap(u, &Dg);
  const BaseTangent transported = traits<G>::AdjointMap(g) * Dg * v;
  if (H1) {
    const Jacobian derivative = rightJacobian(join(u, v));
    *H1 = derivative.leftCols(n);
    if (H2) *H2 = derivative.rightCols(n);
  } else if (H2) {
    *H2 = Matrix::Zero(dimension, n);
    H2->bottomRows(n) = Dg;
  }
  return {g, transported};
}

template <typename G>
typename TangentLieGroup<G>::TangentVector TangentLieGroup<G>::Logmap(
    const TangentLieGroup& p, ChartJacobian H) {
  BaseJacobian Dg;
  const BaseTangent u = traits<G>::Logmap(p.first, &Dg);
  const BaseJacobian AdInverse =
      traits<G>::AdjointMap(traits<G>::Inverse(p.first));
  const BaseTangent v = Dg * AdInverse * p.second;
  const TangentVector xi = join(u, v);
  if (H) {
    const Jacobian derivative = rightJacobian(xi);
    const BaseJacobian Q = derivative.template bottomLeftCorner<n, n>();
    H->setZero();
    H->template topLeftCorner<n, n>() = Dg;
    H->template bottomRightCorner<n, n>() = Dg;
    H->template bottomLeftCorner<n, n>() = -Dg * Q * Dg;
  }
  return xi;
}

template <typename G>
typename TangentLieGroup<G>::Jacobian TangentLieGroup<G>::rightJacobian(
    const TangentVector& xi) {
  const auto [u, v] = split(xi);
  if constexpr (internal::TangentLieGroupJacobian<G>::available) {
    return internal::TangentLieGroupJacobian<G>::rightJacobian(u, v);
  }

  const BaseJacobian A = -G::adjointMap(u);
  const BaseJacobian B = -G::adjointMap(v);
  using Augmented = Eigen::Matrix<double, 3 * n, 3 * n>;
  Augmented M = Augmented::Zero();
  M.template block<n, n>(0, n).setIdentity();
  M.template block<n, n>(n, n) = A;
  M.template block<n, n>(n, 2 * n) = B;
  M.template block<n, n>(2 * n, 2 * n) = A;
  const Augmented expM = M.exp();
  const BaseJacobian J = expM.template block<n, n>(0, n);
  const BaseJacobian Q = expM.template block<n, n>(0, 2 * n);
  Jacobian result = Jacobian::Zero();
  result.template topLeftCorner<n, n>() = J;
  result.template bottomRightCorner<n, n>() = J;
  result.template bottomLeftCorner<n, n>() = Q;
  return result;
}

template <typename G>
typename TangentLieGroup<G>::Jacobian TangentLieGroup<G>::AdjointMap() const {
  const BaseJacobian Ad = traits<G>::AdjointMap(this->first);
  const BaseJacobian adV = G::adjointMap(this->second);
  Jacobian result = Jacobian::Zero();
  result.template topLeftCorner<n, n>() = Ad;
  result.template bottomRightCorner<n, n>() = Ad;
  result.template bottomLeftCorner<n, n>() = adV * Ad;
  return result;
}

template <typename G>
typename TangentLieGroup<G>::Jacobian TangentLieGroup<G>::adjointMap(
    const TangentVector& xi) {
  const auto [u, v] = split(xi);
  const BaseJacobian adU = G::adjointMap(u);
  Jacobian result = Jacobian::Zero();
  result.template topLeftCorner<n, n>() = adU;
  result.template bottomRightCorner<n, n>() = adU;
  result.template bottomLeftCorner<n, n>() = G::adjointMap(v);
  return result;
}

template <typename G>
std::pair<typename TangentLieGroup<G>::BaseTangent,
          typename TangentLieGroup<G>::BaseTangent>
TangentLieGroup<G>::split(const TangentVector& xi) {
  return {xi.template head<n>(), xi.template tail<n>()};
}

template <typename G>
typename TangentLieGroup<G>::TangentVector TangentLieGroup<G>::join(
    const BaseTangent& u, const BaseTangent& v) {
  TangentVector xi;
  xi << u, v;
  return xi;
}

template <typename G>
void TangentLieGroup<G>::print(const std::string& s) const {
  std::cout << s << "TangentLieGroup" << std::endl;
  traits<G>::Print(this->first, "  group: ");
  traits<BaseTangent>::Print(this->second, "  tangent: ");
}

}  // namespace gtsam
