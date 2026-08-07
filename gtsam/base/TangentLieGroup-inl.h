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
TangentLieGroup<G> TangentLieGroup<G>::compose(const TangentLieGroup& other,
                                               ChartJacobian H1,
                                               ChartJacobian H2) const {
  const TangentLieGroup result = (*this) * other;
  if (H1) *H1 = other.inverse().AdjointMap();
  if (H2) *H2 = Jacobian::Identity();
  return result;
}

template <typename G>
TangentLieGroup<G> TangentLieGroup<G>::between(const TangentLieGroup& other,
                                               ChartJacobian H1,
                                               ChartJacobian H2) const {
  const TangentLieGroup result = inverse() * other;
  if (H1) *H1 = -result.inverse().AdjointMap();
  if (H2) *H2 = Jacobian::Identity();
  return result;
}

template <typename G>
TangentLieGroup<G> TangentLieGroup<G>::inverse(ChartJacobian H) const {
  if (H) *H = -AdjointMap();
  return inverse();
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
    H->leftCols(baseDimension) = D1;
    H->rightCols(baseDimension) = D2;
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
    if (H1) *H1 = derivative.leftCols(baseDimension);
    if (H2) *H2 = derivative.rightCols(baseDimension);
    return {g, transported};
  }

  BaseJacobian Dg;
  const G g = traits<G>::Expmap(u, &Dg);
  const BaseTangent transported = traits<G>::AdjointMap(g) * Dg * v;
  if (H1) {
    const Jacobian derivative = rightJacobian(join(u, v));
    *H1 = derivative.leftCols(baseDimension);
    if (H2) *H2 = derivative.rightCols(baseDimension);
  } else if (H2) {
    *H2 = Matrix::Zero(dimension, baseDimension);
    H2->bottomRows(baseDimension) = Dg;
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
    const BaseJacobian Q =
        derivative.template bottomLeftCorner<baseDimension, baseDimension>();
    H->setZero();
    H->template topLeftCorner<baseDimension, baseDimension>() = Dg;
    H->template bottomRightCorner<baseDimension, baseDimension>() = Dg;
    H->template bottomLeftCorner<baseDimension, baseDimension>() = -Dg * Q * Dg;
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
  using Augmented = Eigen::Matrix<double, 3 * baseDimension, 3 * baseDimension>;
  Augmented M = Augmented::Zero();
  M.template block<baseDimension, baseDimension>(0, baseDimension)
      .setIdentity();
  M.template block<baseDimension, baseDimension>(baseDimension, baseDimension) =
      A;
  M.template block<baseDimension, baseDimension>(baseDimension,
                                                 2 * baseDimension) = B;
  M.template block<baseDimension, baseDimension>(2 * baseDimension,
                                                 2 * baseDimension) = A;
  const Augmented expM = M.exp();
  const BaseJacobian J =
      expM.template block<baseDimension, baseDimension>(0, baseDimension);
  const BaseJacobian Q =
      expM.template block<baseDimension, baseDimension>(0, 2 * baseDimension);
  Jacobian result = Jacobian::Zero();
  result.template topLeftCorner<baseDimension, baseDimension>() = J;
  result.template bottomRightCorner<baseDimension, baseDimension>() = J;
  result.template bottomLeftCorner<baseDimension, baseDimension>() = Q;
  return result;
}

template <typename G>
typename TangentLieGroup<G>::Jacobian TangentLieGroup<G>::AdjointMap() const {
  const BaseJacobian Ad = traits<G>::AdjointMap(this->first);
  const BaseJacobian adV = G::adjointMap(this->second);
  Jacobian result = Jacobian::Zero();
  result.template topLeftCorner<baseDimension, baseDimension>() = Ad;
  result.template bottomRightCorner<baseDimension, baseDimension>() = Ad;
  result.template bottomLeftCorner<baseDimension, baseDimension>() = adV * Ad;
  return result;
}

template <typename G>
typename TangentLieGroup<G>::Jacobian TangentLieGroup<G>::adjointMap(
    const TangentVector& xi) {
  const auto [u, v] = split(xi);
  const BaseJacobian adU = G::adjointMap(u);
  Jacobian result = Jacobian::Zero();
  result.template topLeftCorner<baseDimension, baseDimension>() = adU;
  result.template bottomRightCorner<baseDimension, baseDimension>() = adU;
  result.template bottomLeftCorner<baseDimension, baseDimension>() =
      G::adjointMap(v);
  return result;
}

template <typename G>
std::pair<typename TangentLieGroup<G>::BaseTangent,
          typename TangentLieGroup<G>::BaseTangent>
TangentLieGroup<G>::split(const TangentVector& xi) {
  return {xi.template head<baseDimension>(), xi.template tail<baseDimension>()};
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
