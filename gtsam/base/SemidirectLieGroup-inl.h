/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SemidirectLieGroup-inl.h
 * @date August, 2026
 * @author Frank Dellaert
 * @author Rohan Bansal
 * @author Alessandro Fornasier
 * @brief Internal implementation of SemidirectLieGroup.
 */

#pragma once

#include <unsupported/Eigen/MatrixFunctions>

namespace {

template <int D, int Blocks, int Extra = 0>
inline constexpr int semidirectAugmentedDimension =
    D == Eigen::Dynamic ? Eigen::Dynamic : Blocks * D + Extra;

template <int D, int Blocks, int Extra = 0>
using SemidirectAugmentedMatrix =
    Eigen::Matrix<double, semidirectAugmentedDimension<D, Blocks, Extra>,
                  semidirectAugmentedDimension<D, Blocks, Extra>>;

template <typename Dst, typename Src>
void semidirectAssignBlock(const Src& source, size_t row, size_t column,
                           Dst* destination) {
  constexpr int rows = Src::RowsAtCompileTime;
  constexpr int columns = Src::ColsAtCompileTime;
  if constexpr (rows != Eigen::Dynamic && columns != Eigen::Dynamic) {
    destination->template block<rows, columns>(row, column) = source;
  } else {
    destination->block(row, column, source.rows(), source.cols()) = source;
  }
}

}  // namespace

namespace gtsam {

template <typename G, typename H, typename Action>
typename SemidirectLieGroup<G, H, Action>::Phi1KernelResult
SemidirectLieGroup<G, H, Action>::phi1Kernel(const Jacobian2& A) {
  const int r = A.rows();
  using Augmented = SemidirectAugmentedMatrix<m, 2>;
  Augmented M = Augmented::Zero(2 * r, 2 * r);
  M.topLeftCorner(r, r) = A;
  M.topRightCorner(r, r).setIdentity();
  const Augmented expM = M.exp();
  return {expM.topLeftCorner(r, r), expM.topRightCorner(r, r)};
}

template <typename G, typename H, typename Action>
typename traits<H>::TangentVector
SemidirectLieGroup<G, H, Action>::phi1FrechetAction(
    const Jacobian2& A, const Jacobian2& B,
    const typename traits<H>::TangentVector& v) {
  const int r = A.rows();
  using Augmented = SemidirectAugmentedMatrix<m, 2, 1>;
  Augmented M = Augmented::Zero(2 * r + 1, 2 * r + 1);
  M.block(0, 0, r, r) = A;
  M.block(0, r, r, r) = B;
  M.block(r, r, r, r) = A;
  M.block(r, 2 * r, r, 1) = v;
  return M.exp().block(0, 2 * r, r, 1);
}

template <typename G, typename H, typename Action>
typename SemidirectLieGroup<G, H, Action>::Jacobian
SemidirectLieGroup<G, H, Action>::rightJacobian(const TangentVector& xi) {
  const int d = xi.size();
  using Augmented = SemidirectAugmentedMatrix<dimension, 2>;
  Augmented M = Augmented::Zero(2 * d, 2 * d);
  M.topLeftCorner(d, d) = -adjointMap(xi);
  M.topRightCorner(d, d).setIdentity();
  return M.exp().topRightCorner(d, d);
}

template <typename G, typename H, typename Action>
SemidirectLieGroup<G, H, Action> SemidirectLieGroup<G, H, Action>::operator*(
    const SemidirectLieGroup& other) const {
  checkMatchingDimensions(other, "operator*");
  const H acted = Action{}(this->first, other.second);
  return {traits<G>::Compose(this->first, other.first),
          traits<H>::Compose(this->second, acted)};
}

template <typename G, typename H, typename Action>
SemidirectLieGroup<G, H, Action> SemidirectLieGroup<G, H, Action>::inverse()
    const {
  const G gInverse = traits<G>::Inverse(this->first);
  return {gInverse, Action{}(gInverse, traits<H>::Inverse(this->second))};
}

template <typename G, typename H, typename Action>
SemidirectLieGroup<G, H, Action> SemidirectLieGroup<G, H, Action>::retract(
    const TangentVector& xi, ChartJacobian H1, ChartJacobian H2) const {
  const size_t d1 = firstDim(), d2 = secondDim(), d = d1 + d2;
  if (static_cast<size_t>(xi.size()) != d) {
    throw std::invalid_argument(
        "SemidirectLieGroup::retract tangent dimension mismatch");
  }
  Matrix D1, D2;
  using DynamicJacobian = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;
  const SemidirectLieGroup delta =
      Expmap(tangentSegment<G>(xi, 0, d1), tangentSegment<H>(xi, d1, d2),
             H2 ? DynamicJacobian(D1) : DynamicJacobian(),
             H2 ? DynamicJacobian(D2) : DynamicJacobian());
  const SemidirectLieGroup result = compose(delta);
  if (H1) *H1 = delta.inverse().AdjointMap();
  if (H2) {
    *H2 = zeroJacobian(d);
    H2->leftCols(d1) = D1;
    H2->rightCols(d2) = D2;
  }
  return result;
}

template <typename G, typename H, typename Action>
typename SemidirectLieGroup<G, H, Action>::TangentVector
SemidirectLieGroup<G, H, Action>::localCoordinates(
    const SemidirectLieGroup& other, ChartJacobian H1, ChartJacobian H2) const {
  checkMatchingDimensions(other, "localCoordinates");
  const SemidirectLieGroup relative = between(other);
  Jacobian Dlog;
  const TangentVector xi =
      Logmap(relative, H1 || H2 ? ChartJacobian(&Dlog) : ChartJacobian());
  if (H1) *H1 = -Dlog * relative.inverse().AdjointMap();
  if (H2) *H2 = Dlog;
  return xi;
}

template <typename G, typename H, typename Action>
SemidirectLieGroup<G, H, Action> SemidirectLieGroup<G, H, Action>::Expmap(
    const TangentVector& xi, ChartJacobian D) {
  size_t d1;
  if constexpr (firstDynamic) {
    if (xi.size() < m) {
      throw std::invalid_argument(
          "SemidirectLieGroup::Expmap tangent dimension is too small");
    }
    d1 = static_cast<size_t>(xi.size() - m);
  } else {
    d1 = n;
  }
  constexpr size_t d2 = m;
  if (static_cast<size_t>(xi.size()) != d1 + d2) {
    throw std::invalid_argument(
        "SemidirectLieGroup::Expmap tangent dimension mismatch");
  }
  Matrix D1, D2;
  using DynamicJacobian = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;
  const SemidirectLieGroup result =
      Expmap(tangentSegment<G>(xi, 0, d1), tangentSegment<H>(xi, d1, d2),
             D ? DynamicJacobian(D1) : DynamicJacobian(),
             D ? DynamicJacobian(D2) : DynamicJacobian());
  if (D) {
    *D = zeroJacobian(d1 + d2);
    D->leftCols(d1) = D1;
    D->rightCols(d2) = D2;
  }
  return result;
}

template <typename G, typename H, typename Action>
SemidirectLieGroup<G, H, Action> SemidirectLieGroup<G, H, Action>::Expmap(
    const Eigen::Ref<const typename traits<G>::TangentVector>& u,
    const Eigen::Ref<const typename traits<H>::TangentVector>& v,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2) {
  const size_t d1 = u.size(), d2 = v.size(), d = d1 + d2;
  constexpr bool hasAdjoint =
      internal::SemidirectLieGroupHasAdjointMap<G>::value;
  Jacobian1 Dg;
  const G g = traits<G>::Expmap(u, H1 && !hasAdjoint ? &Dg : nullptr);

  if (H1) {
    if constexpr (hasAdjoint) {
      const TangentVector xi = makeTangentVector(u, v, d1, d2);
      const Jacobian derivative = rightJacobian(xi);
      const Jacobian2 actionJacobian = derivative.bottomRightCorner(d2, d2);
      const H h = Action{}(g, actionJacobian * v);
      *H1 = derivative.leftCols(d1);
      if (H2) *H2 = derivative.rightCols(d2);
      return {g, h};
    } else {
      const Jacobian2 A = Action::generator(u);
      const Phi1KernelResult kernels = phi1Kernel(A);
      const auto phi0Solver = kernels.phi0.lu();
      const H h = kernels.phi1 * v;
      *H1 = Matrix::Zero(d, d1);
      if constexpr (firstDynamic) {
        H1->topRows(d1) = Dg;
      } else {
        H1->template topLeftCorner<n, n>() = Dg;
      }
      typename traits<G>::TangentVector ej;
      if constexpr (firstDynamic) ej.resize(d1);
      ej.setZero();
      for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(d1); ++j) {
        ej(j) = 1.0;
        H1->col(j).tail(d2) =
            phi0Solver.solve(phi1FrechetAction(A, Action::generator(ej), v));
        ej(j) = 0.0;
      }
      if (H2) {
        *H2 = Matrix::Zero(d, d2);
        H2->bottomRows(d2) = phi0Solver.solve(kernels.phi1);
      }
      return {g, h};
    }
  }

  const Jacobian2 A = Action::generator(u);
  if (H2) {
    const Jacobian2 actionJacobian = phi1Kernel(-A).phi1;
    const H h = Action{}(g, actionJacobian * v);
    *H2 = Matrix::Zero(d, d2);
    H2->bottomRows(d2) = actionJacobian;
    return {g, h};
  }
  return {g, phi1Kernel(A).phi1 * v};
}

template <typename G, typename H, typename Action>
typename SemidirectLieGroup<G, H, Action>::TangentVector
SemidirectLieGroup<G, H, Action>::Logmap(const SemidirectLieGroup& p,
                                         ChartJacobian D) {
  const size_t d1 = p.firstDim(), d2 = p.secondDim(), d = d1 + d2;
  constexpr bool hasAdjoint =
      internal::SemidirectLieGroupHasAdjointMap<G>::value;
  Jacobian1 Dg;
  const auto u = traits<G>::Logmap(p.first, D && !hasAdjoint ? &Dg : nullptr);
  const Jacobian2 A = Action::generator(u);

  if (D) {
    if constexpr (hasAdjoint) {
      const auto solver = phi1Kernel(A).phi1.lu();
      const typename traits<H>::TangentVector v = solver.solve(p.second);
      const TangentVector xi = makeTangentVector(u, v, d1, d2);
      *D = rightJacobian(xi).partialPivLu().solve(identityJacobian(d));
      return xi;
    } else {
      const Phi1KernelResult kernels = phi1Kernel(A);
      const auto solver = kernels.phi1.lu();
      const typename traits<H>::TangentVector v = solver.solve(p.second);
      *D = zeroJacobian(d);
      if constexpr (firstDynamic) {
        D->topLeftCorner(d1, d1) = Dg;
      } else {
        D->template topLeftCorner<n, n>() = Dg;
      }
      D->bottomRightCorner(d2, d2) = solver.solve(kernels.phi0);
      for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(d1); ++j) {
        const Jacobian2 B = Action::generator(Dg.col(j));
        D->col(j).tail(d2) = -solver.solve(phi1FrechetAction(A, B, v));
      }
      return makeTangentVector(u, v, d1, d2);
    }
  }

  const Jacobian2 phi1 = phi1Kernel(A).phi1;
  const typename traits<H>::TangentVector v = phi1.lu().solve(p.second);
  return makeTangentVector(u, v, d1, d2);
}

template <typename G, typename H, typename Action>
typename SemidirectLieGroup<G, H, Action>::Jacobian
SemidirectLieGroup<G, H, Action>::AdjointMap() const {
  const size_t d1 = firstDim(), d2 = secondDim(), d = d1 + d2;
  ActionJacobianG Jg;
  Jacobian2 Jh;
  Action{}(defaultIdentity<G>(), this->second, &Jg, {});
  Action{}(this->first, traits<H>::Identity(), {}, &Jh);
  const Jacobian1 AdG = traits<G>::AdjointMap(this->first);
  Jacobian result = zeroJacobian(d);
  semidirectAssignBlock(AdG, 0, 0, &result);
  semidirectAssignBlock(Jh, d1, d1, &result);
  result.block(d1, 0, d2, d1) = -Jg * AdG;
  return result;
}

template <typename G, typename H, typename Action>
typename SemidirectLieGroup<G, H, Action>::Jacobian
SemidirectLieGroup<G, H, Action>::adjointMap(const TangentVector& xi) {
  const size_t d = xi.size();
  const size_t d2 = m;
  if (d < d2) {
    throw std::invalid_argument(
        "SemidirectLieGroup::adjointMap tangent dimension is too small");
  }
  const size_t d1 = d - d2;
  const auto u = tangentSegment<G>(xi, 0, d1);
  const auto v = tangentSegment<H>(xi, d1, d2);
  Jacobian result = zeroJacobian(d);
  semidirectAssignBlock(G::adjointMap(u), 0, 0, &result);

  typename traits<G>::TangentVector ei;
  if constexpr (firstDynamic) ei.resize(d1);
  ei.setZero();
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(d1); ++i) {
    ei(i) = 1.0;
    result.block(d1, i, d2, 1) = -(Action::generator(ei) * v);
    ei(i) = 0.0;
  }
  semidirectAssignBlock(Action::generator(u), d1, d1, &result);
  return result;
}

template <typename G, typename H, typename Action>
template <typename T>
T SemidirectLieGroup<G, H, Action>::defaultIdentity() {
  if constexpr (traits<T>::dimension == Eigen::Dynamic) {
    return T();
  } else {
    return traits<T>::Identity();
  }
}

template <typename G, typename H, typename Action>
template <typename T, int D>
typename traits<T>::TangentVector
SemidirectLieGroup<G, H, Action>::tangentSegment(const TangentVector& xi,
                                                 size_t start, size_t d) {
  if constexpr (D == Eigen::Dynamic) {
    return xi.segment(start, d);
  } else {
    return xi.template segment<D>(start);
  }
}

template <typename G, typename H, typename Action>
typename SemidirectLieGroup<G, H, Action>::TangentVector
SemidirectLieGroup<G, H, Action>::makeTangentVector(
    const typename traits<G>::TangentVector& u,
    const typename traits<H>::TangentVector& v, size_t d1, size_t d2) {
  if constexpr (dimension == Eigen::Dynamic) {
    TangentVector xi(d1 + d2);
    xi << u, v;
    return xi;
  } else {
    TangentVector xi;
    xi << u, v;
    return xi;
  }
}

template <typename G, typename H, typename Action>
typename SemidirectLieGroup<G, H, Action>::Jacobian
SemidirectLieGroup<G, H, Action>::zeroJacobian(size_t d) {
  if constexpr (dimension == Eigen::Dynamic) return Jacobian::Zero(d, d);
  return Jacobian::Zero();
}

template <typename G, typename H, typename Action>
typename SemidirectLieGroup<G, H, Action>::Jacobian
SemidirectLieGroup<G, H, Action>::identityJacobian(size_t d) {
  if constexpr (dimension == Eigen::Dynamic) return Jacobian::Identity(d, d);
  return Jacobian::Identity();
}

template <typename G, typename H, typename Action>
void SemidirectLieGroup<G, H, Action>::checkMatchingDimensions(
    const SemidirectLieGroup& other, const char* operation) const {
  if (firstDim() != other.firstDim() || secondDim() != other.secondDim()) {
    throw std::invalid_argument(std::string("SemidirectLieGroup::") +
                                operation + " dimension mismatch");
  }
}

template <typename G, typename H, typename Action>
void SemidirectLieGroup<G, H, Action>::print(const std::string& s) const {
  std::cout << s << "SemidirectLieGroup" << std::endl;
  traits<G>::Print(this->first, "  first: ");
  traits<H>::Print(this->second, "  second: ");
}

}  // namespace gtsam
