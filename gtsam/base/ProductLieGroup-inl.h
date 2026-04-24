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
 * @brief Internals for ProductLieGroup.h, not for general consumption
 */

#pragma once

#include <unsupported/Eigen/MatrixFunctions>

namespace gtsam {

// ---------------------------------------------------------------------------
// phi0Kernel: compute φ₀(A)=exp(A) directly.
// ---------------------------------------------------------------------------
template <typename G, typename H, typename Action>
typename ProductLieGroup<G, H, Action>::Jacobian2
ProductLieGroup<G, H, Action>::phi0Kernel(const Jacobian2& A) {
  Jacobian2 phi0;
  if constexpr (secondDynamic) phi0.resize(A.rows(), A.cols());
  phi0 = A.exp();
  return phi0;
}

// ---------------------------------------------------------------------------
// phi1Kernel: compute φ₁(A)=Σ Aᵏ/(k+1)! from one block matrix exponential.
//
// Identity: exp([[A, I], [0, 0]]) = [[exp(A), φ₁(A)], [0, I]]
//   Proof: M = [[A,I],[0,0]]; M^k = [[A^k, A^{k-1}],[0,0]] for k≥1.
//   exp(M) = I + Σ M^k/k! = [[exp(A), Σ A^{k-1}/k!],[0,I]] = [[φ₀,φ₁],[0,I]].
//
// This is the generic path for arbitrary generators, not a faster replacement
// for closed-form kernels available for specific groups such as Rot3.
// Only called when ProductLieGroup::hasGenerator is true.
// ---------------------------------------------------------------------------
template <typename G, typename H, typename Action>
typename ProductLieGroup<G, H, Action>::Jacobian2
ProductLieGroup<G, H, Action>::phi1Kernel(const Jacobian2& A) {
  const int r = static_cast<int>(A.rows());
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(2 * r, 2 * r);
  M.topLeftCorner(r, r) = A;
  M.topRightCorner(r, r) = Eigen::MatrixXd::Identity(r, r);
  const Eigen::MatrixXd expM = M.exp();
  Jacobian2 phi1;
  if constexpr (secondDynamic) phi1.resize(r, r);
  phi1 = expM.topRightCorner(r, r);
  return phi1;
}



template <typename G, typename H, typename Action>
ProductLieGroup<G, H, Action> ProductLieGroup<G, H, Action>::operator*(
    const ProductLieGroup& other) const {
  checkMatchingDimensions(other, "operator*");
  if constexpr (isDirectProduct) {
    // Direct product: (g₁,h₁)·(g₂,h₂) = (g₁g₂, h₁h₂)
    return ProductLieGroup(traits<G>::Compose(this->first, other.first),
                           traits<H>::Compose(this->second, other.second));
  } else {
    // Semidirect product: (g₁,h₁)·(g₂,h₂) = (g₁g₂, h₁·φ(g₁,h₂)).
    // The LHS G-component acts on the RHS H-component before composing in H.
    const H actedSecond = Action{}(this->first, other.second);
    return ProductLieGroup(traits<G>::Compose(this->first, other.first),
                           traits<H>::Compose(this->second, actedSecond));
  }
}

template <typename G, typename H, typename Action>
ProductLieGroup<G, H, Action> ProductLieGroup<G, H, Action>::inverse() const {
  const G gInv = traits<G>::Inverse(this->first);
  const H hInv = traits<H>::Inverse(this->second);
  if constexpr (isDirectProduct) {
    // Direct product: (g,h)⁻¹ = (g⁻¹, h⁻¹)
    return ProductLieGroup(gInv, hInv);
  } else {
    // Semidirect product: (g,h)⁻¹ = (g⁻¹, φ(g⁻¹, h⁻¹)).
    // g⁻¹ must also un-rotate h⁻¹ to undo the action baked into the group law.
    return ProductLieGroup(gInv, Action{}(gInv, hInv));
  }
}

template <typename G, typename H, typename Action>
ProductLieGroup<G, H, Action> ProductLieGroup<G, H, Action>::retract(
    const TangentVector& v, ChartJacobian H1, ChartJacobian H2) const {
  const size_t d1 = firstDim();
  const size_t d2 = secondDim();
  const size_t d = combinedDimension(d1, d2);
  if (static_cast<size_t>(v.size()) != d) {
    throw std::invalid_argument(
        "ProductLieGroup::retract tangent dimension does not match product "
        "dimension");
  }
  // Expmap-based retract works for both direct and semidirect products:
  //   retract(p, v) = p · Expmap(v)
  // The chart Jacobians are H1 = Ad(Expmap(v)⁻¹) and H2 = D_Expmap(v).
  // For a direct product these are block-diagonal and match the componentwise
  // formula; for a semidirect product they carry the full coupled structure.
  //
  // We pre-split v here so the 2-arg Expmap overload is used, which handles
  // the dynamic-dynamic case correctly (the 1-arg form can't infer the split
  // without an instance to query d1 and d2 from).
  Matrix D_v1, D_v2;
  using DynJ = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;
  const ProductLieGroup delta =
      Expmap(tangentSegment<G>(v, 0, d1), tangentSegment<H>(v, d1, d2),
             H2 ? DynJ(D_v1) : DynJ(), H2 ? DynJ(D_v2) : DynJ());
  const ProductLieGroup result = compose(delta);
  if (H1) *H1 = delta.inverse().AdjointMap();
  if (H2) {
    *H2 = zeroJacobian(d);
    H2->leftCols(d1) = D_v1;
    H2->rightCols(d2) = D_v2;
  }
  return result;
}

template <typename G, typename H, typename Action>
typename ProductLieGroup<G, H, Action>::TangentVector
ProductLieGroup<G, H, Action>::localCoordinates(const ProductLieGroup& g,
                                                ChartJacobian H1,
                                                ChartJacobian H2) const {
  checkMatchingDimensions(g, "localCoordinates");
  // Logmap-based local coordinates work for both direct and semidirect
  // products:
  //   localCoordinates(p, q) = Logmap(p⁻¹·q)
  // For a direct product Logmap is componentwise; for a semidirect product it
  // carries the full coupled Lie algebra structure from Action::Logmap.
  const ProductLieGroup relative = between(g);
  Jacobian D_logmap;
  TangentVector v =
      Logmap(relative, H1 || H2 ? ChartJacobian(&D_logmap) : ChartJacobian());
  if (H1) *H1 = -D_logmap * relative.inverse().AdjointMap();
  if (H2) *H2 = D_logmap;
  return v;
}

template <typename G, typename H, typename Action>
ProductLieGroup<G, H, Action> ProductLieGroup<G, H, Action>::compose(
    const ProductLieGroup& other, ChartJacobian H1, ChartJacobian H2) const {
  checkMatchingDimensions(other, "compose");
  const ProductLieGroup result = (*this) * other;
  if (H1) *H1 = other.inverse().AdjointMap();
  if (H2) *H2 = identityJacobian(dim());
  return result;
}

template <typename G, typename H, typename Action>
ProductLieGroup<G, H, Action> ProductLieGroup<G, H, Action>::between(
    const ProductLieGroup& other, ChartJacobian H1, ChartJacobian H2) const {
  checkMatchingDimensions(other, "between");
  const ProductLieGroup result = this->inverse() * other;
  if (H1) *H1 = -result.inverse().AdjointMap();
  if (H2) *H2 = identityJacobian(dim());
  return result;
}

template <typename G, typename H, typename Action>
ProductLieGroup<G, H, Action> ProductLieGroup<G, H, Action>::inverse(
    ChartJacobian D) const {
  if (D) *D = -AdjointMap();
  return inverse();
}

template <typename G, typename H, typename Action>
ProductLieGroup<G, H, Action> ProductLieGroup<G, H, Action>::Expmap(
    const TangentVector& v, ChartJacobian Hv) {
  size_t firstDimension = 0;
  size_t secondDimension = 0;
  if constexpr (firstDynamic && secondDynamic) {
    if (v.size() == 0) {
      if (Hv) *Hv = Matrix::Zero(0, 0);
      return ProductLieGroup();
    }
    throw std::invalid_argument(
        "ProductLieGroup::Expmap requires split tangent vectors when both "
        "factors are dynamic");
  } else if constexpr (firstDynamic) {
    if (v.size() < dimension2) {
      throw std::invalid_argument(
          "ProductLieGroup::Expmap tangent dimension is too small for the "
          "fixed second factor");
    }
    firstDimension = static_cast<size_t>(v.size() - dimension2);
    secondDimension = static_cast<size_t>(dimension2);
  } else if constexpr (secondDynamic) {
    if (v.size() < dimension1) {
      throw std::invalid_argument(
          "ProductLieGroup::Expmap tangent dimension is too small for the "
          "fixed first factor");
    }
    firstDimension = static_cast<size_t>(dimension1);
    secondDimension = static_cast<size_t>(v.size() - dimension1);
  } else {
    firstDimension = static_cast<size_t>(dimension1);
    secondDimension = static_cast<size_t>(dimension2);
  }
  if (static_cast<size_t>(v.size()) !=
      combinedDimension(firstDimension, secondDimension)) {
    throw std::invalid_argument(
        "ProductLieGroup::Expmap tangent dimension does not match product "
        "dimension");
  }

  Matrix D_g_first;
  Matrix D_h_second;
  const auto v1 = tangentSegment<G>(v, 0, firstDimension);
  const auto v2 = tangentSegment<H>(v, firstDimension, secondDimension);
  ProductLieGroup result =
      Expmap(v1, v2,
             Hv ? OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>(D_g_first)
                : OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>(),
             Hv ? OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>(D_h_second)
                : OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>());
  if (Hv) {
    const size_t productDimension =
        combinedDimension(firstDimension, secondDimension);
    *Hv = zeroJacobian(productDimension);
    Hv->block(0, 0, productDimension, firstDimension) = D_g_first;
    Hv->block(0, firstDimension, productDimension, secondDimension) =
        D_h_second;
  }
  return result;
}

template <typename G, typename H, typename Action>
ProductLieGroup<G, H, Action> ProductLieGroup<G, H, Action>::Expmap(
    const Eigen::Ref<const typename traits<G>::TangentVector>& v1,
    const Eigen::Ref<const typename traits<H>::TangentVector>& v2,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2) {
  if constexpr (isDirectProduct) {
    // Direct product: the two Lie algebras are independent, so the exponential
    // is componentwise: exp(v₁,v₂) = (exp_G(v₁), exp_H(v₂)).
    // The Jacobian columns split cleanly between the G and H blocks.
    const size_t d1 = static_cast<size_t>(v1.size());
    const size_t d2 = static_cast<size_t>(v2.size());
    const size_t d = combinedDimension(d1, d2);
    Jacobian1 D_g_first;
    Jacobian2 D_h_second;
    G g = traits<G>::Expmap(v1, H1 ? &D_g_first : nullptr);
    H h = traits<H>::Expmap(v2, H2 ? &D_h_second : nullptr);
    if (H1) {
      *H1 = Matrix::Zero(d, d1);
      H1->block(0, 0, d1, d1) = D_g_first;
    }
    if (H2) {
      *H2 = Matrix::Zero(d, d2);
      H2->block(d1, 0, d2, d2) = D_h_second;
    }
    return ProductLieGroup(g, h);
  } else if constexpr (hasGenerator) {
    // Generic semidirect Expmap for vector-space H via the φ₁ kernel:
    //   Expmap(u, v) = (expG(u),  φ₁(Aφ(u)) · v)
    // where Aφ(u) = Action::generator(u) is the infinitesimal generator and
    // φ₁(A) = Σ Aᵏ/(k+1)! is computed by phi1Kernel.
    const size_t d1 = static_cast<size_t>(v1.size());
    const size_t d2 = static_cast<size_t>(v2.size());
    const size_t d = combinedDimension(d1, d2);

    const Jacobian2 A = Action::generator(v1);
    const Jacobian2 phi1 = phi1Kernel(A);

    Jacobian1 D_G;
    const G g = traits<G>::Expmap(v1, H1 ? &D_G : nullptr);
    const H h = phi1 * v2;

    if (H1 || H2) {
      const Jacobian2 phi0 = phi0Kernel(A);
      const auto phi0Solver = phi0.lu();

      if (H1) {
        // Top rows: D Exp_G(u) in the output chart (from traits::Expmap).
        // Bottom rows: D(φ₁(Aφ(u))·v) pulled back by φ₀(A)⁻¹, because
        // Expmap Jacobians are expressed in local coordinates at Expmap(u,v).
        // Linearity of generator: generator(u ± ε·eⱼ) = A ± ε·generator(eⱼ),
        // so function values are exact and the only error is O(ε²) truncation.
        *H1 = Matrix::Zero(d, d1);
        H1->topRows(d1) = D_G;
        const double eps = 1e-5;
        typename traits<G>::TangentVector ej;
        if constexpr (firstDynamic) ej.resize(static_cast<Eigen::Index>(d1));
        ej.setZero();
        for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(d1); ++j) {
          ej(j) = 1.0;
          const Jacobian2 Bj = Action::generator(ej);
          const Jacobian2 phi1p = phi1Kernel(A + eps * Bj);
          const Jacobian2 phi1m = phi1Kernel(A - eps * Bj);
          const typename traits<H>::TangentVector dh =
              (phi1p - phi1m) * v2 / (2.0 * eps);
          H1->col(j).tail(d2) = phi0Solver.solve(dh);
          ej(j) = 0.0;
        }
      }
      if (H2) {
        // ∂(φ₁(A)·v)/∂v = φ₁(A) in coordinates, then pulled back by φ₀(A)⁻¹
        // to match the output chart. For SO(3) this is Rᵀ J_l = J_r.
        *H2 = Matrix::Zero(d, d2);
        H2->bottomRows(d2) = phi0Solver.solve(phi1);
      }
    }
    return ProductLieGroup(g, h);
  } else {
    static_assert(hasGenerator,
                  "ProductLieGroup semidirect Expmap requires H to be a "
                  "fixed-size Eigen column vector and Action::generator(u).");
  }
}

template <typename G, typename H, typename Action>
typename ProductLieGroup<G, H, Action>::TangentVector
ProductLieGroup<G, H, Action>::Logmap(const ProductLieGroup& p,
                                      ChartJacobian Hp) {
  if constexpr (isDirectProduct) {
    // Direct product: the inverse of a componentwise Expmap is componentwise,
    // so log(g,h) = (log_G(g), log_H(h)) with a block-diagonal Jacobian.
    const size_t firstDimension = p.firstDim();
    const size_t secondDimension = p.secondDim();
    const size_t productDimension =
        combinedDimension(firstDimension, secondDimension);
    Jacobian1 D_g_first;
    Jacobian2 D_h_second;
    typename traits<G>::TangentVector v1 =
        traits<G>::Logmap(p.first, Hp ? &D_g_first : nullptr);
    typename traits<H>::TangentVector v2 =
        traits<H>::Logmap(p.second, Hp ? &D_h_second : nullptr);
    TangentVector v =
        makeTangentVector(v1, v2, firstDimension, secondDimension);
    if (Hp) {
      *Hp = zeroJacobian(productDimension);
      Hp->block(0, 0, firstDimension, firstDimension) = D_g_first;
      Hp->block(firstDimension, firstDimension, secondDimension,
                secondDimension) = D_h_second;
    }
    return v;
  } else if constexpr (hasGenerator) {
    // Generic semidirect Logmap for vector-space H via the φ₁ kernel:
    //   Logmap(g, h) = (logG(g),  φ₁(Aφ(logG(g)))⁻¹ · h)
    // This is the exact inverse of the Expmap formula above.
    const size_t d1 = p.firstDim();
    const size_t d2 = p.secondDim();
    const size_t d = combinedDimension(d1, d2);

    Jacobian1 D_G;
    const auto v1 = traits<G>::Logmap(p.first, Hp ? &D_G : nullptr);
    const Jacobian2 A = Action::generator(v1);
    const Jacobian2 phi1 = phi1Kernel(A);
    const auto phi1Solver = phi1.lu();
    const typename traits<H>::TangentVector v2 = phi1Solver.solve(p.second);
    TangentVector v = makeTangentVector(v1, v2, d1, d2);

    if (Hp) {
      const Jacobian2 phi0 = phi0Kernel(A);
      *Hp = zeroJacobian(d);
      // Top-left: ∂logG(g)/∂g — analytic.
      Hp->topLeftCorner(d1, d1) = D_G;
      // Top-right: 0 — logG doesn't depend on h (already zeroed).
      // Bottom-right: ∂v₂/∂(δh) = φ₁(A)⁻¹ · φ₀(A).
      // Perturbing h by δh in the semidirect frame moves h by φ₀(A)·δh
      // (because Expmap(0, δh) = (I, δh) and (g,h)*(I,δh) → h + φ₀(A)·δh),
      // so ∂v₂/∂(δh) = φ₁⁻¹ · φ₀.
      Hp->bottomRightCorner(d2, d2) = phi1Solver.solve(phi0);
      // Bottom-left: ∂v₂/∂(δg) — perturbing g via right multiplication
      // changes u=logG(g) non-linearly; computed by central differences.
      // Values are exact (matrix exp), so error is O(ε²).
      const double eps = 1e-5;
      for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(d1); ++j) {
        typename traits<G>::TangentVector ej;
        if constexpr (firstDynamic) ej.resize(static_cast<Eigen::Index>(d1));
        ej.setZero();
        ej(j) = eps;
        const G gp = traits<G>::Compose(p.first, traits<G>::Expmap(ej));
        ej(j) = -eps;
        const G gm = traits<G>::Compose(p.first, traits<G>::Expmap(ej));
        const Jacobian2 phi1p =
            phi1Kernel(Action::generator(traits<G>::Logmap(gp)));
        const Jacobian2 phi1m =
            phi1Kernel(Action::generator(traits<G>::Logmap(gm)));
        Hp->col(j).tail(d2) =
            (phi1p.lu().solve(p.second) - phi1m.lu().solve(p.second)) / (2.0 * eps);
      }
    }
    return v;
  } else {
    static_assert(hasGenerator,
                  "ProductLieGroup semidirect Logmap requires H to be a "
                  "fixed-size Eigen column vector and Action::generator(u).");
  }
}

template <typename G, typename H, typename Action>
ProductLieGroup<G, H, Action> ProductLieGroup<G, H, Action>::expmap(
    const TangentVector& v) const {
  return compose(ProductLieGroup::Expmap(v));
}

template <typename G, typename H, typename Action>
typename ProductLieGroup<G, H, Action>::Jacobian
ProductLieGroup<G, H, Action>::AdjointMap() const {
  if constexpr (isDirectProduct) {
    // Direct product: the two algebras are independent, so the adjoint is
    // block-diagonal: Ad_(g,h) = diag(Ad_G(g), Ad_H(h)).
    const auto adjG = traits<G>::AdjointMap(this->first);
    const auto adjH = traits<H>::AdjointMap(this->second);
    const size_t d1 = static_cast<size_t>(adjG.rows());
    const size_t d2 = static_cast<size_t>(adjH.rows());
    Jacobian adj = zeroJacobian(d1 + d2);
    adj.block(0, 0, d1, d1) = adjG;
    adj.block(d1, d1, d2, d2) = adjH;
    return adj;
  } else {
    // Semidirect product: the action couples the algebra blocks, producing an
    // off-diagonal term. The formula is assembled from the action's Jacobians:
    //
    //   Ad_(g,h) = [[ Ad_G(g),              0   ],
    //               [ -Jg * Ad_G(g),        Jh  ]]
    //
    // where Jg = D_g φ(e_G, h)  (action Jacobian w.r.t. G at identity)
    //       Jh = D_h φ(g, e_H)  (action Jacobian w.r.t. H at identity)
    //
    // Both come directly from the action's operator() Jacobian arguments.
    const size_t d1 = firstDim();
    const size_t d2 = secondDim();
    const size_t d = combinedDimension(d1, d2);

    const Action action{};

    // Jg: DimH × DimG — evaluated at (e_G, h)
    ActionJacobianG Jg;
    action(defaultIdentity<G>(), this->second, &Jg, {});

    // Jh: DimH × DimH — evaluated at (g, e_H)
    Jacobian2 Jh;
    action(this->first, defaultIdentity<H>(), {}, &Jh);

    const auto adjG = traits<G>::AdjointMap(this->first);

    Jacobian adj = zeroJacobian(d);
    adj.block(0, 0, d1, d1) = adjG;
    adj.block(d1, d1, d2, d2) = Jh;
    adj.block(d1, 0, d2, d1) = -Jg * adjG;
    return adj;
  }
}

template <typename G, typename H, typename Action>
template <typename T>
T ProductLieGroup<G, H, Action>::defaultIdentity() {
  if constexpr (traits<T>::dimension == Eigen::Dynamic) {
    return T();
  } else {
    return traits<T>::Identity();
  }
}

template <typename G, typename H, typename Action>
template <typename T, int Dim>
typename traits<T>::TangentVector ProductLieGroup<G, H, Action>::tangentSegment(
    const TangentVector& v, size_t start, size_t runtimeDimension) {
  const int startIndex = static_cast<int>(start);
  const int runtimeIndex = static_cast<int>(runtimeDimension);
  if constexpr (Dim == Eigen::Dynamic) {
    return v.segment(startIndex, runtimeIndex);
  } else {
    static_cast<void>(runtimeDimension);
    return v.template segment<Dim>(startIndex);
  }
}

template <typename G, typename H, typename Action>
typename ProductLieGroup<G, H, Action>::TangentVector
ProductLieGroup<G, H, Action>::makeTangentVector(
    const typename traits<G>::TangentVector& v1,
    const typename traits<H>::TangentVector& v2, size_t firstDimension,
    size_t secondDimension) {
  const int firstIndex = static_cast<int>(firstDimension);
  const int secondIndex = static_cast<int>(secondDimension);
  if constexpr (dimension == Eigen::Dynamic) {
    TangentVector v(combinedDimension(firstDimension, secondDimension));
    v.segment(0, firstIndex) = v1;
    v.segment(firstIndex, secondIndex) = v2;
    return v;
  } else {
    static_cast<void>(firstDimension);
    static_cast<void>(secondDimension);
    TangentVector v;
    v << v1, v2;
    return v;
  }
}

template <typename G, typename H, typename Action>
typename ProductLieGroup<G, H, Action>::Jacobian
ProductLieGroup<G, H, Action>::zeroJacobian(size_t productDimension) {
  if constexpr (dimension == Eigen::Dynamic) {
    return Jacobian::Zero(productDimension, productDimension);
  } else {
    static_cast<void>(productDimension);
    return Jacobian::Zero();
  }
}

template <typename G, typename H, typename Action>
typename ProductLieGroup<G, H, Action>::Jacobian
ProductLieGroup<G, H, Action>::identityJacobian(size_t productDimension) {
  if constexpr (dimension == Eigen::Dynamic) {
    return Jacobian::Identity(productDimension, productDimension);
  } else {
    static_cast<void>(productDimension);
    return Jacobian::Identity();
  }
}

template <typename G, typename H, typename Action>
void ProductLieGroup<G, H, Action>::checkMatchingDimensions(
    const ProductLieGroup& other, const char* operation) const {
  if (firstDim() != other.firstDim() || secondDim() != other.secondDim()) {
    throw std::invalid_argument(std::string("ProductLieGroup::") + operation +
                                " requires matching component dimensions");
  }
}

template <typename G, typename H, typename Action>
void ProductLieGroup<G, H, Action>::print(const std::string& s) const {
  std::cout << s << "ProductLieGroup" << std::endl;
  traits<G>::Print(this->first, "  first");
  traits<H>::Print(this->second, "  second");
}

template <typename G, int N, typename Derived>
void PowerLieGroupBase<G, N, Derived>::checkDynamicTangentSize(
    const TangentVector& v, size_t count, const char* operation) {
  if constexpr (isDynamic) {
    if (static_cast<size_t>(v.size()) != totalDimension(count)) {
      throw std::invalid_argument(std::string("PowerLieGroup::") + operation +
                                  " tangent dimension does not match group "
                                  "dimension");
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
    return v.segment(offset(i), baseDimension);
  } else {
    return v.template segment<baseDimension>(i * baseDimension);
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
    v.segment(offset(i), baseDimension) = vi;
  } else {
    v.template segment<baseDimension>(i * baseDimension) = vi;
  }
}

template <typename G, int N, typename Derived>
template <typename MatrixType>
void PowerLieGroupBase<G, N, Derived>::assignJacobianBlock(
    MatrixType& H, size_t i, const BaseJacobian& block) {
  if constexpr (isDynamic) {
    H.block(offset(i), offset(i), baseDimension, baseDimension) = block;
  } else {
    H.template block<baseDimension, baseDimension>(i * baseDimension,
                                                   i * baseDimension) = block;
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
Derived PowerLieGroupBase<G, N, Derived>::compose(const Derived& other,
                                                  ChartJacobian H1,
                                                  ChartJacobian H2) const {
  checkMatchingCounts(other, "compose");
  const size_t count = componentCount();
  JacobianStorage jacobians = makeJacobianStorage(count);
  Derived result = makeResult(count);
  for (size_t i = 0; i < count; ++i) {
    result[i] = traits<G>::Compose(derived()[i], other[i],
                                   H1 ? &jacobians[i] : nullptr);
  }
  fillJacobianBlocks(H1, jacobians, count);
  if (H2) *H2 = identityJacobian(count);
  return result;
}

template <typename G, int N, typename Derived>
Derived PowerLieGroupBase<G, N, Derived>::between(const Derived& other,
                                                  ChartJacobian H1,
                                                  ChartJacobian H2) const {
  checkMatchingCounts(other, "between");
  const size_t count = componentCount();
  JacobianStorage jacobians = makeJacobianStorage(count);
  Derived result = makeResult(count);
  for (size_t i = 0; i < count; ++i) {
    result[i] = traits<G>::Between(derived()[i], other[i],
                                   H1 ? &jacobians[i] : nullptr);
  }
  fillJacobianBlocks(H1, jacobians, count);
  if (H2) *H2 = identityJacobian(count);
  return result;
}

template <typename G, int N, typename Derived>
Derived PowerLieGroupBase<G, N, Derived>::inverse(ChartJacobian D) const {
  const size_t count = componentCount();
  JacobianStorage jacobians = makeJacobianStorage(count);
  Derived result = makeResult(count);
  for (size_t i = 0; i < count; ++i) {
    result[i] = traits<G>::Inverse(derived()[i], D ? &jacobians[i] : nullptr);
  }
  fillJacobianBlocks(D, jacobians, count);
  return result;
}

template <typename G, int N, typename Derived>
Derived PowerLieGroupBase<G, N, Derived>::Expmap(const TangentVector& v,
                                                 ChartJacobian Hv) {
  size_t count = 0;
  if constexpr (isDynamic) {
    if (v.size() % baseDimension != 0) {
      throw std::invalid_argument(
          "PowerLieGroup::Expmap tangent dimension must be divisible by base "
          "group dimension");
    }
    count = static_cast<size_t>(v.size() /
                                static_cast<Eigen::Index>(baseDimension));
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

template <typename G, int N, typename Derived>
typename PowerLieGroupBase<G, N, Derived>::Jacobian
PowerLieGroupBase<G, N, Derived>::identityJacobian(size_t count) {
  if constexpr (isDynamic) {
    return Jacobian::Identity(totalDimension(count), totalDimension(count));
  } else {
    static_cast<void>(count);
    return Jacobian::Identity();
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
