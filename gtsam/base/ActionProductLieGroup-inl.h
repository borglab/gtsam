/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ActionProductLieGroup-inl.h
 * @date April, 2026
 * @author Frank Dellaert
 * @brief Internals for ActionProductLieGroup.h, not for general consumption
 */

#pragma once

namespace gtsam {

template <typename G, typename H, typename Action>
ActionProductLieGroup<G, H, Action>
ActionProductLieGroup<G, H, Action>::operator*(const This& other) const {
  checkMatchingDimensions(other, "operator*");
  if constexpr (isDirectProduct) {
    return This(Base::operator*(other));
  } else {
    const Action action{};
    const H actedSecond = action(this->first, other.second);
    return This(traits<G>::Compose(this->first, other.first),
                traits<H>::Compose(this->second, actedSecond));
  }
}

template <typename G, typename H, typename Action>
ActionProductLieGroup<G, H, Action>
ActionProductLieGroup<G, H, Action>::inverse() const {
  if constexpr (isDirectProduct) {
    return This(Base::inverse());
  } else {
    const Action action{};
    const G gInv = traits<G>::Inverse(this->first);
    const H hInv = traits<H>::Inverse(this->second);
    return This(gInv, action(gInv, hInv));
  }
}

template <typename G, typename H, typename Action>
ActionProductLieGroup<G, H, Action>
ActionProductLieGroup<G, H, Action>::retract(const TangentVector& v,
                                             ChartJacobian H1,
                                             ChartJacobian H2) const {
  if constexpr (isDirectProduct) {
    return This(Base::retract(v, H1, H2));
  } else {
    const size_t productDimension =
        combinedDimension(this->firstDim(), this->secondDim());
    if (static_cast<size_t>(v.size()) != productDimension) {
      throw std::invalid_argument(
          "ActionProductLieGroup::retract tangent dimension does not match "
          "product dimension");
    }
    Jacobian D_g_v;
    This g =
        This::Expmap(v, H2 ? OptionalJacobian<dimension, dimension>(&D_g_v)
                           : ChartJacobian());
    This h = compose(g);
    if (H1) *H1 = g.inverse().AdjointMap();
    if (H2) *H2 = D_g_v;
    return h;
  }
}

template <typename G, typename H, typename Action>
typename ActionProductLieGroup<G, H, Action>::TangentVector
ActionProductLieGroup<G, H, Action>::localCoordinates(const This& g,
                                                      ChartJacobian H1,
                                                      ChartJacobian H2) const {
  checkMatchingDimensions(g, "localCoordinates");
  if constexpr (isDirectProduct) {
    return Base::localCoordinates(g, H1, H2);
  } else {
    This h = between(g);
    Jacobian D_v_h;
    TangentVector v = This::Logmap(
        h, H1 || H2 ? OptionalJacobian<dimension, dimension>(&D_v_h)
                    : ChartJacobian());
    if (H1) *H1 = -D_v_h * h.inverse().AdjointMap();
    if (H2) *H2 = D_v_h;
    return v;
  }
}

template <typename G, typename H, typename Action>
ActionProductLieGroup<G, H, Action>
ActionProductLieGroup<G, H, Action>::compose(const This& other,
                                             ChartJacobian H1,
                                             ChartJacobian H2) const {
  checkMatchingDimensions(other, "compose");
  if constexpr (isDirectProduct) {
    return This(Base::compose(other, H1, H2));
  } else {
    const size_t productDimension =
        combinedDimension(this->firstDim(), this->secondDim());
    This result = (*this) * other;
    if (H1) *H1 = other.inverse().AdjointMap();
    if (H2) *H2 = identityJacobian(productDimension);
    return result;
  }
}

template <typename G, typename H, typename Action>
ActionProductLieGroup<G, H, Action>
ActionProductLieGroup<G, H, Action>::between(const This& other,
                                             ChartJacobian H1,
                                             ChartJacobian H2) const {
  checkMatchingDimensions(other, "between");
  if constexpr (isDirectProduct) {
    return This(Base::between(other, H1, H2));
  } else {
    const size_t productDimension =
        combinedDimension(this->firstDim(), this->secondDim());
    This result = this->inverse() * other;
    if (H1) *H1 = -result.inverse().AdjointMap();
    if (H2) *H2 = identityJacobian(productDimension);
    return result;
  }
}

template <typename G, typename H, typename Action>
ActionProductLieGroup<G, H, Action>
ActionProductLieGroup<G, H, Action>::inverse(ChartJacobian D) const {
  if constexpr (isDirectProduct) {
    return This(Base::inverse(D));
  } else {
    if (D) *D = -AdjointMap();
    return inverse();
  }
}

template <typename G, typename H, typename Action>
ActionProductLieGroup<G, H, Action>
ActionProductLieGroup<G, H, Action>::Expmap(const TangentVector& v,
                                            ChartJacobian Hv) {
  if constexpr (isDirectProduct) {
    return This(Base::Expmap(v, Hv));
  } else {
    size_t firstDimension = 0;
    size_t secondDimension = 0;
    if constexpr (firstDynamic && secondDynamic) {
      if (v.size() == 0) {
        if (Hv) *Hv = Matrix::Zero(0, 0);
        return This();
      }
      throw std::invalid_argument(
          "ActionProductLieGroup::Expmap requires split tangent vectors when "
          "both factors are dynamic");
    } else if constexpr (firstDynamic) {
      if (v.size() < dimension2) {
        throw std::invalid_argument(
            "ActionProductLieGroup::Expmap tangent dimension is too small for "
            "the fixed second factor");
      }
      firstDimension = static_cast<size_t>(v.size() - dimension2);
      secondDimension = static_cast<size_t>(dimension2);
    } else if constexpr (secondDynamic) {
      if (v.size() < dimension1) {
        throw std::invalid_argument(
            "ActionProductLieGroup::Expmap tangent dimension is too small for "
            "the fixed first factor");
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
          "ActionProductLieGroup::Expmap tangent dimension does not match "
          "product dimension");
    }

    Matrix D_g_first;
    Matrix D_h_second;
    const auto v1 = tangentSegment<G>(v, 0, firstDimension);
    const auto v2 = tangentSegment<H>(v, firstDimension, secondDimension);
    This result = Expmap(v1, v2, Hv ? DynamicJacobian(D_g_first)
                                    : DynamicJacobian(),
                         Hv ? DynamicJacobian(D_h_second)
                            : DynamicJacobian());
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
}

template <typename G, typename H, typename Action>
ActionProductLieGroup<G, H, Action>
ActionProductLieGroup<G, H, Action>::Expmap(
    const Eigen::Ref<const typename traits<G>::TangentVector>& v1,
    const Eigen::Ref<const typename traits<H>::TangentVector>& v2,
    DynamicJacobian H1, DynamicJacobian H2) {
  if constexpr (isDirectProduct) {
    return This(Base::Expmap(v1, v2, H1, H2));
  } else {
    static_assert(action_product_lie_group_detail::HasSemidirectExpmap<
                      This, Action, G, H>::value,
                  "ActionProductLieGroup semidirect actions must provide "
                  "template <class Product> static Product Expmap(v1, v2, H1, "
                  "H2).");
    return Action::template Expmap<This>(v1, v2, H1, H2);
  }
}

template <typename G, typename H, typename Action>
typename ActionProductLieGroup<G, H, Action>::TangentVector
ActionProductLieGroup<G, H, Action>::Logmap(const This& p,
                                            ChartJacobian Hp) {
  if constexpr (isDirectProduct) {
    return Base::Logmap(p, Hp);
  } else {
    static_assert(action_product_lie_group_detail::HasSemidirectLogmap<
                      This, Action, G, H>::value,
                  "ActionProductLieGroup semidirect actions must provide "
                  "template <class Product> static typename Product::"
                  "TangentVector Logmap(const Product&, ChartJacobian).");
    return Action::template Logmap<This>(p, Hp);
  }
}

template <typename G, typename H, typename Action>
ActionProductLieGroup<G, H, Action>
ActionProductLieGroup<G, H, Action>::expmap(const TangentVector& v) const {
  return compose(This::Expmap(v));
}

template <typename G, typename H, typename Action>
typename ActionProductLieGroup<G, H, Action>::Jacobian
ActionProductLieGroup<G, H, Action>::AdjointMap() const {
  if constexpr (isDirectProduct) {
    return Base::AdjointMap();
  } else {
    static_assert(action_product_lie_group_detail::HasSemidirectAdjointMap<
                      This, Action, G, H>::value,
                  "ActionProductLieGroup semidirect actions must provide "
                  "template <class Product> static typename Product::Jacobian "
                  "AdjointMap(const Product&).");
    return Action::template AdjointMap<This>(*this);
  }
}

template <typename G, typename H, typename Action>
void ActionProductLieGroup<G, H, Action>::checkMatchingDimensions(
    const This& other, const char* operation) const {
  if (this->firstDim() != other.firstDim() ||
      this->secondDim() != other.secondDim()) {
    throw std::invalid_argument(std::string("ActionProductLieGroup::") +
                                operation +
                                " requires matching component dimensions");
  }
}

template <typename G, typename H, typename Action>
void ActionProductLieGroup<G, H, Action>::print(const std::string& s) const {
  std::cout << s << "ActionProductLieGroup" << std::endl;
  traits<G>::Print(this->first, "  first");
  traits<H>::Print(this->second, "  second");
}

}  // namespace gtsam
