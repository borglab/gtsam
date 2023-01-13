/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file ProductLieGroup.h
 * @date May, 2015
 * @author Frank Dellaert
 * @brief Group product of two Lie Groups
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <utility>  // pair

namespace gtsam {

/// Template to construct the product Lie group of two other Lie groups
/// Assumes Lie group structure for G and H
template<typename G, typename H>
class ProductLieGroup: public std::pair<G, H> {
  BOOST_CONCEPT_ASSERT((IsLieGroup<G>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<H>));
  typedef std::pair<G, H> Base;

protected:
  enum {dimension1 = traits<G>::dimension};
  enum {dimension2 = traits<H>::dimension};

public:
  /// Default constructor yields identity
  ProductLieGroup():Base(traits<G>::Identity(),traits<H>::Identity()) {}

  // Construct from two subgroup elements
  ProductLieGroup(const G& g, const H& h):Base(g,h) {}

  // Construct from base
  ProductLieGroup(const Base& base):Base(base) {}

  /// @name Group
  /// @{
  typedef multiplicative_group_tag group_flavor;
  static ProductLieGroup Identity() {return ProductLieGroup();}

  ProductLieGroup operator*(const ProductLieGroup& other) const {
    return ProductLieGroup(traits<G>::Compose(this->first,other.first),
        traits<H>::Compose(this->second,other.second));
  }
  ProductLieGroup inverse() const {
    return ProductLieGroup(traits<G>::Inverse(this->first), traits<H>::Inverse(this->second));
  }
  ProductLieGroup compose(const ProductLieGroup& g) const {
    return (*this) * g;
  }
  ProductLieGroup between(const ProductLieGroup& g) const {
    return this->inverse() * g;
  }
  /// @}

  /// @name Manifold
  /// @{
  enum {dimension = dimension1 + dimension2};
  inline static size_t Dim() {return dimension;}
  inline size_t dim() const {return dimension;}

  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  ProductLieGroup retract(const TangentVector& v, //
      ChartJacobian H1 = {}, ChartJacobian H2 = {}) const {
    if (H1||H2) throw std::runtime_error("ProductLieGroup::retract derivatives not implemented yet");
    G g = traits<G>::Retract(this->first, v.template head<dimension1>());
    H h = traits<H>::Retract(this->second, v.template tail<dimension2>());
    return ProductLieGroup(g,h);
  }
  TangentVector localCoordinates(const ProductLieGroup& g, //
      ChartJacobian H1 = {}, ChartJacobian H2 = {}) const {
    if (H1||H2) throw std::runtime_error("ProductLieGroup::localCoordinates derivatives not implemented yet");
    typename traits<G>::TangentVector v1 = traits<G>::Local(this->first, g.first);
    typename traits<H>::TangentVector v2 = traits<H>::Local(this->second, g.second);
    TangentVector v;
    v << v1, v2;
    return v;
  }
  /// @}

  /// @name Lie Group
  /// @{
protected:
  typedef Eigen::Matrix<double, dimension, dimension> Jacobian;
  typedef Eigen::Matrix<double, dimension1, dimension1> Jacobian1;
  typedef Eigen::Matrix<double, dimension2, dimension2> Jacobian2;

public:
  ProductLieGroup compose(const ProductLieGroup& other, ChartJacobian H1,
      ChartJacobian H2 = {}) const {
    Jacobian1 D_g_first; Jacobian2 D_h_second;
    G g = traits<G>::Compose(this->first,other.first, H1 ? &D_g_first : 0);
    H h = traits<H>::Compose(this->second,other.second, H1 ? &D_h_second : 0);
    if (H1) {
      H1->setZero();
      H1->template topLeftCorner<dimension1,dimension1>() = D_g_first;
      H1->template bottomRightCorner<dimension2,dimension2>() = D_h_second;
    }
    if (H2) *H2 = Jacobian::Identity();
    return ProductLieGroup(g,h);
  }
  ProductLieGroup between(const ProductLieGroup& other, ChartJacobian H1,
      ChartJacobian H2 = {}) const {
    Jacobian1 D_g_first; Jacobian2 D_h_second;
    G g = traits<G>::Between(this->first,other.first, H1 ? &D_g_first : 0);
    H h = traits<H>::Between(this->second,other.second, H1 ? &D_h_second : 0);
    if (H1) {
      H1->setZero();
      H1->template topLeftCorner<dimension1,dimension1>() = D_g_first;
      H1->template bottomRightCorner<dimension2,dimension2>() = D_h_second;
    }
    if (H2) *H2 = Jacobian::Identity();
    return ProductLieGroup(g,h);
  }
  ProductLieGroup inverse(ChartJacobian D) const {
    Jacobian1 D_g_first; Jacobian2 D_h_second;
    G g = traits<G>::Inverse(this->first, D ? &D_g_first : 0);
    H h = traits<H>::Inverse(this->second, D ? &D_h_second : 0);
    if (D) {
      D->setZero();
      D->template topLeftCorner<dimension1,dimension1>() = D_g_first;
      D->template bottomRightCorner<dimension2,dimension2>() = D_h_second;
    }
    return ProductLieGroup(g,h);
  }
  static ProductLieGroup Expmap(const TangentVector& v, ChartJacobian Hv = {}) {
    Jacobian1 D_g_first; Jacobian2 D_h_second;
    G g = traits<G>::Expmap(v.template head<dimension1>(), Hv ? &D_g_first : 0);
    H h = traits<H>::Expmap(v.template tail<dimension2>(), Hv ? &D_h_second : 0);
    if (Hv) {
      Hv->setZero();
      Hv->template topLeftCorner<dimension1,dimension1>() = D_g_first;
      Hv->template bottomRightCorner<dimension2,dimension2>() = D_h_second;
    }
    return ProductLieGroup(g,h);
  }
  static TangentVector Logmap(const ProductLieGroup& p, ChartJacobian Hp = {}) {
    Jacobian1 D_g_first; Jacobian2 D_h_second;
    typename traits<G>::TangentVector v1 = traits<G>::Logmap(p.first, Hp ? &D_g_first : 0);
    typename traits<H>::TangentVector v2 = traits<H>::Logmap(p.second, Hp ? &D_h_second : 0);
    TangentVector v;
    v << v1, v2;
    if (Hp) {
      Hp->setZero();
      Hp->template topLeftCorner<dimension1,dimension1>() = D_g_first;
      Hp->template bottomRightCorner<dimension2,dimension2>() = D_h_second;
    }
    return v;
  }
  static TangentVector LocalCoordinates(const ProductLieGroup& p, ChartJacobian Hp = {}) {
    return Logmap(p, Hp);
  }
  ProductLieGroup expmap(const TangentVector& v) const {
    return compose(ProductLieGroup::Expmap(v));
  }
  TangentVector logmap(const ProductLieGroup& g) const {
    return ProductLieGroup::Logmap(between(g));
  }
  /// @}

};

// Define any direct product group to be a model of the multiplicative Group concept
template<typename G, typename H>
struct traits<ProductLieGroup<G, H> > : internal::LieGroupTraits<ProductLieGroup<G, H> > {};

} // namespace gtsam

