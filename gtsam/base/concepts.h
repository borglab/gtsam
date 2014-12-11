/*
 * concepts.h
 *
 * @data Dec 4, 2014
 * @author Mike Bosse
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/OptionalJacobian.h>
#include <boost/concept_check.hpp>
#include <boost/concept/requires.hpp>
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_base_of.hpp>

namespace gtsam {

namespace traits {

/**
 * @name Algebraic Structure Traits
 * @brief Associate a unique tag with each of the main GTSAM concepts
 */
//@{
template<typename T> struct structure_category;
//@}

/**
 * @name Algebraic Structure Tags
 * @brief Possible values for traits::structure_category<T>::type
 */
//@{
struct manifold_tag {};
struct group_tag {};
struct lie_group_tag: public manifold_tag, public group_tag {};
struct vector_space_tag: public lie_group_tag {};
//@}

}// namespace traits

namespace manifold {

/** @name Free functions any Manifold needs to define */
//@{
//@}

namespace traits {

/** @name Manifold Traits */
//@{
template<typename Manifold> struct dimension;
template<typename Manifold> struct TangentVector;
template<typename Manifold> struct DefaultChart;
template<typename Manifold> struct ChartJacobian {
  typedef OptionalJacobian<dimension<Manifold>::value, dimension<Manifold>::value> value;
};
//@}

}// \ namespace manifold::traits

/// Check invariants for Manifold type
template<typename T>
BOOST_CONCEPT_REQUIRES(((Testable<T>)),(bool)) //
check_invariants(const T& a, const T& b) {
  typedef typename traits::DefaultChart<T>::type Chart;
  // no invariants to check for manifolds, so always true if concept check compiles
  return true;
}

} // \ namespace manifold

#define GTSAM_MANIFOLD(TEMPLATE,MANIFOLD,DIM,TANGENT_VECTOR,DEFAULT_CHART) \
namespace manifold { \
namespace traits { \
template<TEMPLATE> struct dimension    < MANIFOLD > : public boost::integral_constant<int, DIM> {};\
template<TEMPLATE> struct TangentVector< MANIFOLD > { typedef TANGENT_VECTOR type;};\
template<TEMPLATE> struct DefaultChart < MANIFOLD > { typedef DEFAULT_CHART type;};\
}}

/**
 * Chart concept
 */
template<typename C>
class IsChart {
public:
  typedef typename C::ManifoldType ManifoldType;
  typedef typename manifold::traits::TangentVector<ManifoldType>::type V;
  static const int dim = manifold::traits::dimension<ManifoldType>::value;
  typedef OptionalJacobian<dim, dim> OptionalJacobian;

  BOOST_CONCEPT_USAGE(IsChart) {
    // make sure methods in Chart are defined
    v = C::Local(p,q);
    q = C::Retract(p,v);
    // and the versions with Jacobians.
    v = C::Local(p,q,Hp,Hq);
    q = C::Retract(p,v,Hp,Hv);
  }
private:
  ManifoldType p,q;
  OptionalJacobian Hp,Hq,Hv;
  V v;
};

/**
 * Manifold concept
 */
template<typename M>
class IsManifold {
public:
  typedef typename traits::structure_category<M>::type structure_category_tag;
  static const size_t dim = manifold::traits::dimension<M>::value;
  typedef typename manifold::traits::TangentVector<M>::type TangentVector;
  typedef typename manifold::traits::DefaultChart<M>::type DefaultChart;

  BOOST_CONCEPT_USAGE(IsManifold) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<traits::manifold_tag, structure_category_tag>::value),
        "This type's structure_category trait does not assert it as a manifold (or derived)");
    BOOST_STATIC_ASSERT(TangentVector::SizeAtCompileTime == dim);
    BOOST_CONCEPT_ASSERT((IsChart<DefaultChart >));
  }
};

namespace group {

/** @name Free functions any Group needs to define */
//@{
template<typename G> G compose(const G& g, const G& h);
template<typename G> G between(const G& g, const G& h);
template<typename G> G inverse(const G& g);
template<typename G, typename S> S act(const G& g, const S& s);
//@}

namespace traits {

/** @name Group Traits */
//@{
template<typename G> struct identity;
template<typename G> struct flavor;
//@}

/** @name Group Flavor Tags */
//@{
struct additive_tag {};
struct multiplicative_tag {};
//@}

}// \ namespace traits

/// Check invariants
template<typename G>
BOOST_CONCEPT_REQUIRES(((Testable<G>)),(bool)) //
check_invariants(const G& a, const G& b, double tol = 1e-9) {
  G e = traits::identity<G>::value;
  return compose(a, inverse(a)).equals(e, tol)
      && between(a, b).equals(compose(inverse(a), b), tol)
      && compose(a, between(a, b)).equals(b, tol);
}
} // \ namespace group

#define GTSAM_GROUP_IDENTITY0(GROUP) \
namespace group { namespace traits { \
template<> struct identity<GROUP > { static const GROUP value; typedef GROUP value_type;};\
const GROUP identity<GROUP >::value = GROUP::Identity();\
}}

#define GTSAM_GROUP_IDENTITY(TEMPLATE,GROUP) \
namespace group { namespace traits { \
template<TEMPLATE> struct identity<GROUP > { static const GROUP value; typedef GROUP value_type;};\
template<TEMPLATE> const GROUP identity<GROUP >::value = GROUP::Identity();\
}}

#define GTSAM_ADDITIVE_GROUP(TEMPLATE,GROUP) \
namespace group { \
template<TEMPLATE> GROUP compose(const GROUP &g, const GROUP & h) { return g + h;} \
template<TEMPLATE> GROUP between(const GROUP &g, const GROUP & h) { return h - g;} \
template<TEMPLATE> GROUP inverse(const GROUP &g) { return -g;} \
namespace traits { \
template<TEMPLATE> struct flavor<GROUP > { typedef additive_tag type;};\
}}

#define GTSAM_MULTIPLICATIVE_GROUP(TEMPLATE,GROUP) \
namespace group { \
template<TEMPLATE> GROUP compose(const GROUP &g, const GROUP & h) { return g * h;} \
template<TEMPLATE> GROUP between(const GROUP &g, const GROUP & h) { return g.inverse() * h;} \
template<TEMPLATE> GROUP inverse(const GROUP &g) { return g.inverse();} \
namespace traits { \
template<TEMPLATE> struct flavor<GROUP > { typedef multiplicative_tag type;};\
}}

/**
 * Group Concept
 */
template<typename G>
class IsGroup {
public:

  typedef typename traits::structure_category<G>::type structure_category_tag;
  typedef typename group::traits::identity<G>::value_type identity_value_type;
  typedef typename group::traits::flavor<G>::type flavor_tag;

  BOOST_CONCEPT_USAGE(IsGroup) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<traits::group_tag, structure_category_tag>::value),
        "This type's structure_category trait does not assert it as a group (or derived)");
    e = group::traits::identity<G>::value;
    e = group::compose(g, h);
    e = group::between(g, h);
    e = group::inverse(g);
    operator_usage(flavor);
    // todo: how do we test the act concept? or do we even need to?
  }

private:
  void operator_usage(group::traits::multiplicative_tag) {
    e = g * h;
    //e = -g; // todo this should work, but it is failing for Quaternions
  }
  void operator_usage(group::traits::additive_tag) {
    e = g + h;
    e = h - g;
    e = -g;
  }

  flavor_tag flavor;
  G e, g, h;
};

namespace lie_group {

/** @name Free functions any Lie Group needs to define */
//@{
template<typename LG, int dim> LG compose(const LG& g, const LG& h, OptionalJacobian<dim, dim> Hg, OptionalJacobian<dim, dim> Hh);
template<typename LG, int dim> LG between(const LG& g, const LG& h, OptionalJacobian<dim, dim> Hg, OptionalJacobian<dim, dim> Hh);
template<typename LG> LG inverse(const LG& g, OptionalJacobian<manifold::traits::dimension<LG>::value, manifold::traits::dimension<LG>::value > Hg);
template<typename LG> typename manifold::traits::TangentVector<LG>::type logmap(const LG & g);
//template<typename LG> LG expmap(const typename manifold::traits::TangentVector<LG>::type& v);
template<typename LG> LG expmap(const Eigen::Ref<const typename manifold::traits::TangentVector<LG>::type>& v);
//@}

namespace traits {

/** @name Lie Group Traits */
//@{
//@}

}// \ namespace traits

/// Check invariants
//template<typename LG>
//BOOST_CONCEPT_REQUIRES(((Testable<LG>)),(bool)) check_invariants(const LG& a,
//    const LG& b) {
//  bool check_invariants(const LG& a, const LG& b) {
//    return equal(Chart::Retract(a, b), a + b)
//        && equal(Chart::Local(a, b), b - a);
//  }
//}
}// \ namespace lie_group

/**
 * Lie Group Concept
 */
template<typename LG>
class IsLieGroup: public IsGroup<LG>, public IsManifold<LG> {
public:

  typedef typename traits::structure_category<LG>::type structure_category_tag;
  typedef OptionalJacobian<IsManifold<LG>::dim, IsManifold<LG>::dim> OptionalJacobian;
  typedef typename manifold::traits::TangentVector<LG>::type V;
  BOOST_CONCEPT_USAGE(IsLieGroup) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<traits::lie_group_tag, structure_category_tag>::value),
        "This type's trait does not assert it is a Lie group (or derived)");
    // TODO Check with Jacobian
    using lie_group::compose;
    using lie_group::between;
    using lie_group::inverse;
    g = compose(g, h, Hg, Hh);
    g = between(g, h, Hg, Hh);
    g = inverse(g, Hg);
    g = lie_group::expmap<LG>(v);
    v = lie_group::logmap<LG>(g);
  }
private:
  LG g, h;
  V v;
  OptionalJacobian Hg, Hh;
};

/**
 * A Lie Group Chart
 * Creates Local/Retract from exponential map and its inverse
 * Assumes Expmap and Logmap defined in Derived
 */
template<typename M>
struct LieGroupChart {
  typedef M ManifoldType;
  typedef typename manifold::traits::TangentVector<ManifoldType>::type TangentVector;
  static const int dim = manifold::traits::dimension<ManifoldType>::value;
  typedef OptionalJacobian<dim, dim> OptionalJacobian;

  /// retract, composes with Expmap around identity
  static ManifoldType Retract(const ManifoldType& p, const TangentVector& omega, OptionalJacobian Hp=boost::none, OptionalJacobian Hw=boost::none) {
    // todo: use the chain rule with Jacobian of Expmap
    return lie_group::compose(p, lie_group::expmap<ManifoldType>(omega), Hp, Hw);
  }

  /// local is our own, as there is a slight bug in Eigen
  static TangentVector Local(const ManifoldType& p, const ManifoldType& q, OptionalJacobian Hp=boost::none, OptionalJacobian Hq=boost::none) {
    // todo: use the chain rule with Jacobian of Logmap
    return lie_group::logmap<ManifoldType>(lie_group::between(p, q, Hp, Hq));
  }

};

template<typename V>
class IsVectorSpace: public IsLieGroup<V> {
public:

  typedef typename traits::structure_category<V>::type structure_category_tag;

  BOOST_CONCEPT_USAGE(IsVectorSpace) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<traits::vector_space_tag, structure_category_tag>::value),
        "This type's trait does not assert it as a vector space (or derived)");
    r = p + q;
    r = -p;
    r = p - q;
  }

private:
  V p, q, r;
};

} // namespace gtsam

