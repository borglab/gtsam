/*
 * concepts.h
 *
 * @data Dec 4, 2014
 * @author Mike Bosse
 * @author Frank Dellaert
 */

#pragma once

//#include "manifold.h"
//#include "chart.h"
#include <boost/concept_check.hpp>
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_base_of.hpp>

namespace gtsam {

namespace traits {

/**
 * @name Algebraic Structure Traits
 * @brief Associate a unique tag with each of the main GTSAM concepts
 */
//@{
template<class T>
struct structure_category;
// specializations should be derived from one of the following tags
//@}

/**
 * @name Algebraic Structure Tags
 * @brief Possible values for traits::structure_category<T>::type
 */
//@{
struct manifold_tag {
};
struct group_tag {
};
struct lie_group_tag: public manifold_tag, public group_tag {
};
struct vector_space_tag: public lie_group_tag {
};
//@}

}// namespace traits

namespace traits {

/** @name Manifold Traits */
//@{
template<class Manifold> struct TangentVector;
template<class Manifold> struct DefaultChart;
//@}

}// namespace traits

/*
 template<class T>
 class ManifoldConcept {
 public:
 typedef T Manifold;
 typedef typename traits::TangentVector<T>::type TangentVector;
 typedef typename traits::DefaultChart<T>::type DefaultChart;
 static const size_t dim = traits::dimension<T>::value;

 BOOST_CONCEPT_USAGE(ManifoldConcept) {
 BOOST_STATIC_ASSERT(boost::is_base_of<traits::manifold_tag, traits::structure<Manifold> >);
 BOOST_STATIC_ASSERT(TangentVector::SizeAtCompileTime == dim);
 // no direct usage for manifold since most usage is through a chart
 }
 private:
 Manifold p;
 TangentVector v;
 };

 template<class C>
 class ChartConcept {
 public:
 typedef C Chart;
 typedef typename traits::Manifold<Chart>::type Manifold;
 typedef typename traits::TangentVector<Manifold>::type TangentVector;

 BOOST_CONCEPT_USAGE(ChartConcept) {
 v = Chart::local(p,q); // returns local coordinates of q w.r.t. origin p
 q = Chart::retract(p,v); // returns retracted update of p with v
 }

 private:
 Manifold p,q;
 TangentVector v;

 };
 */

namespace group {

/** @name Free functions any Group needs to define */
//@{
template<typename G> G compose(const G&g, const G& h);
template<typename G> G between(const G&g, const G& h);
template<typename G> G inverse(const G&g);
//@}

namespace traits {

/** @name Group Traits */
//@{
template<class G> struct identity;
template<class G> struct flavor;
//@}

/** @name Group Flavor Tags */
//@{
struct additive_tag {
};
struct multiplicative_tag {
};
//@}

} // \ namespace traits
} // \ namespace group

/**
 * Group Concept
 */
template<typename G>
class Group {
public:

  typedef typename traits::structure_category<G>::type structure_category_tag;
  typedef typename group::traits::identity<G>::value_type identity_value_type;
  typedef typename group::traits::flavor<G>::type flavor_tag;

  BOOST_CONCEPT_USAGE(Group) {
    using group::compose;
    using group::between;
    using group::inverse;
    BOOST_STATIC_ASSERT(
        boost::is_base_of<traits::group_tag, structure_category_tag>::value);
    e = group::traits::identity<G>::value;
    d = compose(g, h);
    d = between(g, h);
    ig = inverse(g);
    test = operator_usage(g, h, flavor);
  }

// TODO: these all require default constructors :-(
// Also, requires equal which is not required of a group
//  Group():e(group::traits::identity<G>::value) {
//  }
//
//  bool check_invariants(const G& a, const G& b) {
//    return (equal(compose(a, inverse(a)), e))
//        && (equal(between(a, b), compose(inverse(a), b)))
//        && (equal(compose(a, between(a, b)), b))
//        && operator_usage(a, b, flavor);
//  }

private:
  flavor_tag flavor;
  G e, g, h, gh, ig, d;
  bool test, test2;

  bool operator_usage(const G& a, const G& b,
      group::traits::multiplicative_tag) {
    return group::compose(a, b) == a * b;

  }
  bool operator_usage(const G& a, const G& b, group::traits::additive_tag) {
    return group::compose(a, b) == a + b;
  }

};

/*
 template <class L>
 class LieGroupConcept : public GroupConcept<L>, public ManifoldConcept<L> {

 BOOST_CONCEPT_USAGE(LieGroupConcept) {
 BOOST_STATIC_ASSERT(boost::is_base_of<traits::lie_group_tag, traits::structure<L> >);
 }
 };

 template <class V>
 class VectorSpaceConcept : public LieGroupConcept {
 typedef typename traits::DefaultChart<V>::type Chart;
 typedef typename GroupConcept<V>::identity identity;

 BOOST_CONCEPT_USAGE(VectorSpaceConcept) {
 BOOST_STATIC_ASSERT(boost::is_base_of<traits::vector_space_tag, traits::structure<L> >);
 r = p+q;
 r = -p;
 r = p-q;
 }

 bool check_invariants(const V& a, const V& b) {
 return equal(compose(a, b), a+b)
 && equal(inverse(a), -a)
 && equal(between(a, b), b-a)
 && equal(Chart::retract(a, b), a+b)
 && equal(Chart::local(a, b), b-a);
 }

 private:
 V g,q,r;
 };
 */

} // namespace gtsam

