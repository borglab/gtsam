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
template <class T>
struct structure_category {}; // specializations should be derived from one of the following tags
//@}

/**
 * @name Algebraic Structure Tags
 * @brief Possible values for traits::structure_category<T>::type
 */
//@{
struct manifold_tag {};
struct group_tag {};
struct lie_group_tag : public manifold_tag, public group_tag {};
struct vector_space_tag : public lie_group_tag {};
//@}

} // namespace traits

namespace traits {

/** @name Manifold Traits */
//@{
template <class Manifold> struct TangentVector;
template <class Manifold> struct DefaultChart;
//@}

} // namespace traits

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

namespace traits {

/** @name Group Traits */
//@{
template <class Group> struct identity {};
template <class Group> struct group_flavor {};
//@}

/** @name Group Flavor Tags */
//@{
struct additive_group_tag {};
struct multiplicative_group_tag {};
//@}

} // namespace traits

template<class G>
class GroupConcept {
 public:
  typedef G Group;
  static const Group identity = traits::identity<G>::value;

  BOOST_CONCEPT_USAGE(GroupConcept) {
    BOOST_STATIC_ASSERT(boost::is_base_of<traits::group_tag,typename traits::structure_category<Group>::type>::value );
    Group ip = inverse(p);
    Group pq = compose(p, q);
    Group d = between(p, q);
    test = equal(p, q);
    test2 = operator_usage(p, q, traits::group_flavor<Group>::type);
  }

  bool check_invariants(const Group& a, const Group& b) {
    return (equal(compose(a, inverse(a)), identity))
        && (equal(between(a, b), compose(inverse(a), b)))
        && (equal(compose(a, between(a, b)), b))
        && operator_usage(a, b, traits::group_flavor<Group>::type);
  }

 private:
  Group p,q;
  bool test, test2;

  bool operator_usage(const Group& a, const Group& b, const traits::multiplicative_group_tag&) {
    return equal(compose(a, b), a*b);

  }
  bool operator_usage(const Group& a, const Group& b, const traits::additive_group_tag&) {
    return equal(compose(a, b), a+b);
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
  V p,q,r;
};
*/

} // namespace gtsam


