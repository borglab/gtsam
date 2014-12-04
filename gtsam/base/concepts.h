/*
 * concepts.h
 *
 *  Created on: Dec 4, 2014
 *      Author: mike bosse
 */

#ifndef CONCEPTS_H_
#define CONCEPTS_H_

#include "manifold.h"
#include "chart.h"

namespace gtsam {

namespace traits {

template <class Manifold>
struct TangentVector {
  //typedef XXX type;
};

// used to identify the manifold associated with a chart
template <class Chart>
struct Manifold {
  //typedef XXX type;
};

template <class Manifold>
struct DefaultChart {
  //typedef XXX type;
};

template <class Group>
struct group_flavor {};

struct additive_group_tag {};
struct multiplicative_group_tag {};

} // namespace traits

template<class T>
class ManifoldConcept {
 public:
  typedef T Manifold;
  typedef traits::TangentVector<T>::type TangentVector;
  typedef traits::DefaultChart<T>::type DefaultChart;
  static const size_t dim = traits::dimension<T>::value;

  BOOST_CONCEPT_USAGE(ManifoldConcept) {
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

template<class G>
class GroupConcept {
 public:
  typedef G Group;
  static const Group identity = traits::identity<G>::value;

  BOOST_CONCEPT_USAGE(GroupConcept) {
    Group ip = inverse(p);
    Group pq = compose(p, q);
    Group d = between(p, q);
    bool test = equal(p, q);
    bool test2 = operator_usage(p, q, traits::group_flavor<Group>);
  }

  bool check_invariants(const Group& a, const Group& b) {
    return (equal(compose(a, inverse(a)), identity))
        && (equal(between(a, b), compose(inverse(a), b)))
        && (equal(compose(a, between(a, b)), b))
        && operator_usage(a, b, traits::group_flavor<Group>);
  }

 private:
  Group p,q;

  bool operator_usage(const Group& a, const Group& b, const traits::multiplicative_group_tag&) {
    return equal(compose(a, b), a*b);

  }
  bool operator_usage(const Group& a, const Group& b, const traits::additive_group_tag&) {
    return equal(compose(a, b), a+b);
  }

};

} // namespace gtsam

#endif /* CONCEPTS_H_ */
