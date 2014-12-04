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
  typedef Eigen::VectorXd type;
};

template <class Chart>
struct Manifold {
  typedef Chart::value_type type;
};

struct additive_group_tag {};
struct multiplicative_group_tag {};

} // namespace traits

template<class T>
class ManifoldConcept {
 public:
  typedef T Manifold;
  typedef traits::TangentVector<T>::type TangentVector;
  typedef traits::DefaultChart<T> DefaultChart;
  static const size_t dim = traits::dimension<T>::value;

  BOOST_CONCEPT_USAGE(ManifoldConcept) {

    // assignable
    T t2 = t;

    TangentVector v;
    BOOST_STATIC_ASSERT(TangentVector::SizeAtCompileTime == dim);

  }
 private:
  Manifold p;
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
    operator_usage(p, q, traits::group_flavor<Group>::tag);
  }

  bool check_invariants(const Group& a, const Group& b) {
    return (equal(compose(a, inverse(a)), identity))
        && (equal(between(a, b), compose(inverse(a), b)))
        && (equal(compose(a, between(a, b)), b))
        && operator_usage(a, b, traits::group_flavor<Group>::tag)
  }

 private:
  Group p,q;

  bool operator_usage(const Group& a, const Group& b, traits::multiplicative_group_tag) {
    return equals(compose(a, b), a*b);

  }
  bool operator_usage(const Group& a, const Group& b, traits::additive_group_tag) {
    return equals(compose(a, b), a+b);
  }

};

} // namespace gtsam

#endif /* CONCEPTS_H_ */
