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

//FIXME temporary until all conflicts with namespace traits resolved
#define traits traits_foo

namespace gtsam {

template <typename T> struct traits {};

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

// Group operator syntax flavors
struct multiplicative_group_tag {};
struct additive_group_tag {};

// a fictitious example
class Transformation;
template<>
struct traits<Transformation> {

  // Typedefs required by all manifold types.
  typedef multiplicative_group_tag group_flavor;
  typedef lie_group_tag structure_category;
  typedef Transformation ManifoldType;

  enum { dimension = 6 };
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  // Required by all Manifold types.
  static TangentVector Local(const ManifoldType& origin,
                             const ManifoldType& other);

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v);

  static int GetDimension(const ManifoldType& m){ return dimension; }

  // For Group. Only implemented for groups
  static ManifoldType Compose(const ManifoldType& m1,
                              const ManifoldType& m2);
  static ManifoldType Between(const ManifoldType& m1,
                              const ManifoldType& m2);
  static ManifoldType Inverse(const ManifoldType& m);

  static Vector3 Act(const ManifoldType& T,
                     const Vector3& p);
  static Vector3 Act(const ManifoldType& T,
                     const Vector3& p,
                     OptionalJacobian<3,3> Hp);


  // For Lie Group. Only implemented for lie groups.
  static ManifoldType Compose(const ManifoldType& m1,
                              const ManifoldType& m2,
                              ChartJacobian H1,
                              ChartJacobian H2);
  static ManifoldType Between(const ManifoldType& m1,
                              const ManifoldType& m2,
                              ChartJacobian H1,
                              ChartJacobian H2);
  static ManifoldType Inverse(const ManifoldType& m,
                              ChartJacobian H);
  static Vector3 Act(const ManifoldType& T,
                     const Vector3& p,
                     OptionalJacobian<3, dimension> HT,
                     OptionalJacobian<3, 3> Hp);
  static const ManifoldType Identity;
  static TangentVector Local(const ManifoldType& origin,
                             const ManifoldType& other,
                             ChartJacobian Horigin,
                             ChartJacobian Hother);

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v,
                              ChartJacobian Horigin,
                              ChartJacobian Hv);

  static TangentVector Logmap(const ManifoldType& m);
  static ManifoldType Expmap(const TangentVector& v);
  static TangentVector Logmap(const ManifoldType& m, ChartJacobian Hm);
  static ManifoldType Expmap(const TangentVector& v, ChartJacobian Hv);

  // For Testable
  static void Print(const ManifoldType& T);
  static void Equals(const ManifoldType& m1,
                     const ManifoldType& m2,
                     double tol = 1e-8);
};


/// Check invariants for Manifold type
template<typename T>
BOOST_CONCEPT_REQUIRES(((Testable<traits<T> >)),(bool)) //
check_manifold_invariants(const T& a, const T& b, double tol=1e-9) {
  typename traits<T>::TangentVector v0 = traits<T>::Local(a,a);
  typename traits<T>::TangentVector v = traits<T>::Local(a,b);
  T c = traits<T>::Retract(a,v);
  return v0.norm() < tol && traits<T>::Equals(b,c,tol);
}

#define GTSAM_MANIFOLD_DECLARATIONS(MANIFOLD,DIM,TANGENT_VECTOR) \
  typedef MANIFOLD ManifoldType;\
  typedef manifold_tag structure_category; \
  struct dimension : public boost::integral_constant<int, DIM> {};\
  typedef TANGENT_VECTOR TangentVector;\
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;  \
  static TangentVector Local(const ManifoldType& origin, \
                              const ManifoldType& other, \
                              ChartJacobian Horigin=boost::none, \
                              ChartJacobian Hother=boost::none); \
  static ManifoldType Retract(const ManifoldType& origin, \
                             const TangentVector& v,\
                             ChartJacobian Horigin=boost::none, \
                             ChartJacobian Hv=boost::none); \
  static int GetDimension(const ManifoldType& m) { return dimension; }

/**
 * Manifold concept
 */
template<typename M>
class IsManifold {
public:
  typedef typename traits<M>::structure_category structure_category_tag;
  static const size_t dim = traits<M>::dimension;
  typedef typename traits<M>::ManifoldType ManifoldType;
  typedef typename traits<M>::TangentVector TangentVector;
  typedef typename traits<M>::ChartJacobian ChartJacobian;

  BOOST_CONCEPT_USAGE(IsManifold) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<manifold_tag, structure_category_tag>::value),
        "This type's structure_category trait does not assert it as a manifold (or derived)");
    BOOST_STATIC_ASSERT(TangentVector::SizeAtCompileTime == dim);

    // make sure Chart methods are defined
    v = traits<M>::Local(p,q);
    q = traits<M>::Retract(p,v);
    // and the versions with Jacobians.
    v = traits<M>::Local(p,q,Hp,Hq);
    q = traits<M>::Retract(p,v,Hp,Hv);
  }
private:
  ManifoldType p,q;
  ChartJacobian Hp,Hq,Hv;
  TangentVector v;
};

/**
 * Group Concept
 */
template<typename G>
class IsGroup {
public:
  typedef typename traits<G>::structure_category structure_category_tag;
  typedef typename traits<G>::group_flavor flavor_tag;
  //typedef typename traits<G>::identity::value_type identity_value_type;

  BOOST_CONCEPT_USAGE(IsGroup) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<group_tag, structure_category_tag>::value),
        "This type's structure_category trait does not assert it as a group (or derived)");
    e = traits<G>::identity;
    e = traits<G>::Compose(g, h);
    e = traits<G>::Between(g, h);
    e = traits<G>::Inverse(g);
    operator_usage(flavor);
    // todo: how do we test the act concept? or do we even need to?
  }

private:
  void operator_usage(multiplicative_group_tag) {
    e = g * h;
    //e = -g; // todo this should work, but it is failing for Quaternions
  }
  void operator_usage(additive_group_tag) {
    e = g + h;
    e = h - g;
    e = -g;
  }

  flavor_tag flavor;
  G e, g, h;
};

/// Check invariants
template<typename G>
BOOST_CONCEPT_REQUIRES(((IsGroup<G>)),(bool)) //
check_group_invariants(const G& a, const G& b, double tol = 1e-9) {
  G e = traits<G>::identity;
  return traits<G>::Equals(traits<G>::Compose(a, traits<G>::inverse(a)), e, tol)
      && traits<G>::Equals(traits<G>::Between(a, b), traits<G>::Compose(traits<G>::Inverse(a), b), tol)
      && traits<G>::Equals(traits<G>::Compose(a, traits<G>::Between(a, b)), b, tol);
}


#define GTSAM_ADDITIVE_GROUP(GROUP) \
    typedef additive_group_tag group_flavor; \
    static GROUP Compose(const GROUP &g, const GROUP & h) { return g + h;} \
    static GROUP Between(const GROUP &g, const GROUP & h) { return h - g;} \
    static GROUP Inverse(const GROUP &g) { return -g;}


#define GTSAM_MULTIPLICATIVE_GROUP(GROUP) \
    typedef additive_group_tag group_flavor; \
    static GROUP Compose(const GROUP &g, const GROUP & h) { return g * h;} \
    static GROUP Between(const GROUP &g, const GROUP & h) { return g.inverse() * h;} \
    static GROUP Inverse(const GROUP &g) { return g.inverse();}


/**
 * Lie Group Concept
 */
template<typename LG>
class IsLieGroup: public IsGroup<LG>, public IsManifold<LG> {
public:
  typedef typename traits<LG>::structure_category structure_category_tag;
  typedef typename traits<LG>::ManifoldType ManifoldType;
  typedef typename traits<LG>::TangentVector TangentVector;
  typedef typename traits<LG>::ChartJacobian ChartJacobian;

  BOOST_CONCEPT_USAGE(IsLieGroup) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<lie_group_tag, structure_category_tag>::value),
        "This type's trait does not assert it is a Lie group (or derived)");

    // group opertations with Jacobians
    g = traits<LG>::Compose(g, h, Hg, Hh);
    g = traits<LG>::Between(g, h, Hg, Hh);
    g = traits<LG>::Inverse(g, Hg);
    // log and exp map without Jacobians
    g = traits<LG>::Expmap(v);
    v = traits<LG>::Logmap(g);
    // log and exp map with Jacobians
    g = traits<LG>::Expmap(v, Hg);
    v = traits<LG>::Logmap(g, Hg);
  }
private:
  LG g, h;
  TangentVector v;
  ChartJacobian Hg, Hh;
};


template<typename V>
class IsVectorSpace: public IsLieGroup<V> {
public:

  typedef typename traits<V>::structure_category structure_category_tag;

  BOOST_CONCEPT_USAGE(IsVectorSpace) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<vector_space_tag, structure_category_tag>::value),
        "This type's trait does not assert it as a vector space (or derived)");
    r = p + q;
    r = -p;
    r = p - q;
  }

private:
  V p, q, r;
};

} // namespace gtsam

