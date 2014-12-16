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

// This is a helper to ease the transition to the new traits defined in this file.
// Uncomment this if you want all methods that are tagged as not implemented
// to cause compilation errors.
// #define COMPILE_ERROR_NOT_IMPLEMENTED

#ifdef COMPILE_ERROR_NOT_IMPLEMENTED
#define CONCEPT_NOT_IMPLEMENTED BOOST_STATIC_ASSERT_MSG(boost::false_type, \
"This method is required by the new concepts framework but has not been implemented yet.");
#else
#define CONCEPT_NOT_IMPLEMENTED \
  throw std::runtime_error("This method is required by the new concepts framework but has not been implemented yet.");
#endif

namespace gtsam {



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

// TODO: Remove
namespace traits {
template<typename T>
struct dimension{};
}
template <typename T> struct traits_x {
  // TODO: remove anything in here ASAP.
  // This is just here during development to avoid compilation
  // errors while implmenting traits for everything.
  enum { dimension = traits::dimension<T>::value };
  typedef manifold_tag structure_category;
};


namespace internal {

/// A helper that implements the traits interface for GTSAM manifolds.
/// To use this for your gtsam type, define:
/// template<> struct traits<Type> : public Manifold<Type> { };
template<typename _ManifoldType>
struct Manifold {
  // Typedefs required by all manifold types.
  typedef _ManifoldType ManifoldType;
  typedef manifold_tag structure_category;
  enum { dimension = ManifoldType::dimension };
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  // For Testable
  static void Print(const ManifoldType& m, const std::string& str = "") {
    m.print(str);
  }
  static bool Equals(const ManifoldType& m1,
              const ManifoldType& m2,
              double tol = 1e-8) {
    return m1.equals(m2, tol);
  }

  static TangentVector Local(const ManifoldType& origin,
                             const ManifoldType& other) {
    return origin.localCoordinates(other);
  }

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v) {
    return origin.retract(v);
  }

  static TangentVector Local(const ManifoldType& origin,
                             const ManifoldType& other,
                             ChartJacobian Horigin,
                             ChartJacobian Hother) {
    return origin.localCoordinates(other, Horigin, Hother);
  }

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v,
                              ChartJacobian Horigin,
                              ChartJacobian Hv) {
    return origin.retract(v, Horigin, Hv);
  }

  static int GetDimension(const ManifoldType& m){ return m.dim(); }

};


/// A helper that implements the traits interface for GTSAM lie groups.
/// To use this for your gtsam type, define:
/// template<> struct traits<Type> : public LieGroup<Type> { };
template<typename _ManifoldType,typename _group_flavor = additive_group_tag>
struct LieGroup {
  // Typedefs required by all manifold types.
  typedef _ManifoldType ManifoldType;
  typedef lie_group_tag structure_category;
  typedef _group_flavor group_flavor;

  enum { dimension = ManifoldType::dimension };
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  // For Testable
  static void Print(const ManifoldType& m, const std::string& str = "") {
    m.print();
  }
  static bool Equals(const ManifoldType& m1,
              const ManifoldType& m2,
              double tol = 1e-8) {
    return m1.equals(m2, tol);
  }

  static TangentVector Local(const ManifoldType& origin,
                             const ManifoldType& other) {
    return origin.localCoordinates(other);
  }

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v) {
    return origin.retract(v);
  }

  static TangentVector Local(const ManifoldType& origin,
                             const ManifoldType& other,
                             ChartJacobian Horigin,
                             ChartJacobian Hother) {
    return origin.localCoordinates(other, Horigin, Hother);
  }

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v,
                              ChartJacobian Horigin,
                              ChartJacobian Hv) {
    return origin.retract(v, Horigin, Hv);
  }

  static int GetDimension(const ManifoldType& m){ return m.dim(); }

  // For Group. Only implemented for groups
  static ManifoldType Compose(const ManifoldType& m1,
                              const ManifoldType& m2) {
    return m1.compose(m2);
  }

  static ManifoldType Between(const ManifoldType& m1,
                              const ManifoldType& m2) {
    return m1.between(m2);
  }

  static ManifoldType Inverse(const ManifoldType& m) {
    return m.inverse();
  }

  static ManifoldType Compose(const ManifoldType& m1,
                              const ManifoldType& m2,
                              ChartJacobian H1,
                              ChartJacobian H2) {
    return m1.compose(m2, H1, H2);
  }

  static ManifoldType Between(const ManifoldType& m1,
                              const ManifoldType& m2,
                              ChartJacobian H1,
                              ChartJacobian H2) {
    return m1.between(m2, H1, H2);
  }

  static ManifoldType Inverse(const ManifoldType& m,
                              ChartJacobian H) {
    return m.inverse(H);
  }

  static ManifoldType Identity() {
    return ManifoldType::identity();
  }

  static TangentVector Logmap(const ManifoldType& m) {
    return ManifoldType::Logmap(m);
  }

  static ManifoldType Expmap(const TangentVector& v) {
    return ManifoldType::Expmap(v);
  }

  static TangentVector Logmap(const ManifoldType& m, ChartJacobian Hm) {
    return ManifoldType::Logmap(m, Hm);
  }

  static ManifoldType Expmap(const TangentVector& v, ChartJacobian Hv) {
    return ManifoldType::Expmap(v, Hv);
  }

};

/// A helper that implements the traits interface for GTSAM lie groups.
/// To use this for your gtsam type, define:
/// template<> struct traits<Type> : public ScalarTraits<Type> { };
template<typename Scalar>
struct ScalarTraits {
  // Typedefs required by all manifold types.
  typedef vector_space_tag structure_category;
  typedef additive_group_tag group_flavor;
  typedef Scalar ManifoldType;
  enum { dimension = 1 };
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  // For Testable
  static void Print(Scalar m, const std::string& str = "") {
    gtsam::print(m,str);
  }
  static bool Equals(Scalar m1,
      Scalar m2,
      double tol = 1e-8) {
    return fabs(m1 - m2) < tol;
  }

  static TangentVector Local(Scalar origin,
      Scalar other) {
    return TangentVector(other - origin);
  }

  static Scalar Retract(Scalar origin,
      const TangentVector& v) {
    return origin + v[0];
  }

  static TangentVector Local(Scalar origin,
      Scalar other,
      ChartJacobian Horigin,
      ChartJacobian Hother = boost::none) {
    if(Horigin) {
      (*Horigin)[0] = -1.0;
    }
    if(Hother) {
      (*Hother)[0] = 1.0;
    }
    return Local(origin, other);
  }

  static Scalar Retract(Scalar origin,
      const TangentVector& v,
      ChartJacobian Horigin,
      ChartJacobian Hv = boost::none) {
    if(Horigin) {
      (*Horigin)[0] = 1.0;
    }
    if(Hv) {
      (*Hv)[0] = 1.0;
    }
    return Retract(origin, v);
  }

  static int GetDimension(Scalar m){ return 1; }

  // For Group. Only implemented for groups
  static Scalar Compose(Scalar m1,
      Scalar m2) {
    return m1 + m2;
  }

  static Scalar Between(Scalar m1,
      Scalar m2) {
    return m2 - m1;
  }

  static Scalar Inverse(Scalar m) {
    return -m;
  }

  static Scalar Compose(Scalar m1,
      Scalar m2,
      ChartJacobian H1,
      ChartJacobian H2 = boost::none) {
    if(H1) {
      (*H1)[0] = 1.0;
    }
    if(H2) {
      (*H2)[0] = 1.0;
    }
    return Compose(m1, m2);
  }

  static Scalar Between(Scalar m1,
      Scalar m2,
      ChartJacobian H1,
      ChartJacobian H2 = boost::none) {
    if(H1) {
      (*H1)[0] = -1.0;
    }
    if(H2) {
      (*H2)[0] = 1.0;
    }
    return Between(m1, m2);
  }

  static Scalar Inverse(Scalar m, ChartJacobian H) {
    if(H) {
      (*H)[0] = -1;
    }
    return Inverse(m);
  }

  static Scalar Identity() {
    return 0;
  }

  static TangentVector Logmap(Scalar m) {
    return TangentVector(m);
  }

  static Scalar Expmap(const TangentVector& v) {
    return v[0];
  }

  static TangentVector Logmap(Scalar m, ChartJacobian Hm) {
    if(Hm) {
      (*Hm)[0] = 1.0;
    }
    return Scalar::Logmap(m);
  }

  static Scalar Expmap(const TangentVector& v, ChartJacobian Hv) {
    if(Hv) {
      (*Hv)[0] = 1.0;
    }
    return Scalar::Expmap(v);
  }

};

}  // namespace internal

template<>
struct traits_x<double> : public internal::ScalarTraits<double> {};
template<>
struct traits_x<float> : public internal::ScalarTraits<float> {};

// traits for any double Eigen matrix
template<int M, int N, int Options, int MaxRows, int MaxCols>
struct traits_x< Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> > {
  BOOST_STATIC_ASSERT_MSG(
      M != Eigen::Dynamic && N != Eigen::Dynamic,
      "These traits are only valid on fixed-size types.");

  // Typedefs required by all manifold types.
  typedef vector_space_tag structure_category;
  enum { dimension = (M == Eigen::Dynamic ? Eigen::Dynamic :
              (N == Eigen::Dynamic ? Eigen::Dynamic : M * N)) };
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  typedef Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> ManifoldType;

  static int GetDimension(const ManifoldType& m){ return m.rows()*m.cols(); }

  static Eigen::Matrix<double, dimension, dimension> Eye(const ManifoldType& m) {
    int dim = GetDimension(m);
    return Eigen::Matrix<double, dimension, dimension>::Identity(dim,dim);
  }

  // For Testable
  static void Print(const ManifoldType& m, const std::string& str = "") {
    gtsam::print(m, str);
  }
  static bool Equals(const ManifoldType& m1,
              const ManifoldType& m2,
              double tol = 1e-8) {
    return equal_with_abs_tol(m1, m2, 1e-9);
  }

  static TangentVector Local(const ManifoldType& origin,
                             const ManifoldType& other,
                             ChartJacobian Horigin = boost::none,
                             ChartJacobian Hother = boost::none) {
   if (Horigin) *Horigin = -Eye(origin);
   if (Hother) *Hother = Eye(origin);
   TangentVector result(GetDimension(origin));
   Eigen::Map<Eigen::Matrix<double, M, N> >(
    result.data(), origin.rows(), origin.cols()) = other - origin;
   return result;
  }

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v,
                              ChartJacobian Horigin = boost::none,
                              ChartJacobian Hv = boost::none) {
    if (Horigin) *Horigin = Eye(origin);
    if (Hv) *Hv = Eye(origin);
    return origin + Eigen::Map<const Eigen::Matrix<double, M, N> >(v.data(), origin.rows(), origin.cols());
  }

  static ManifoldType Compose(const ManifoldType& m1,
                              const ManifoldType& m2,
                              ChartJacobian H1 = boost::none,
                              ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Eye(m1);
    if (H2) *H2 = Eye(m1);

    return m1+m2;
  }

  static ManifoldType Between(const ManifoldType& m1,
                              const ManifoldType& m2,
                              ChartJacobian H1 = boost::none,
                              ChartJacobian H2 = boost::none) {
    if (H1) *H1 = -Eye(m1);
    if (H2) *H2 = Eye(m1);

    return m2-m1;
  }

  static ManifoldType Inverse(const ManifoldType& m,
                              ChartJacobian H = boost::none) {
    if (H) *H = -Eye(m);
    return -m;
  }

  static ManifoldType Identity() {
    //FIXME: this won't work for dynamic matrices, but where to get the size???
    return ManifoldType::Zero();
  }

  static TangentVector Logmap(const ManifoldType& m, ChartJacobian Hm = boost::none) {
    if (Hm) *Hm = Eye(m);
    TangentVector result(GetDimension(m));
    Eigen::Map<Eigen::Matrix<double, M, N> >(
      result.data()) = m;
    return result;
  }

  //FIXME: this also does not work for dynamic matrices
  static ManifoldType Expmap(const TangentVector& v, ChartJacobian Hv = boost::none) {
    ManifoldType m; m.setZero();
    if (Hv) *Hv = Eye(m);
    return m +
        Eigen::Map<Eigen::Matrix<double, M, N> >(v.data());
  }

};

template<typename ManifoldType>
struct Canonical {
  BOOST_STATIC_ASSERT_MSG(
      (boost::is_base_of<group_tag, typename traits_x<ManifoldType>::structure_category>::value),
      "This type's trait does not assert it is a manifold (or derived)");
  typedef traits_x<ManifoldType> Traits;
  typedef typename Traits::TangentVector TangentVector;
  enum { dimension = Traits::dimension };
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  static TangentVector Local(const ManifoldType& other) {
    return Traits::Local(Traits::Identity(), other);
  }

  static ManifoldType Retract(const TangentVector& v) {
    return Traits::Retract(Traits::Identity(), v);
  }

  static TangentVector Local(const ManifoldType& other,
                             ChartJacobian Hother) {
    return Traits::Local(Traits::Identity(), other, boost::none, Hother);
  }

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v,
                              ChartJacobian Horigin,
                              ChartJacobian Hv) {
    return Traits::Retract(Traits::Identity(), v, boost::none, Hv);
  }
};

/// Check invariants for Manifold type
template<typename T>
BOOST_CONCEPT_REQUIRES(((Testable<traits_x<T> >)),(bool)) //
check_manifold_invariants(const T& a, const T& b, double tol=1e-9) {
  typename traits_x<T>::TangentVector v0 = traits_x<T>::Local(a,a);
  typename traits_x<T>::TangentVector v = traits_x<T>::Local(a,b);
  T c = traits_x<T>::Retract(a,v);
  return v0.norm() < tol && traits_x<T>::Equals(b,c,tol);
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
  typedef typename traits_x<M>::structure_category structure_category_tag;
  static const size_t dim = traits_x<M>::dimension;
  typedef typename traits_x<M>::ManifoldType ManifoldType;
  typedef typename traits_x<M>::TangentVector TangentVector;
  typedef typename traits_x<M>::ChartJacobian ChartJacobian;

  BOOST_CONCEPT_USAGE(IsManifold) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<manifold_tag, structure_category_tag>::value),
        "This type's structure_category trait does not assert it as a manifold (or derived)");
    BOOST_STATIC_ASSERT(TangentVector::SizeAtCompileTime == dim);

    // make sure Chart methods are defined
    v = traits_x<M>::Local(p,q);
    q = traits_x<M>::Retract(p,v);
    // and the versions with Jacobians.
    v = traits_x<M>::Local(p,q,Hp,Hq);
    q = traits_x<M>::Retract(p,v,Hp,Hv);

    traits_x<M>::Print(p);
    traits_x<M>::Print(p, "p");
    b = traits_x<M>::Equals(p,q);
    b = traits_x<M>::Equals(p,q,1e-9);
  }
private:
  ManifoldType p,q;
  ChartJacobian Hp,Hq,Hv;
  TangentVector v;
  bool b;
};

/**
 * Group Concept
 */
template<typename G>
class IsGroup {
public:
  typedef typename traits_x<G>::structure_category structure_category_tag;
  typedef typename traits_x<G>::group_flavor flavor_tag;
  //typedef typename traits_x<G>::identity::value_type identity_value_type;

  BOOST_CONCEPT_USAGE(IsGroup) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<group_tag, structure_category_tag>::value),
        "This type's structure_category trait does not assert it as a group (or derived)");
    e = traits_x<G>::Identity();
    e = traits_x<G>::Compose(g, h);
    e = traits_x<G>::Between(g, h);
    e = traits_x<G>::Inverse(g);
    operator_usage(flavor);
    // todo: how do we test the act concept? or do we even need to?

    traits_x<G>::Print(g);
    traits_x<G>::Print(g, "g");
    b = traits_x<G>::Equals(g,h);
    b = traits_x<G>::Equals(g,h,1e-9);
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
  bool b;
};

/// Check invariants
template<typename G>
BOOST_CONCEPT_REQUIRES(((IsGroup<G>)),(bool)) //
check_group_invariants(const G& a, const G& b, double tol = 1e-9) {
  G e = traits_x<G>::Identity();
  return traits_x<G>::Equals(traits_x<G>::Compose(a, traits_x<G>::Inverse(a)), e, tol)
      && traits_x<G>::Equals(traits_x<G>::Between(a, b), traits_x<G>::Compose(traits_x<G>::Inverse(a), b), tol)
      && traits_x<G>::Equals(traits_x<G>::Compose(a, traits_x<G>::Between(a, b)), b, tol);
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
  typedef typename traits_x<LG>::structure_category structure_category_tag;
  typedef typename traits_x<LG>::ManifoldType ManifoldType;
  typedef typename traits_x<LG>::TangentVector TangentVector;
  typedef typename traits_x<LG>::ChartJacobian ChartJacobian;

  BOOST_CONCEPT_USAGE(IsLieGroup) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<lie_group_tag, structure_category_tag>::value),
        "This type's trait does not assert it is a Lie group (or derived)");

    // group opertations with Jacobians
    g = traits_x<LG>::Compose(g, h, Hg, Hh);
    g = traits_x<LG>::Between(g, h, Hg, Hh);
    g = traits_x<LG>::Inverse(g, Hg);
    // log and exp map without Jacobians
    g = traits_x<LG>::Expmap(v);
    v = traits_x<LG>::Logmap(g);
    // log and exp map with Jacobians
    g = traits_x<LG>::Expmap(v, Hg);
    v = traits_x<LG>::Logmap(g, Hg);
  }
private:
  LG g, h;
  TangentVector v;
  ChartJacobian Hg, Hh;
};


template<typename V>
class IsVectorSpace: public IsLieGroup<V> {
public:

  typedef typename traits_x<V>::structure_category structure_category_tag;

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

