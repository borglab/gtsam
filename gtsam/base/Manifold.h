/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Manifold.h
 * @brief Base class and basic functions for Manifold types
 * @author Alex Cunningham
 * @author Frank Dellaert
 * @author Mike Bosse
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/OptionalJacobian.h>

#include <boost/concept_check.hpp>
#include <boost/concept/requires.hpp>
#include <boost/type_traits/is_base_of.hpp>

namespace gtsam {

/// tag to assert a type is a manifold
struct manifold_tag {};

/**
 * A manifold defines a space in which there is a notion of a linear tangent space
 * that can be centered around a given point on the manifold.  These nonlinear
 * spaces may have such properties as wrapping around (as is the case with rotations),
 * which might make linear operations on parameters not return a viable element of
 * the manifold.
 *
 * We perform optimization by computing a linear delta in the tangent space of the
 * current estimate, and then apply this change using a retraction operation, which
 * maps the change in tangent space back to the manifold itself.
 *
 * There may be multiple possible retractions for a given manifold, which can be chosen
 * between depending on the computational complexity.  The important criteria for
 * the creation for the retract and localCoordinates functions is that they be
 * inverse operations. The new notion of a Chart guarantees that.
 *
 */

template <typename T> struct traits_x;

namespace internal {

/// A helper that implements the traits interface for GTSAM manifolds.
/// To use this for your gtsam type, define:
/// template<> struct traits<Type> : public Manifold<Type> { };
template<typename _ManifoldType>
struct Manifold : Testable<_ManifoldType> {
  // Typedefs required by all manifold types.
  typedef _ManifoldType ManifoldType;
  typedef manifold_tag structure_category;
  enum { dimension = ManifoldType::dimension };
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

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

} // \ namespace internal

/// Check invariants for Manifold type
template<typename T>
BOOST_CONCEPT_REQUIRES(((Testable<T>)),(bool)) //
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
    //v = traits_x<M>::Local(p,q,Hp,Hq);
    //q = traits_x<M>::Retract(p,v,Hp,Hv);
  }
private:
  ManifoldType p,q;
  ChartJacobian Hp,Hq,Hv;
  TangentVector v;
  bool b;
};

} // \ namespace gtsam

///**
// * Macros for using the ManifoldConcept
// *  - An instantiation for use inside unit tests
// *  - A typedef for use inside generic algorithms
// *
// * NOTE: intentionally not in the gtsam namespace to allow for classes not in
// * the gtsam namespace to be more easily enforced as testable
// */
#define GTSAM_CONCEPT_MANIFOLD_INST(T) template class gtsam::IsManifold<T>;
#define GTSAM_CONCEPT_MANIFOLD_TYPE(T) typedef gtsam::IsManifold<T> _gtsam_IsManifold_##T;
