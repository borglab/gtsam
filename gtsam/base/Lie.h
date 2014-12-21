/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Lie.h
 * @brief Base class and basic functions for Lie types
 * @author Richard Roberts
 * @author Alex Cunningham
 */


#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Group.h>

namespace gtsam {

/// tag to assert a type is a Lie group
struct lie_group_tag: public manifold_tag, public group_tag {};

namespace internal {

/// A helper that implements the traits interface for GTSAM lie groups.
/// To use this for your gtsam type, define:
/// template<> struct traits<Type> : public LieGroup<Type> { };
template<typename T>
struct LieGroup : Testable<T> {
  typedef lie_group_tag structure_category;

  /// @name Group
  /// @{
  typedef multiplicative_group_tag group_flavor;
  static T Identity() { return T::identity();}
  static T Compose(const T& m1, const T& m2) { return m1 * m2;}
  static T Between(const T& m1, const T& m2) { return m1.inverse() * m2;}
  static T Inverse(const T& m) { return m.inverse();}
  /// @}

  /// @name Manifold
  /// @{
  typedef T ManifoldType;
  enum { dimension = ManifoldType::dimension };
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  static int GetDimension(const ManifoldType& m){ return m.dim(); }

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
                             ChartJacobian Hother = boost::none) {
    return origin.localCoordinates(other, Horigin, Hother);
  }

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v,
                              ChartJacobian Horigin,
                              ChartJacobian Hv = boost::none) {
    return origin.retract(v, Horigin, Hv);
  }

  /// @}

  /// @name Lie Group
  /// @{

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

  static ManifoldType Compose(const ManifoldType& m1,
                              const ManifoldType& m2,
                              ChartJacobian H1,
                              ChartJacobian H2 = boost::none) {
    return m1.compose(m2, H1, H2);
  }

  static ManifoldType Between(const ManifoldType& m1,
                              const ManifoldType& m2,
                              ChartJacobian H1,
                              ChartJacobian H2 = boost::none) {
    return m1.between(m2, H1, H2);
  }

  static ManifoldType Inverse(const ManifoldType& m,
                              ChartJacobian H) {
    return m.inverse(H);
  }

  /// @}

};

} // \ namepsace internal

/**
 * These core global functions can be specialized by new Lie types
 * for better performance.
 */

/** Compute l0 s.t. l2=l1*l0 */
template<class T>
inline T between_default(const T& l1, const T& l2) {
  return l1.inverse().compose(l2);
}

/** Log map centered at l0, s.t. exp(l0,log(l0,lp)) = lp */
template<class T>
inline Vector logmap_default(const T& l0, const T& lp) {
  return T::Logmap(l0.between(lp));
}

/** Exponential map centered at l0, s.t. exp(t,d) = t*exp(d) */
template<class T>
inline T expmap_default(const T& t, const Vector& d) {
  return t.compose(T::Expmap(d));
}

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
    //g = traits_x<LG>::Expmap(v, Hg);
    //v = traits_x<LG>::Logmap(g, Hg);
  }
private:
  LG g, h;
  TangentVector v;
  ChartJacobian Hg, Hh;
};

/**
 *  Three term approximation of the Baker�Campbell�Hausdorff formula
 *  In non-commutative Lie groups, when composing exp(Z) = exp(X)exp(Y)
 *  it is not true that Z = X+Y. Instead, Z can be calculated using the BCH
 *  formula: Z = X + Y + [X,Y]/2 + [X-Y,[X,Y]]/12 - [Y,[X,[X,Y]]]/24
 *  http://en.wikipedia.org/wiki/Baker�Campbell�Hausdorff_formula
 */
/// AGC: bracket() only appears in Rot3 tests, should this be used elsewhere?
template<class T>
T BCH(const T& X, const T& Y) {
  static const double _2 = 1. / 2., _12 = 1. / 12., _24 = 1. / 24.;
  T X_Y = bracket(X, Y);
  return X + Y + _2 * X_Y + _12 * bracket(X - Y, X_Y) - _24 * bracket(Y,
      bracket(X, X_Y));
}

/**
 * Declaration of wedge (see Murray94book) used to convert
 * from n exponential coordinates to n*n element of the Lie algebra
 */
template <class T> Matrix wedge(const Vector& x);

/**
 * Exponential map given exponential coordinates
 * class T needs a wedge<> function and a constructor from Matrix
 * @param x exponential coordinates, vector of size n
 * @ return a T
 */
template <class T>
T expm(const Vector& x, int K=7) {
  Matrix xhat = wedge<T>(x);
  return T(expm(xhat,K));
}

} // namespace gtsam

/**
 * Macros for using the LieConcept
 *  - An instantiation for use inside unit tests
 *  - A typedef for use inside generic algorithms
 *
 * NOTE: intentionally not in the gtsam namespace to allow for classes not in
 * the gtsam namespace to be more easily enforced as testable
 */
#define GTSAM_CONCEPT_LIE_INST(T) template class gtsam::IsLieGroup<T>;
#define GTSAM_CONCEPT_LIE_TYPE(T) typedef gtsam::IsLieGroup<T> _gtsam_IsLieGroup_##T;
