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
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include <string>

namespace gtsam {

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

// Traits, same style as Boost.TypeTraits
// All meta-functions below ever only declare a single type
// or a type/value/value_type
namespace traits {

// is group, by default this is false
template<typename T>
struct is_group: public boost::false_type {
};

// identity, no default provided, by default given by default constructor
template<typename T>
struct identity {
  static T value() {
    return T();
  }
};

// is manifold, by default this is false
template<typename T>
struct is_manifold: public boost::false_type {
};

// dimension, can return Eigen::Dynamic (-1) if not known at compile time
// defaults to dynamic, TODO makes sense ?
typedef boost::integral_constant<int, Eigen::Dynamic> Dynamic;
template<typename T>
struct dimension: public Dynamic {
};

/**
 * zero<T>::value is intended to be the origin of a canonical coordinate system
 * with canonical(t) == DefaultChart<T>::local(zero<T>::value, t)
 * Below we provide the group identity as zero *in case* it is a group
 */
template<typename T> struct zero: public identity<T> {
  BOOST_STATIC_ASSERT(is_group<T>::value);
};

// double

template<>
struct is_group<double> : public boost::true_type {
};

template<>
struct is_manifold<double> : public boost::true_type {
};

template<>
struct dimension<double> : public boost::integral_constant<int, 1> {
};

template<>
struct zero<double> {
  static double value() {
    return 0;
  }
};

// Fixed size Eigen::Matrix type

template<int M, int N, int Options, int MaxRows, int MaxCols>
struct is_group<Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> > : public boost::true_type {
};

template<int M, int N, int Options, int MaxRows, int MaxCols>
struct is_manifold<Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> > : public boost::true_type {
};

template<int M, int N, int Options, int MaxRows, int MaxCols>
struct dimension<Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> > : public boost::integral_constant<
    int,
    M == Eigen::Dynamic ? Eigen::Dynamic :
        (N == Eigen::Dynamic ? Eigen::Dynamic : M * N)> {
  //TODO after switch to c++11 : the above should should be extracted to a constexpr function
  // for readability and to reduce code duplication
};

template<int M, int N, int Options, int MaxRows, int MaxCols>
struct zero<Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> > {
  BOOST_STATIC_ASSERT_MSG((M!=Eigen::Dynamic && N!=Eigen::Dynamic),
      "traits::zero is only supported for fixed-size matrices");
  static Eigen::Matrix<double, M, N, Options> value() {
    return Eigen::Matrix<double, M, N, Options>::Zero();
  }
};

template<int M, int N, int Options>
struct identity<Eigen::Matrix<double, M, N, Options> > : public zero<
    Eigen::Matrix<double, M, N, Options> > {
};

template<typename T> struct is_chart: public boost::false_type {
};

} // \ namespace traits

// Chart is a map from T -> vector, retract is its inverse
template<typename T>
struct DefaultChart {
  //BOOST_STATIC_ASSERT(traits::is_manifold<T>::value);
  typedef T type;
  typedef Eigen::Matrix<double, traits::dimension<T>::value, 1> vector;

  static vector local(const T& origin, const T& other) {
    return origin.localCoordinates(other);
  }
  static T retract(const T& origin, const vector& d) {
    return origin.retract(d);
  }
  static int getDimension(const T& origin) {
    return origin.dim();
  }
};

namespace traits {
// populate default traits
template<typename T> struct is_chart<DefaultChart<T> > : public boost::true_type {
};
template<typename T> struct dimension<DefaultChart<T> > : public dimension<T> {
};
}

template<class C>
struct ChartConcept {
public:
  typedef typename C::type type;
  typedef typename C::vector vector;

  BOOST_CONCEPT_USAGE(ChartConcept) {
    // is_chart trait should be true
    BOOST_STATIC_ASSERT((traits::is_chart<C>::value));

    /**
     * Returns Retraction update of val_
     */
    type retract_ret = C::retract(val_, vec_);

    /**
     * Returns local coordinates of another object
     */
    vec_ = C::local(val_, retract_ret);

    // a way to get the dimension that is compatible with dynamically sized types
    dim_ = C::getDimension(val_);
  }

private:
  type val_;
  vector vec_;
  int dim_;
};

/**
 * CanonicalChart<Chart<T> > is a chart around zero<T>::value
 * Canonical<T> is CanonicalChart<DefaultChart<T> >
 * An example is Canonical<Rot3>
 */
template<typename C> struct CanonicalChart {
  BOOST_CONCEPT_ASSERT((ChartConcept<C>));

  typedef C Chart;
  typedef typename Chart::type type;
  typedef typename Chart::vector vector;

  // Convert t of type T into canonical coordinates
  vector local(const type& t) {
    return Chart::local(traits::zero<type>::value(), t);
  }
  // Convert back from canonical coordinates to T
  type retract(const vector& v) {
    return Chart::retract(traits::zero<type>::value(), v);
  }
};
template<typename T> struct Canonical: public CanonicalChart<DefaultChart<T> > {
};

// double

template<>
struct DefaultChart<double> {
  typedef double type;
  typedef Eigen::Matrix<double, 1, 1> vector;

  static vector local(double origin, double other) {
    vector d;
    d << other - origin;
    return d;
  }
  static double retract(double origin, const vector& d) {
    return origin + d[0];
  }
  static int getDimension(double) {
    return 1;
  }
};

// Fixed size Eigen::Matrix type

template<int M, int N, int Options, int MaxRows, int MaxCols>
struct DefaultChart<Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> > {
  /**
   * This chart for the vector space of M x N matrices (represented by Eigen matrices) chooses as basis the one with respect to which the coordinates are exactly the matrix entries as laid out in memory (as determined by Options).
   * Computing coordinates for a matrix is then simply a reshape to the row vector of appropriate size.
   */
  typedef Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> type;
  typedef type T;
  typedef Eigen::Matrix<double, traits::dimension<T>::value, 1> vector;

  BOOST_STATIC_ASSERT_MSG((M!=Eigen::Dynamic && N!=Eigen::Dynamic),
      "Internal error: DefaultChart for Dynamic should be handled by template below");

  static vector local(const T& origin, const T& other) {
    return reshape<vector::RowsAtCompileTime, 1, vector::Options>(other)
        - reshape<vector::RowsAtCompileTime, 1, vector::Options>(origin);
  }
  static T retract(const T& origin, const vector& d) {
    return origin + reshape<M, N, Options>(d);
  }
  static int getDimension(const T&origin) {
    return origin.rows() * origin.cols();
  }
};

// Dynamically sized Vector
template<>
struct DefaultChart<Vector> {
  typedef Vector type;
  typedef Vector vector;
  static vector local(const Vector& origin, const Vector& other) {
    return other - origin;
  }
  static Vector retract(const Vector& origin, const vector& d) {
    return origin + d;
  }
  static int getDimension(const Vector& origin) {
    return origin.size();
  }
};

// Dynamically sized Matrix
template<>
struct DefaultChart<Matrix> {
  typedef Matrix type;
  typedef Vector vector;
  static vector local(const Matrix& origin, const Matrix& other) {
    Matrix d = other - origin;
    return Eigen::Map<Vector>(d.data(),getDimension(d));
  }
  static Matrix retract(const Matrix& origin, const vector& d) {
    return origin + Eigen::Map<const Matrix>(d.data(),origin.rows(),origin.cols());
  }
  static int getDimension(const Matrix& m) {
    return m.size();
  }
};

/**
 * Old Concept check class for Manifold types
 * Requires a mapping between a linear tangent space and the underlying
 * manifold, of which Lie is a specialization.
 *
 * The necessary functions to implement for Manifold are defined
 * below with additional details as to the interface.  The
 * concept checking function in class Manifold will check whether or not
 * the function exists and throw compile-time errors.
 *
 * Returns dimensionality of the tangent space, which may be smaller than the number
 * of nonlinear parameters.
 *     size_t dim() const;
 *
 * Returns a new T that is a result of updating *this with the delta v after pulling
 * the updated value back to the manifold T.
 *     T retract(const Vector& v) const;
 *
 * Returns the linear coordinates of lp in the tangent space centered around *this.
 *     Vector localCoordinates(const T& lp) const;
 *
 * By convention, we use capital letters to designate a static function
 * @tparam T is a Lie type, like Point2, Pose3, etc.
 */
template<class T>
class ManifoldConcept {
private:
  /** concept checking function - implement the functions this demands */
  static T concept_check(const T& t) {

    /** assignment */
    T t2 = t;

    /**
     * Returns dimensionality of the tangent space
     */
    size_t dim_ret = t.dim();

    /**
     * Returns Retraction update of T
     */
    T retract_ret = t.retract(gtsam::zero(dim_ret));

    /**
     * Returns local coordinates of another object
     */
    Vector localCoords_ret = t.localCoordinates(t2);

    return retract_ret;
  }
};

} // \ namespace gtsam

/**
 * Macros for using the ManifoldConcept
 *  - An instantiation for use inside unit tests
 *  - A typedef for use inside generic algorithms
 *
 * NOTE: intentionally not in the gtsam namespace to allow for classes not in
 * the gtsam namespace to be more easily enforced as testable
 */
#define GTSAM_CONCEPT_MANIFOLD_INST(T) template class gtsam::ManifoldConcept<T>;
#define GTSAM_CONCEPT_MANIFOLD_TYPE(T) typedef gtsam::ManifoldConcept<T> _gtsam_ManifoldConcept_##T;
