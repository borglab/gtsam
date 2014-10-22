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
#include <type_traits>
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
struct is_group: public std::false_type {
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
struct is_manifold: public std::false_type {
};

// dimension, can return Eigen::Dynamic (-1) if not known at compile time
template<typename T>
struct dimension;

/**
 * zero<T>::value is intended to be the origin of a canonical coordinate system
 * with canonical(t) == DefaultChart<T>(zero<T>::value).apply(t)
 * Below we provide the group identity as zero *in case* it is a group
 */
template<typename T> struct zero: public identity<T> {
  BOOST_STATIC_ASSERT(is_group<T>::value);
};

// double

template<>
struct is_group<double> : public std::true_type {
};

template<>
struct is_manifold<double> : public std::true_type {
};

template<>
struct dimension<double> : public std::integral_constant<int, 1> {
};

template<>
struct zero<double> {
  static double value() {
    return 0;
  }
};

// Fixed size Eigen::Matrix type

template<int M, int N, int Options>
struct is_group<Eigen::Matrix<double, M, N, Options> > : public std::true_type {
};

template<int M, int N, int Options>
struct is_manifold<Eigen::Matrix<double, M, N, Options> > : public std::true_type {
};

// TODO: Could be more sophisticated using Eigen traits and SFINAE?

typedef std::integral_constant<int, Eigen::Dynamic> Dynamic;

template<int Options>
struct dimension<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Options> > : public Dynamic {
};

template<int M, int Options>
struct dimension<Eigen::Matrix<double, M, Eigen::Dynamic, Options> > : public Dynamic {
};

template<int N, int Options>
struct dimension<Eigen::Matrix<double, Eigen::Dynamic, N, Options> > : public Dynamic {
};

template<int M, int N, int Options>
struct dimension<Eigen::Matrix<double, M, N, Options> > : public std::integral_constant<
    int, M * N> {
};

template<int M, int N, int Options>
struct zero<Eigen::Matrix<double, M, N, Options> > : public std::integral_constant<
    int, M * N> {
  BOOST_STATIC_ASSERT_MSG((M!=Eigen::Dynamic && N!=Eigen::Dynamic),
      "traits::zero is only supported for fixed-size matrices");
  static Eigen::Matrix<double, M, N, Options> value() {
    return Eigen::Matrix<double, M, N, Options>::Zero();
  }
};

} // \ namespace traits

// Chart is a map from T -> vector, retract is its inverse
template<typename T>
struct DefaultChart {
  BOOST_STATIC_ASSERT(traits::is_manifold<T>::value);
  typedef Eigen::Matrix<double, traits::dimension<T>::value, 1> vector;
  T t_;
  DefaultChart(const T& t) :
      t_(t) {
  }
  vector apply(const T& other) const {
    return t_.localCoordinates(other);
  }
  T retract(const vector& d) const {
    return t_.retract(d);
  }
  void print(const std::string& str = "") const {
    std::cout << str << t_ << endl;
  }
};

/**
 * Canonical<T>::value is a chart around zero<T>::value
 * An example is Canonical<Rot3>
 */
template<typename T> struct Canonical {
  typedef T type;
  typedef typename DefaultChart<T>::vector vector;
  DefaultChart<T> chart;
  Canonical() :
      chart(traits::zero<T>::value()) {
  }
  // Convert t of type T into canonical coordinates
  vector apply(const T& t) {
    return chart.apply(t);
  }
  // Convert back from canonical coordinates to T
  T retract(const vector& v) {
    return chart.retract(v);
  }
};

// double

template<>
struct DefaultChart<double> {
  typedef Eigen::Matrix<double, 1, 1> vector;
  double t_;
  DefaultChart(double t) :
      t_(t) {
  }
  vector apply(double other) {
    vector d;
    d << other - t_;
    return d;
  }
  double retract(const vector& d) {
    return t_ + d[0];
  }
  void print(const std::string& str = "") const {
    std::cout << str << t_ << endl;
  }
};

// Fixed size Eigen::Matrix type

template<int M, int N, int Options>
struct DefaultChart<Eigen::Matrix<double, M, N, Options> > {
  typedef Eigen::Matrix<double, M, N, Options> T;
  typedef Eigen::Matrix<double, traits::dimension<T>::value, 1> vector;
  BOOST_STATIC_ASSERT_MSG((M!=Eigen::Dynamic && N!=Eigen::Dynamic),
      "DefaultChart has not been implemented yet for dynamically sized matrices");
  T t_;
  DefaultChart(const T& t) :
      t_(t) {
  }
  vector apply(const T& other) const {
    T diff = other - t_;
    Eigen::Map<vector> map(diff.data());
    return vector(map);
  }
  T retract(const vector& d) const {
    Eigen::Map<const T> map(d.data());
    return t_ + map;
  }
  void print(const std::string& str = "") const {
    std::cout << str << t_ << endl;
  }
};

// Dynamically sized Vector
template<>
struct DefaultChart<Vector> {
  typedef Vector T;
  typedef T vector;
  T t_;
  DefaultChart(const T& t) :
      t_(t) {
  }
  vector apply(const T& other) const {
    return other - t_;
  }
  T retract(const vector& d) const {
    return t_ + d;
  }
  void print(const std::string& str = "") const {
    std::cout << str << t_ << endl;
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
