/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    numericalDerivative.h
 * @brief   Some functions to compute numerical derivatives
 * @author  Frank Dellaert
 */

// \callgraph
#pragma once

#include <functional>

#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/Lie.h>

namespace gtsam {

/*
 * Note that all of these functions have two versions, a boost.function version and a
 * standard C++ function pointer version.  This allows reformulating the arguments of
 * a function to fit the correct structure, which is useful for situations like
 * member functions and functions with arguments not involved in the derivative:
 *
 * Usage of the boost bind version to rearrange arguments:
 *   for a function with one relevant param and an optional derivative:
 *     Foo bar(const Obj& a, OptionalMatrixType H1)
 *   Use boost.bind to restructure:
 *     std::bind(bar, std::placeholders::_1, boost::none)
 *   This syntax will fix the optional argument to boost::none, while using the first argument provided
 *
 * For member functions, such as below, with an instantiated copy instanceOfSomeClass
 *     Foo SomeClass::bar(const Obj& a)
 * Use boost bind as follows to create a function pointer that uses the member function:
 *       std::bind(&SomeClass::bar, ref(instanceOfSomeClass), std::placeholders::_1)
 *
 * For additional details, see the documentation:
 *     http://www.boost.org/doc/libs/release/libs/bind/bind.html
 */


// a quick helper struct to get the appropriate fixed sized matrix from two value types
namespace internal {
template<class Y, class X=double>
struct FixedSizeMatrix {
  typedef Eigen::Matrix<double,traits<Y>::dimension, traits<X>::dimension> type;
};
}

/**
 * @brief Numerically compute gradient of scalar function
 * @return n-dimensional gradient computed via central differencing
 * Class X is the input argument
 * The class X needs to have dim, expmap, logmap
  * int N is the dimension of the X input value if variable dimension type but known at test time
 */

template <class X, int N = traits<X>::dimension>
typename Eigen::Matrix<double, N, 1> numericalGradient(
    std::function<double(const X&)> h, const X& x, double delta = 1e-5) {
  double factor = 1.0 / (2.0 * delta);

  BOOST_STATIC_ASSERT_MSG(
      (boost::is_base_of<manifold_tag, typename traits<X>::structure_category>::value),
      "Template argument X must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG(N>0, "Template argument X must be fixed-size type or N must be specified.");

  // Prepare a tangent vector to perturb x with, only works for fixed size
  typename traits<X>::TangentVector d;
  d.setZero();

  Eigen::Matrix<double,N,1> g; 
  g.setZero();
  for (int j = 0; j < N; j++) {
    d(j) = delta;
    double hxplus = h(traits<X>::Retract(x, d));
    d(j) = -delta;
    double hxmin = h(traits<X>::Retract(x, d));
    d(j) = 0;
    g(j) = (hxplus - hxmin) * factor;
  }
  return g;
}

/**
 * @brief New-style numerical derivatives using manifold_traits
 * @brief Computes numerical derivative in argument 1 of unary function
 * @param h unary function yielding m-vector
 * @param x n-dimensional value at which to evaluate h
 * @param delta increment for numerical derivative
 * Class Y is the output argument
 * Class X is the input argument
 * @tparam int N is the dimension of the X input value if variable dimension type but known at test time
 * @return m*n Jacobian computed via central differencing
 */

template <class Y, class X, int N = traits<X>::dimension>
// TODO Should compute fixed-size matrix
typename internal::FixedSizeMatrix<Y, X>::type numericalDerivative11(
    std::function<Y(const X&)> h, const X& x, double delta = 1e-5) {
  typedef typename internal::FixedSizeMatrix<Y,X>::type Matrix;

  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  typedef traits<Y> TraitsY;

  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X>::structure_category>::value),
      "Template argument X must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG(N>0, "Template argument X must be fixed-size type or N must be specified.");
  typedef traits<X> TraitsX;

  // get value at x, and corresponding chart
  const Y hx = h(x);

  // Bit of a hack for now to find number of rows
  const typename TraitsY::TangentVector zeroY = TraitsY::Local(hx, hx);
  const size_t m = zeroY.size();

  // Prepare a tangent vector to perturb x with, only works for fixed size
  Eigen::Matrix<double, N, 1> dx;
  dx.setZero();

  // Fill in Jacobian H
  Matrix H = Matrix::Zero(m, N);
  const double factor = 1.0 / (2.0 * delta);
  for (int j = 0; j < N; j++) {
    dx(j) = delta;
    const auto dy1 = TraitsY::Local(hx, h(TraitsX::Retract(x, dx)));
    dx(j) = -delta;
    const auto dy2 = TraitsY::Local(hx, h(TraitsX::Retract(x, dx)));
    dx(j) = 0;
    H.col(j) << (dy1 - dy2) * factor;
  }
  return H;
}

/** use a raw C++ function pointer */
template<class Y, class X>
typename internal::FixedSizeMatrix<Y,X>::type numericalDerivative11(Y (*h)(const X&), const X& x,
    double delta = 1e-5) {
  return numericalDerivative11<Y, X>(std::bind(h, std::placeholders::_1), x,
                                     delta);
}

/**
 * Compute numerical derivative in argument 1 of binary function
 * @param h binary function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X1 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, int N = traits<X1>::dimension>
typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative21(const std::function<Y(const X1&, const X2&)>& h,
    const X1& x1, const X2& x2, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X1, N>(
      std::bind(h, std::placeholders::_1, std::cref(x2)), x1, delta);
}

/** use a raw C++ function pointer */
template<class Y, class X1, class X2>
typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative21(Y (*h)(const X1&, const X2&), const X1& x1,
    const X2& x2, double delta = 1e-5) {
  return numericalDerivative21<Y, X1, X2>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2), x1, x2,
      delta);
}

/**
 * Compute numerical derivative in argument 2 of binary function
 * @param h binary function yielding m-vector
 * @param x1 first argument value
 * @param x2 n-dimensional second argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X2 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, int N = traits<X2>::dimension>
typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative22(std::function<Y(const X1&, const X2&)> h,
    const X1& x1, const X2& x2, double delta = 1e-5) {
//  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
//       "Template argument X1 must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X2>::structure_category>::value),
       "Template argument X2 must be a manifold type.");
  return numericalDerivative11<Y, X2, N>(
      std::bind(h, std::cref(x1), std::placeholders::_1), x2, delta);
}

/** use a raw C++ function pointer */
template<class Y, class X1, class X2>
typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative22(Y (*h)(const X1&, const X2&), const X1& x1,
    const X2& x2, double delta = 1e-5) {
  return numericalDerivative22<Y, X1, X2>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2), x1, x2,
      delta);
}

/**
 * Compute numerical derivative in argument 1 of ternary function
 * @param h ternary function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * All classes Y,X1,X2,X3 need dim, expmap, logmap
 * @tparam int N is the dimension of the X1 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, int N = traits<X1>::dimension>
typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative31(
    std::function<Y(const X1&, const X2&, const X3&)> h, const X1& x1,
    const X2& x2, const X3& x3, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X1, N>(
      std::bind(h, std::placeholders::_1, std::cref(x2), std::cref(x3)),
      x1, delta);
}

template<class Y, class X1, class X2, class X3>
typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative31(Y (*h)(const X1&, const X2&, const X3&),
    const X1& x1, const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalDerivative31<Y, X1, X2, X3>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3),
      x1, x2, x3, delta);
}

/**
 * Compute numerical derivative in argument 2 of ternary function
 * @param h ternary function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * All classes Y,X1,X2,X3 need dim, expmap, logmap
 * @tparam int N is the dimension of the X2 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, int N = traits<X2>::dimension>
typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative32(
    std::function<Y(const X1&, const X2&, const X3&)> h, const X1& x1,
    const X2& x2, const X3& x3, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X2>::structure_category>::value),
      "Template argument X2 must be a manifold type.");
  return numericalDerivative11<Y, X2, N>(
      std::bind(h, std::cref(x1), std::placeholders::_1, std::cref(x3)),
      x2, delta);
}

template<class Y, class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative32(Y (*h)(const X1&, const X2&, const X3&),
    const X1& x1, const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalDerivative32<Y, X1, X2, X3>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3),
      x1, x2, x3, delta);
}

/**
 * Compute numerical derivative in argument 3 of ternary function
 * @param h ternary function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * All classes Y,X1,X2,X3 need dim, expmap, logmap
 * @tparam int N is the dimension of the X3 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, int N = traits<X3>::dimension>
typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative33(
    std::function<Y(const X1&, const X2&, const X3&)> h, const X1& x1,
    const X2& x2, const X3& x3, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X3>::structure_category>::value),
      "Template argument X3 must be a manifold type.");
  return numericalDerivative11<Y, X3, N>(
      std::bind(h, std::cref(x1), std::cref(x2), std::placeholders::_1),
      x3, delta);
}

template<class Y, class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative33(Y (*h)(const X1&, const X2&, const X3&),
    const X1& x1, const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalDerivative33<Y, X1, X2, X3>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3),
      x1, x2, x3, delta);
}

/**
 * Compute numerical derivative in argument 1 of 4-argument function
 * @param h quartic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X1 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, int N = traits<X1>::dimension>
typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative41(
    std::function<Y(const X1&, const X2&, const X3&, const X4&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X1, N>(
      std::bind(h, std::placeholders::_1, std::cref(x2), std::cref(x3),
                  std::cref(x4)),
      x1, delta);
}

template<class Y, class X1, class X2, class X3, class X4>
inline typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative41(Y (*h)(const X1&, const X2&, const X3&, const X4&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, double delta = 1e-5) {
  return numericalDerivative41<Y, X1, X2, X3, X4>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4),
      x1, x2, x3, x4);
}

/**
 * Compute numerical derivative in argument 2 of 4-argument function
 * @param h quartic function yielding m-vector
 * @param x1 first argument value
 * @param x2 n-dimensional second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X2 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, int N = traits<X2>::dimension>
typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative42(
    std::function<Y(const X1&, const X2&, const X3&, const X4&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X2>::structure_category>::value),
      "Template argument X2 must be a manifold type.");
  return numericalDerivative11<Y, X2, N>(
      std::bind(h, std::cref(x1), std::placeholders::_1, std::cref(x3),
                  std::cref(x4)),
      x2, delta);
}

template<class Y, class X1, class X2, class X3, class X4>
inline typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative42(Y (*h)(const X1&, const X2&, const X3&, const X4&),
  const X1& x1, const X2& x2, const X3& x3, const X4& x4, double delta = 1e-5) {
  return numericalDerivative42<Y, X1, X2, X3, X4>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4),
      x1, x2, x3, x4);
}

/**
 * Compute numerical derivative in argument 3 of 4-argument function
 * @param h quartic function yielding m-vector
 * @param x1 first argument value
 * @param x2 second argument value
 * @param x3 n-dimensional third argument value
 * @param x4 fourth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X3 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, int N = traits<X3>::dimension>
typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative43(
    std::function<Y(const X1&, const X2&, const X3&, const X4&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X3>::structure_category>::value),
      "Template argument X3 must be a manifold type.");
  return numericalDerivative11<Y, X3, N>(
      std::bind(h, std::cref(x1), std::cref(x2), std::placeholders::_1,
                  std::cref(x4)),
      x3, delta);
}

template<class Y, class X1, class X2, class X3, class X4>
inline typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative43(Y (*h)(const X1&, const X2&, const X3&, const X4&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, double delta = 1e-5) {
  return numericalDerivative43<Y, X1, X2, X3, X4>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4),
      x1, x2, x3, x4);
}

/**
 * Compute numerical derivative in argument 4 of 4-argument function
 * @param h quartic function yielding m-vector
 * @param x1 first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 n-dimensional fourth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X4 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, int N = traits<X4>::dimension>
typename internal::FixedSizeMatrix<Y,X4>::type numericalDerivative44(
    std::function<Y(const X1&, const X2&, const X3&, const X4&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X4>::structure_category>::value),
      "Template argument X4 must be a manifold type.");
  return numericalDerivative11<Y, X4, N>(
      std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3),
                  std::placeholders::_1),
      x4, delta);
}

template<class Y, class X1, class X2, class X3, class X4>
inline typename internal::FixedSizeMatrix<Y,X4>::type numericalDerivative44(Y (*h)(const X1&, const X2&, const X3&, const X4&),
  const X1& x1, const X2& x2, const X3& x3, const X4& x4, double delta = 1e-5) {
  return numericalDerivative44<Y, X1, X2, X3, X4>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4),
      x1, x2, x3, x4);
}

/**
 * Compute numerical derivative in argument 1 of 5-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X1 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, int N = traits<X1>::dimension>
typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative51(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, const X5& x5, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X1, N>(
      std::bind(h, std::placeholders::_1, std::cref(x2), std::cref(x3),
                  std::cref(x4), std::cref(x5)),
      x1, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5>
inline typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative51(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, double delta = 1e-5) {
  return numericalDerivative51<Y, X1, X2, X3, X4, X5>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5),
      x1, x2, x3, x4, x5);
}

/**
 * Compute numerical derivative in argument 2 of 5-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X2 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, int N = traits<X2>::dimension>
typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative52(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, const X5& x5, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X2, N>(
      std::bind(h, std::cref(x1), std::placeholders::_1, std::cref(x3),
                  std::cref(x4), std::cref(x5)),
      x2, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5>
inline typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative52(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, double delta = 1e-5) {
  return numericalDerivative52<Y, X1, X2, X3, X4, X5>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5),
      x1, x2, x3, x4, x5);
}

/**
 * Compute numerical derivative in argument 3 of 5-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X3 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, int N = traits<X3>::dimension>
typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative53(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, const X5& x5, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X3, N>(
      std::bind(h, std::cref(x1), std::cref(x2), std::placeholders::_1,
                  std::cref(x4), std::cref(x5)),
      x3, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5>
inline typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative53(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, double delta = 1e-5) {
  return numericalDerivative53<Y, X1, X2, X3, X4, X5>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5),
      x1, x2, x3, x4, x5);
}

/**
 * Compute numerical derivative in argument 4 of 5-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X4 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, int N = traits<X4>::dimension>
typename internal::FixedSizeMatrix<Y,X4>::type numericalDerivative54(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, const X5& x5, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X4, N>(
      std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3),
                  std::placeholders::_1, std::cref(x5)),
      x4, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5>
inline typename internal::FixedSizeMatrix<Y,X4>::type numericalDerivative54(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, double delta = 1e-5) {
  return numericalDerivative54<Y, X1, X2, X3, X4, X5>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5),
      x1, x2, x3, x4, x5);
}

/**
 * Compute numerical derivative in argument 5 of 5-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X5 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, int N = traits<X5>::dimension>
typename internal::FixedSizeMatrix<Y,X5>::type numericalDerivative55(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, const X5& x5, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X5, N>(
      std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3),
                  std::cref(x4), std::placeholders::_1),
      x5, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5>
inline typename internal::FixedSizeMatrix<Y,X5>::type numericalDerivative55(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, double delta = 1e-5) {
  return numericalDerivative55<Y, X1, X2, X3, X4, X5>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5),
      x1, x2, x3, x4, x5);
}

/**
 * Compute numerical derivative in argument 1 of 6-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param x6 sixth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X1 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, int N = traits<X1>::dimension>
typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative61(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X1, N>(
      std::bind(h, std::placeholders::_1, std::cref(x2), std::cref(x3),
                  std::cref(x4), std::cref(x5), std::cref(x6)),
      x1, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5, class X6>
inline typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative61(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  return numericalDerivative61<Y, X1, X2, X3, X4, X5, X6>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5, std::placeholders::_6),
      x1, x2, x3, x4, x5, x6);
}

/**
 * Compute numerical derivative in argument 2 of 6-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param x6 sixth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X2 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, int N = traits<X2>::dimension>
typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative62(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X2, N>(
      std::bind(h, std::cref(x1), std::placeholders::_1, std::cref(x3),
                  std::cref(x4), std::cref(x5), std::cref(x6)),
      x2, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5, class X6>
inline typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative62(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  return numericalDerivative62<Y, X1, X2, X3, X4, X5, X6>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5, std::placeholders::_6),
      x1, x2, x3, x4, x5, x6);
}

/**
 * Compute numerical derivative in argument 3 of 6-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param x6 sixth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X3 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, int N = traits<X3>::dimension>
typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative63(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X3, N>(
      std::bind(h, std::cref(x1), std::cref(x2), std::placeholders::_1,
                  std::cref(x4), std::cref(x5), std::cref(x6)),
      x3, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5, class X6>
inline typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative63(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  return numericalDerivative63<Y, X1, X2, X3, X4, X5, X6>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5, std::placeholders::_6),
      x1, x2, x3, x4, x5, x6);
}

/**
 * Compute numerical derivative in argument 4 of 6-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param x6 sixth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X4 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, int N = traits<X4>::dimension>
typename internal::FixedSizeMatrix<Y,X4>::type numericalDerivative64(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X4, N>(
      std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3),
                  std::placeholders::_1, std::cref(x5), std::cref(x6)),
      x4, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5, class X6>
inline typename internal::FixedSizeMatrix<Y,X4>::type numericalDerivative64(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  return numericalDerivative64<Y, X1, X2, X3, X4, X5>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5, std::placeholders::_6),
      x1, x2, x3, x4, x5, x6);
}

/**
 * Compute numerical derivative in argument 5 of 6-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param x6 sixth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X5 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, int N = traits<X5>::dimension>
typename internal::FixedSizeMatrix<Y,X5>::type numericalDerivative65(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&)> h, const X1& x1,
    const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X5, N>(
      std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3),
                  std::cref(x4), std::placeholders::_1, std::cref(x6)),
      x5, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5, class X6>
inline typename internal::FixedSizeMatrix<Y,X5>::type numericalDerivative65(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  return numericalDerivative65<Y, X1, X2, X3, X4, X5, X6>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5, std::placeholders::_6),
      x1, x2, x3, x4, x5, x6);
}

/**
 * Compute numerical derivative in argument 6 of 6-argument function
 * @param h quintic function yielding m-vector
 * @param x1 n-dimensional first argument value
 * @param x2 second argument value
 * @param x3 third argument value
 * @param x4 fourth argument value
 * @param x5 fifth argument value
 * @param x6 sixth argument value
 * @param delta increment for numerical derivative
 * @return m*n Jacobian computed via central differencing
 * @tparam int N is the dimension of the X6 input value if variable dimension type but known at test time
 */
template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, int N = traits<X6>::dimension>
typename internal::FixedSizeMatrix<Y, X6>::type numericalDerivative66(
    std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&)> h,
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6,
    double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X1>::structure_category>::value),
      "Template argument X1 must be a manifold type.");
  return numericalDerivative11<Y, X6, N>(
      std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3),
                  std::cref(x4), std::cref(x5), std::placeholders::_1),
      x6, delta);
}

template<class Y, class X1, class X2, class X3, class X4, class X5, class X6>
inline typename internal::FixedSizeMatrix<Y,X6>::type numericalDerivative66(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&),
    const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, double delta = 1e-5) {
  return numericalDerivative66<Y, X1, X2, X3, X4, X5, X6>(
      std::bind(h, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5, std::placeholders::_6),
      x1, x2, x3, x4, x5, x6);
}

/**
 * Compute numerical Hessian matrix.  Requires a single-argument Lie->scalar
 * function.  This is implemented simply as the derivative of the gradient.
 * @param f A function taking a Lie object as input and returning a scalar
 * @param x The center point for computing the Hessian
 * @param delta The numerical derivative step size
 * @return n*n Hessian matrix computed via central differencing
 */
template<class X>
inline typename internal::FixedSizeMatrix<X,X>::type numericalHessian(std::function<double(const X&)> f, const X& x,
    double delta = 1e-5) {
  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename traits<X>::structure_category>::value),
      "Template argument X must be a manifold type.");
  typedef Eigen::Matrix<double, traits<X>::dimension, 1> VectorD;
  typedef std::function<double(const X&)> F;
  typedef std::function<VectorD(F, const X&, double)> G;
  G ng = static_cast<G>(numericalGradient<X> );
  return numericalDerivative11<VectorD, X>(
      std::bind(ng, f, std::placeholders::_1, delta), x, delta);
}

template<class X>
inline typename internal::FixedSizeMatrix<X,X>::type numericalHessian(double (*f)(const X&), const X& x, double delta =
    1e-5) {
  return numericalHessian(std::function<double(const X&)>(f), x, delta);
}

/** Helper class that computes the derivative of f w.r.t. x1, centered about
 * x1_, as a function of x2
 */
template<class X1, class X2>
class G_x1 {
  const std::function<double(const X1&, const X2&)>& f_;
  X1 x1_;
  double delta_;
public:
  typedef typename internal::FixedSizeMatrix<X1>::type Vector;

  G_x1(const std::function<double(const X1&, const X2&)>& f, const X1& x1,
      double delta) :
      f_(f), x1_(x1), delta_(delta) {
  }
  Vector operator()(const X2& x2) {
    return numericalGradient<X1>(
        std::bind(f_, std::placeholders::_1, std::cref(x2)), x1_, delta_);
  }
};

template<class X1, class X2>
inline typename internal::FixedSizeMatrix<X1,X2>::type numericalHessian212(
    std::function<double(const X1&, const X2&)> f, const X1& x1, const X2& x2,
    double delta = 1e-5) {
  typedef typename internal::FixedSizeMatrix<X1>::type Vector;
  G_x1<X1, X2> g_x1(f, x1, delta);
  return numericalDerivative11<Vector, X2>(
      std::function<Vector(const X2&)>(
          std::bind<Vector>(std::ref(g_x1), std::placeholders::_1)),
      x2, delta);
}

template<class X1, class X2>
inline typename internal::FixedSizeMatrix<X1,X2>::type numericalHessian212(double (*f)(const X1&, const X2&),
    const X1& x1, const X2& x2, double delta = 1e-5) {
  return numericalHessian212(std::function<double(const X1&, const X2&)>(f),
      x1, x2, delta);
}

template<class X1, class X2>
inline typename internal::FixedSizeMatrix<X1,X1>::type numericalHessian211(
    std::function<double(const X1&, const X2&)> f, const X1& x1, const X2& x2,
    double delta = 1e-5) {

  typedef typename internal::FixedSizeMatrix<X1>::type Vector;

  Vector (*numGrad)(std::function<double(const X1&)>, const X1&,
      double) = &numericalGradient<X1>;
  std::function<double(const X1&)> f2(
      std::bind(f, std::placeholders::_1, std::cref(x2)));

  return numericalDerivative11<Vector, X1>(
      std::function<Vector(const X1&)>(
          std::bind(numGrad, f2, std::placeholders::_1, delta)),
      x1, delta);
}

template<class X1, class X2>
inline typename internal::FixedSizeMatrix<X1,X1>::type numericalHessian211(double (*f)(const X1&, const X2&),
    const X1& x1, const X2& x2, double delta = 1e-5) {
  return numericalHessian211(std::function<double(const X1&, const X2&)>(f),
      x1, x2, delta);
}

template<class X1, class X2>
inline typename internal::FixedSizeMatrix<X2,X2>::type numericalHessian222(
    std::function<double(const X1&, const X2&)> f, const X1& x1, const X2& x2,
    double delta = 1e-5) {
  typedef typename internal::FixedSizeMatrix<X2>::type Vector;
  Vector (*numGrad)(std::function<double(const X2&)>, const X2&,
      double) = &numericalGradient<X2>;
  std::function<double(const X2&)> f2(
      std::bind(f, std::cref(x1), std::placeholders::_1));

  return numericalDerivative11<Vector, X2>(
      std::function<Vector(const X2&)>(
          std::bind(numGrad, f2, std::placeholders::_1, delta)),
      x2, delta);
}

template<class X1, class X2>
inline typename internal::FixedSizeMatrix<X2,X2>::type numericalHessian222(double (*f)(const X1&, const X2&),
    const X1& x1, const X2& x2, double delta = 1e-5) {
  return numericalHessian222(std::function<double(const X1&, const X2&)>(f),
      x1, x2, delta);
}

/**
 * Numerical Hessian for tenary functions
 */
/* **************************************************************** */
template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X1,X1>::type numericalHessian311(
    std::function<double(const X1&, const X2&, const X3&)> f, const X1& x1,
    const X2& x2, const X3& x3, double delta = 1e-5) {
  typedef typename internal::FixedSizeMatrix<X1>::type Vector;
  Vector (*numGrad)(std::function<double(const X1&)>, const X1&,
      double) = &numericalGradient<X1>;
  std::function<double(const X1&)> f2(std::bind(
      f, std::placeholders::_1, std::cref(x2), std::cref(x3)));

  return numericalDerivative11<Vector, X1>(
      std::function<Vector(const X1&)>(
          std::bind(numGrad, f2, std::placeholders::_1, delta)),
      x1, delta);
}

template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X1,X1>::type numericalHessian311(double (*f)(const X1&, const X2&, const X3&),
    const X1& x1, const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalHessian311(
      std::function<double(const X1&, const X2&, const X3&)>(f), x1, x2, x3,
      delta);
}

/* **************************************************************** */
template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X2,X2>::type numericalHessian322(
    std::function<double(const X1&, const X2&, const X3&)> f, const X1& x1,
    const X2& x2, const X3& x3, double delta = 1e-5) {
  typedef typename internal::FixedSizeMatrix<X2>::type Vector;
  Vector (*numGrad)(std::function<double(const X2&)>, const X2&,
      double) = &numericalGradient<X2>;
  std::function<double(const X2&)> f2(std::bind(
      f, std::cref(x1), std::placeholders::_1, std::cref(x3)));

  return numericalDerivative11<Vector, X2>(
      std::function<Vector(const X2&)>(
          std::bind(numGrad, f2, std::placeholders::_1, delta)),
      x2, delta);
}

template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X2,X2>::type numericalHessian322(double (*f)(const X1&, const X2&, const X3&),
    const X1& x1, const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalHessian322(
      std::function<double(const X1&, const X2&, const X3&)>(f), x1, x2, x3,
      delta);
}

/* **************************************************************** */
template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X3,X3>::type numericalHessian333(
    std::function<double(const X1&, const X2&, const X3&)> f, const X1& x1,
    const X2& x2, const X3& x3, double delta = 1e-5) {
  typedef typename internal::FixedSizeMatrix<X3>::type Vector;
  Vector (*numGrad)(std::function<double(const X3&)>, const X3&,
      double) = &numericalGradient<X3>;
  std::function<double(const X3&)> f2(std::bind(
      f, std::cref(x1), std::cref(x2), std::placeholders::_1));

  return numericalDerivative11<Vector, X3>(
      std::function<Vector(const X3&)>(
          std::bind(numGrad, f2, std::placeholders::_1, delta)),
      x3, delta);
}

template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X3,X3>::type numericalHessian333(double (*f)(const X1&, const X2&, const X3&),
    const X1& x1, const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalHessian333(
      std::function<double(const X1&, const X2&, const X3&)>(f), x1, x2, x3,
      delta);
}

/* **************************************************************** */
template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X1,X2>::type numericalHessian312(
    std::function<double(const X1&, const X2&, const X3&)> f, const X1& x1,
    const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalHessian212<X1, X2>(
      std::function<double(const X1&, const X2&)>(
          std::bind(f, std::placeholders::_1, std::placeholders::_2,
                      std::cref(x3))),
      x1, x2, delta);
}

template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X1,X3>::type numericalHessian313(
    std::function<double(const X1&, const X2&, const X3&)> f, const X1& x1,
    const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalHessian212<X1, X3>(
      std::function<double(const X1&, const X3&)>(
          std::bind(f, std::placeholders::_1, std::cref(x2),
                      std::placeholders::_2)),
      x1, x3, delta);
}

template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X2,X3>::type numericalHessian323(
    std::function<double(const X1&, const X2&, const X3&)> f, const X1& x1,
    const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalHessian212<X2, X3>(
      std::function<double(const X2&, const X3&)>(
          std::bind(f, std::cref(x1), std::placeholders::_1,
                      std::placeholders::_2)),
      x2, x3, delta);
}

/* **************************************************************** */
template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X1,X2>::type numericalHessian312(double (*f)(const X1&, const X2&, const X3&),
    const X1& x1, const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalHessian312(
      std::function<double(const X1&, const X2&, const X3&)>(f), x1, x2, x3,
      delta);
}

template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X1,X3>::type numericalHessian313(double (*f)(const X1&, const X2&, const X3&),
    const X1& x1, const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalHessian313(
      std::function<double(const X1&, const X2&, const X3&)>(f), x1, x2, x3,
      delta);
}

template<class X1, class X2, class X3>
inline typename internal::FixedSizeMatrix<X2,X3>::type numericalHessian323(double (*f)(const X1&, const X2&, const X3&),
    const X1& x1, const X2& x2, const X3& x3, double delta = 1e-5) {
  return numericalHessian323(
      std::function<double(const X1&, const X2&, const X3&)>(f), x1, x2, x3,
      delta);
}

} // namespace gtsam

