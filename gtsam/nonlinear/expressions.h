/**
 * @file expressions.h
 * @brief Common expressions, both linear and non-linear
 * @date Nov 23, 2014
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/base/Lie.h>

namespace gtsam {

// Generic between, assumes existence of traits<T>::Between
template<typename T>
Expression<T> between(const Expression<T>& t1, const Expression<T>& t2) {
  return Expression<T>(traits<T>::Between, t1, t2);
}

// Generic compose, assumes existence of traits<T>::Compose
template<typename T>
Expression<T> compose(const Expression<T>& t1, const Expression<T>& t2) {
  return Expression<T>(traits<T>::Compose, t1, t2);
}

/**
 * Functor that implements multiplication of a vector b with the inverse of a
 * matrix A. The derivatives are inspired by Mike Giles' "An extended collection
 * of matrix derivative results for forward and reverse mode algorithmic
 * differentiation", at https://people.maths.ox.ac.uk/gilesm/files/NA-08-01.pdf
 *
 * Usage example:
  *  Expression<Vector3> expression = MultiplyWithInverse<3>()(Key(0), Key(1));
 */
template <int N>
struct MultiplyWithInverse {
  typedef Eigen::Matrix<double, N, 1> VectorN;
  typedef Eigen::Matrix<double, N, N> MatrixN;

  /// A.inverse() * b, with optional derivatives
  VectorN operator()(const MatrixN& A, const VectorN& b,
                     OptionalJacobian<N, N* N> H1 = boost::none,
                     OptionalJacobian<N, N> H2 = boost::none) const {
    const MatrixN invA = A.inverse();
    const VectorN c = invA * b;
    // The derivative in A is just -[c[0]*invA c[1]*invA ... c[N-1]*invA]
    if (H1)
      for (size_t j = 0; j < N; j++)
        H1->template middleCols<N>(N * j) = -c[j] * invA;
    // The derivative in b is easy, as invA*b is just a linear map:
    if (H2) *H2 = invA;
    return c;
  }

  /// Create expression
  Expression<VectorN> operator()(const Expression<MatrixN>& A_,
                                 const Expression<VectorN>& b_) const {
    return Expression<VectorN>(*this, A_, b_);
  }
};

typedef Expression<double> double_;
typedef Expression<Vector3> Vector3_;

} // \namespace gtsam

