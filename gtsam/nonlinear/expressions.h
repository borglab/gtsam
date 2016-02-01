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
template <typename T>
Expression<T> between(const Expression<T>& t1, const Expression<T>& t2) {
  return Expression<T>(traits<T>::Between, t1, t2);
}

// Generic compose, assumes existence of traits<T>::Compose
template <typename T>
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

/**
 * Functor that implements multiplication with the inverse of a matrix, itself
 * the result of a function f. It turn out we only need the derivatives of the
 * operator phi(a): b -> f(a) * b
 */
template <typename T, int N>
struct MultiplyWithInverseFunction {
  enum { M = traits<T>::dimension };
  typedef Eigen::Matrix<double, N, 1> VectorN;
  typedef Eigen::Matrix<double, N, N> MatrixN;

  // The function phi should calculate f(a)*b, with derivatives in a and b.
  // Naturally, the derivative in b is f(a).
  typedef boost::function<VectorN(
      const T&, const VectorN&, OptionalJacobian<N, M>, OptionalJacobian<N, N>)>
      Operator;

  /// Construct with function as explained above
  MultiplyWithInverseFunction(const Operator& phi) : phi_(phi) {}

  /// f(a).inverse() * b, with optional derivatives
  VectorN operator()(const T& a, const VectorN& b,
                     OptionalJacobian<N, M> H1 = boost::none,
                     OptionalJacobian<N, N> H2 = boost::none) const {
    MatrixN A;
    phi_(a, b, boost::none, A);  // get A = f(a) by calling f once
    const MatrixN invA = A.inverse();
    const VectorN c = invA * b;

    if (H1) {
      Eigen::Matrix<double, N, M> H;
      phi_(a, c, H, boost::none);  // get derivative H of forward mapping
      *H1 = -invA* H;
    }
    if (H2) *H2 = invA;
    return c;
  }

  /// Create expression
  Expression<VectorN> operator()(const Expression<T>& a_,
                                 const Expression<VectorN>& b_) const {
    return Expression<VectorN>(*this, a_, b_);
  }

 private:
  const Operator phi_;
};

// Some typedefs
typedef Expression<double> double_;
typedef Expression<Vector1> Vector1_;
typedef Expression<Vector2> Vector2_;
typedef Expression<Vector3> Vector3_;
typedef Expression<Vector4> Vector4_;
typedef Expression<Vector5> Vector5_;
typedef Expression<Vector6> Vector6_;
typedef Expression<Vector7> Vector7_;
typedef Expression<Vector8> Vector8_;
typedef Expression<Vector9> Vector9_;

}  // \namespace gtsam
