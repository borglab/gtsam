/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SOn.h
 * @brief   n*n matrix representation of SO(n)
 * @author  Frank Dellaert
 * @date    March 2019
 */

#pragma once

#include <gtsam/base/Manifold.h>

#include <Eigen/Core>

#include <boost/random.hpp>

#include <iostream>

namespace gtsam {
namespace internal {
// Calculate N^2 at compile time, or return Dynamic if so
constexpr int VecSize(int N) { return (N < 0) ? Eigen::Dynamic : N * N; }
}  // namespace internal

/**
 * Base class for rotation group objects. Template arguments:
 *   Derived: derived class
 *   N : dimension of ambient space, or Eigen::Dynamic, e.g. 4 for SO4
 *   D : dimension of rotation manifold, or Eigen::Dynamic, e.g. 6 for SO4
 */
template <class Derived, int N, int D>
class SOnBase {
 public:
  enum { N2 = internal::VecSize(N) };
  using VectorN2 = Eigen::Matrix<double, N2, 1>;

  /// @name Basic functionality
  /// @{

  /// Get access to derived class
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  /// @}
  /// @name Manifold
  /// @{

  /// @}
  /// @name Lie Group
  /// @{

  /// @}
  /// @name Other methods
  /// @{

  /**
   * Return vectorized rotation matrix in column order.
   * Will use dynamic matrices as intermediate results, but returns a fixed size
   * X and fixed-size Jacobian if dimension is known at compile time.
   * */
  VectorN2 vec(OptionalJacobian<N2, D> H = boost::none) const {
    const size_t n = derived().rows(), n2 = n * n;
    const size_t d = (n2 - n) / 2;  // manifold dimension

    // Calculate G matrix of vectorized generators
    Matrix G(n2, d);
    for (size_t j = 0; j < d; j++) {
      // TODO(frank): this can't be right. Think about fixed vs dynamic.
      const auto X = derived().Hat(n, Eigen::VectorXd::Unit(d, j));
      G.col(j) = Eigen::Map<const Matrix>(X.data(), n2, 1);
    }

    // Vectorize
    Vector X(n2);
    X << Eigen::Map<const Matrix>(derived().data(), n2, 1);

    // If requested, calculate H as (I \oplus Q) * P
    if (H) {
      H->resize(n2, d);
      for (size_t i = 0; i < n; i++) {
        H->block(i * n, 0, n, d) = derived() * G.block(i * n, 0, n, d);
      }
    }
    return X;
  }
  /// @}
};

/**
 *  Variable size rotations
 */
class SOn : public Eigen::MatrixXd,
            public SOnBase<SOn, Eigen::Dynamic, Eigen::Dynamic> {
 public:
  enum { N = Eigen::Dynamic };
  enum { dimension = Eigen::Dynamic };

  /// @name Constructors
  /// @{

  /// Construct SO(n) identity
  SOn(size_t n) : Eigen::MatrixXd(n, n) {
    *this << Eigen::MatrixXd::Identity(n, n);
  }

  /// Constructor from Eigen Matrix
  template <typename Derived>
  SOn(const Eigen::MatrixBase<Derived>& R) : Eigen::MatrixXd(R.eval()) {}

  /// Random SO(n) element (no big claims about uniformity)
  static SOn Random(boost::mt19937& rng, size_t n) {
    // This needs to be re-thought!
    static boost::uniform_real<double> randomAngle(-M_PI, M_PI);
    const size_t d = n * (n - 1) / 2;
    Vector xi(d);
    for (size_t j = 0; j < d; j++) {
      xi(j) = randomAngle(rng);
    }
    return SOn::Retract(n, xi);
  }

  /// @}
  /// @name Manifold
  /// @{

  /**
   * Hat operator creates Lie algebra element corresponding to d-vector, where d
   * is the dimensionality of the manifold. This function is implemented
   * recursively, and the d-vector is assumed to laid out such that the last
   * element corresponds to so(2), the last 3 to so(3), the last for to so(4)
   * etc... For example, the vector-space isomorphic to so(5) is laid out as:
   *   a b c d | u v w | x y | z
   * where the latter elements correspond to "telescoping" sub-algebras:
   *   0 -z  y -w  d
   *   z  0 -x  v -c
   *  -y  x  0 -u  b
   *   w -v  u  0 -a
   *  -d  c -b  a  0
   * This scheme behaves exactly as expected for SO(2) and SO(3).
   */
  static Matrix Hat(size_t n, const Vector& xi) {
    if (n < 2) throw std::invalid_argument("SOn::Hat: n<2 not supported");

    Matrix X(n, n);  // allocate space for n*n skew-symmetric matrix
    X.setZero();
    if (n == 2) {
      // Handle SO(2) case as recursion bottom
      assert(xi.size() == 1);
      X << 0, -xi(0), xi(0), 0;
    } else {
      // Recursively call SO(n-1) call for top-left block
      const size_t dmin = (n - 1) * (n - 2) / 2;
      X.topLeftCorner(n - 1, n - 1) = Hat(n - 1, xi.tail(dmin));

      // Now fill last row and column
      double sign = 1.0;
      for (size_t i = 0; i < n - 1; i++) {
        const size_t j = n - 2 - i;
        X(n - 1, j) = sign * xi(i);
        X(j, n - 1) = -X(n - 1, j);
        sign = -sign;
      }
    }
    return X;
  }

  /**
   * Inverse of Hat. See note about xi element order in Hat.
   */
  static Vector Vee(const Matrix& X) {
    const size_t n = X.rows();
    if (n < 2) throw std::invalid_argument("SOn::Hat: n<2 not supported");

    if (n == 2) {
      // Handle SO(2) case as recursion bottom
      Vector xi(1);
      xi(0) = X(1, 0);
      return xi;
    } else {
      // Calculate dimension and allocate space
      const size_t d = n * (n - 1) / 2;
      Vector xi(d);

      // Fill first n-1 spots from last row of X
      double sign = 1.0;
      for (size_t i = 0; i < n - 1; i++) {
        const size_t j = n - 2 - i;
        xi(i) = sign * X(n - 1, j);
        sign = -sign;
      }

      // Recursively call Vee to fill remainder of x
      const size_t dmin = (n - 1) * (n - 2) / 2;
      xi.tail(dmin) = Vee(X.topLeftCorner(n - 1, n - 1));
      return xi;
    }
  }

  /**
   * Retract uses Cayley map. See note about xi element order in Hat.
   */
  static SOn Retract(size_t n, const Vector& xi) {
    const Matrix X = Hat(n, xi / 2.0);
    const auto I = Eigen::MatrixXd::Identity(n, n);
    return (I + X) * (I - X).inverse();
  }

  /**
   * Inverse of Retract. See note about xi element order in Hat.
   */
  static Vector Local(const SOn& R) {
    const size_t n = R.rows();
    const auto I = Eigen::MatrixXd::Identity(n, n);
    const Matrix X = (I - R) * (I + R).inverse();
    return -2 * Vee(X);
  }

  /// @}
  /// @name Lie group
  /// @{

  /// @}
};

template <>
struct traits<SOn> {
  typedef manifold_tag structure_category;

  /// @name Testable
  /// @{
  static void Print(SOn R, const std::string& str = "") {
    gtsam::print(R, str);
  }
  static bool Equals(SOn R1, SOn R2, double tol = 1e-8) {
    return equal_with_abs_tol(R1, R2, tol);
  }
  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = Eigen::Dynamic };
  typedef Eigen::VectorXd TangentVector;
  //   typedef Eigen::MatrixXd Jacobian;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  //   typedef SOn ManifoldType;

  /**
   * Calculate manifold dimension, e.g.,
   *   n = 3 -> 3*2/2 = 3
   *   n = 4 -> 4*3/2 = 6
   *   n = 5 -> 5*4/2 = 10
   */
  static int GetDimension(const SOn& R) {
    const size_t n = R.rows();
    return n * (n - 1) / 2;
  }

  //   static Jacobian Eye(const SOn& R) {
  //     int dim = GetDimension(R);
  //     return Eigen::Matrix<double, dimension, dimension>::Identity(dim,
  //     dim);
  //   }

  static SOn Retract(const SOn& R, const TangentVector& xi,  //
                     ChartJacobian H1 = boost::none,
                     ChartJacobian H2 = boost::none) {
    if (H1 || H2) throw std::runtime_error("SOn::Retract");
    const size_t n = R.rows();
    return R * SOn::Retract(n, xi);
  }

  static TangentVector Local(const SOn& R, const SOn& other,  //
                             ChartJacobian H1 = boost::none,
                             ChartJacobian H2 = boost::none) {
    if (H1 || H2) throw std::runtime_error("SOn::Local");
    Matrix between = R.inverse() * other;
    return SOn::Local(between);
  }

  /// @}
};

}  // namespace gtsam
