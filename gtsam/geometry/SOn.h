/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SOn.h
 * @brief   N*N matrix representation of SO(N). N can be Eigen::Dynamic
 * @author  Frank Dellaert
 * @date    March 2019
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/make_shared.h>
#include <gtsam/dllexport.h>
#include <Eigen/Core>

#include <boost/serialization/nvp.hpp>

#include <iostream> // TODO(frank): how to avoid?
#include <string>
#include <type_traits>
#include <vector>
#include <random>

namespace gtsam {

namespace internal {
/// Calculate dimensionality of SO<N> manifold, or return Dynamic if so
constexpr int DimensionSO(int N) {
  return (N < 0) ? Eigen::Dynamic : N * (N - 1) / 2;
}

// Calculate N^2 at compile time, or return Dynamic if so
constexpr int NSquaredSO(int N) { return (N < 0) ? Eigen::Dynamic : N * N; }
}  // namespace internal

/**
 * Manifold of special orthogonal rotation matrices SO<N>.
 * Template paramater N can be a fixed integer or can be Eigen::Dynamic
 */
template <int N>
class SO : public LieGroup<SO<N>, internal::DimensionSO(N)> {
 public:
  enum { dimension = internal::DimensionSO(N) };
  using MatrixNN = Eigen::Matrix<double, N, N>;
  using VectorN2 = Eigen::Matrix<double, internal::NSquaredSO(N), 1>;
  using MatrixDD = Eigen::Matrix<double, dimension, dimension>;

  GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(true)

 protected:
  MatrixNN matrix_;  ///< Rotation matrix

  // enable_if_t aliases, used to specialize constructors/methods, see
  // https://www.fluentcpp.com/2018/05/18/make-sfinae-pretty-2-hidden-beauty-sfinae/
  template <int N_>
  using IsDynamic = typename std::enable_if<N_ == Eigen::Dynamic, void>::type;
  template <int N_>
  using IsFixed = typename std::enable_if<N_ >= 2, void>::type;
  template <int N_>
  using IsSO3 = typename std::enable_if<N_ == 3, void>::type;

 public:
  /// @name Constructors
  /// @{

  /// Construct SO<N> identity for N >= 2
  template <int N_ = N, typename = IsFixed<N_>>
  SO() : matrix_(MatrixNN::Identity()) {}

  /// Construct SO<N> identity for N == Eigen::Dynamic
  template <int N_ = N, typename = IsDynamic<N_>>
  explicit SO(size_t n = 0) {
    // We allow for n=0 as the default constructor, needed for serialization,
    // wrappers etc.
    matrix_ = Eigen::MatrixXd::Identity(n, n);
  }

  /// Constructor from Eigen Matrix, dynamic version
  template <typename Derived>
  explicit SO(const Eigen::MatrixBase<Derived>& R) : matrix_(R.eval()) {}

  /// Named constructor from Eigen Matrix
  template <typename Derived>
  static SO FromMatrix(const Eigen::MatrixBase<Derived>& R) {
    return SO(R);
  }

  /// Named constructor from lower dimensional matrix
  template <typename Derived, int N_ = N, typename = IsDynamic<N_>>
  static SO Lift(size_t n, const Eigen::MatrixBase<Derived> &R) {
    Matrix Q = Matrix::Identity(n, n);
    const int p = R.rows();
    assert(p >= 0 && p <= static_cast<int>(n) && R.cols() == p);
    Q.topLeftCorner(p, p) = R;
    return SO(Q);
  }

  /// Construct dynamic SO(n) from Fixed SO<M>
  template <int M, int N_ = N, typename = IsDynamic<N_>>
  explicit SO(const SO<M>& R) : matrix_(R.matrix()) {}

  /// Constructor from AngleAxisd
  template <int N_ = N, typename = IsSO3<N_>>
  explicit SO(const Eigen::AngleAxisd& angleAxis) : matrix_(angleAxis) {}

  /// Constructor from axis and angle. Only defined for SO3
  static SO AxisAngle(const Vector3& axis, double theta);

  /// Named constructor that finds SO(n) matrix closest to M in Frobenius norm,
  /// currently only defined for SO3.
  static SO ClosestTo(const MatrixNN& M);

  /// Named constructor that finds chordal mean
  /// \f$ mu = argmin_R \sum sqr(|R-R_i|_F) \f$,
  /// currently only defined for SO3.
  static SO ChordalMean(const std::vector<SO>& rotations);

  /// Random SO(n) element (no big claims about uniformity). SO(3) is specialized in SO3.cpp
  template <int N_ = N, typename = IsDynamic<N_>>
  static SO Random(std::mt19937& rng, size_t n = 0) {
    if (n == 0) throw std::runtime_error("SO: Dimensionality not known.");
    // TODO(frank): this might need to be re-thought
    static std::uniform_real_distribution<double> randomAngle(-M_PI, M_PI);
    const size_t d = SO::Dimension(n);
    Vector xi(d);
    for (size_t j = 0; j < d; j++) {
      xi(j) = randomAngle(rng);
    }
    return SO::Retract(xi);
  }

  /// Random SO(N) element (no big claims about uniformity)
  template <int N_ = N, typename = IsFixed<N_>>
  static SO Random(std::mt19937& rng) {
    // By default, use dynamic implementation above. Specialized for SO(3).
    return SO(SO<Eigen::Dynamic>::Random(rng, N).matrix());
  }

  /// @}
  /// @name Standard methods
  /// @{

  /// Return matrix
  const MatrixNN& matrix() const { return matrix_; }

  size_t rows() const { return matrix_.rows(); }
  size_t cols() const { return matrix_.cols(); }

  /// @}
  /// @name Testable
  /// @{

  void print(const std::string& s = std::string()) const;

  bool equals(const SO& other, double tol) const {
    return equal_with_abs_tol(matrix_, other.matrix_, tol);
  }

  /// @}
  /// @name Group
  /// @{

  /// Multiplication
  SO operator*(const SO& other) const {
    assert(dim() == other.dim());
    return SO(matrix_ * other.matrix_);
  }

  /// SO<N> identity for N >= 2
  template <int N_ = N, typename = IsFixed<N_>>
  static SO Identity() {
    return SO();
  }

  /// SO<N> identity for N == Eigen::Dynamic
  template <int N_ = N, typename = IsDynamic<N_>>
  static SO Identity(size_t n = 0) {
    return SO(n);
  }

  /// inverse of a rotation = transpose
  SO inverse() const { return SO(matrix_.transpose()); }

  /// @}
  /// @name Manifold
  /// @{

  using TangentVector = Eigen::Matrix<double, dimension, 1>;
  using ChartJacobian = OptionalJacobian<dimension, dimension>;

  /// Return compile-time dimensionality: fixed size N or Eigen::Dynamic
  static int Dim() { return dimension; }

  // Calculate manifold dimensionality for SO(n).
  // Available as dimension or Dim() for fixed N.
  static size_t Dimension(size_t n) { return n * (n - 1) / 2; }

  // Calculate ambient dimension n from manifold dimensionality d.
  static size_t AmbientDim(size_t d) { return (1 + std::sqrt(1 + 8 * d)) / 2; }

  // Calculate run-time dimensionality of manifold.
  // Available as dimension or Dim() for fixed N.
  size_t dim() const { return Dimension(static_cast<size_t>(matrix_.rows())); }

  /**
   * Hat operator creates Lie algebra element corresponding to d-vector, where d
   * is the dimensionality of the manifold. This function is implemented
   * recursively, and the d-vector is assumed to laid out such that the last
   * element corresponds to so(2), the last 3 to so(3), the last 6 to so(4)
   * etc... For example, the vector-space isomorphic to so(5) is laid out as:
   *   a b c d | u v w | x y | z
   * where the latter elements correspond to "telescoping" sub-algebras:
   *   0 -z  y  w -d
   *   z  0 -x -v  c
   *  -y  x  0  u -b
   *  -w  v -u  0  a
   *   d -c  b -a  0
   * This scheme behaves exactly as expected for SO(2) and SO(3).
   */
  static MatrixNN Hat(const TangentVector& xi);

  /// In-place version of Hat (see details there), implements recursion.
  static void Hat(const Vector &xi, Eigen::Ref<MatrixNN> X);

  /// Inverse of Hat. See note about xi element order in Hat.
  static TangentVector Vee(const MatrixNN& X);

  // Chart at origin
  struct ChartAtOrigin {
    /**
     * Retract uses Cayley map. See note about xi element order in Hat.
     * Deafault implementation has no Jacobian implemented
     */
    static SO Retract(const TangentVector& xi, ChartJacobian H = {});

    /**
     * Inverse of Retract. See note about xi element order in Hat.
     */
    static TangentVector Local(const SO& R, ChartJacobian H = {});
  };

  // Return dynamic identity DxD Jacobian for given SO(n)
  template <int N_ = N, typename = IsDynamic<N_>>
  static MatrixDD IdentityJacobian(size_t n) {
    const size_t d = Dimension(n);
    return MatrixDD::Identity(d, d);
  }

  /// @}
  /// @name Lie Group
  /// @{

  /// Adjoint map
  MatrixDD AdjointMap() const;

  /**
   * Exponential map at identity - create a rotation from canonical coordinates
   */
  static SO Expmap(const TangentVector& omega, ChartJacobian H = {});

  /// Derivative of Expmap, currently only defined for SO3
  static MatrixDD ExpmapDerivative(const TangentVector& omega);

  /**
   * Log map at identity - returns the canonical coordinates of this rotation
   */
  static TangentVector Logmap(const SO& R, ChartJacobian H = {});

  /// Derivative of Logmap, currently only defined for SO3
  static MatrixDD LogmapDerivative(const TangentVector& omega);

  // inverse with optional derivative
  using LieGroup<SO<N>, internal::DimensionSO(N)>::inverse;

  /// @}
  /// @name Other methods
  /// @{

  /**
   * Return vectorized rotation matrix in column order.
   * Will use dynamic matrices as intermediate results, but returns a fixed size
   * X and fixed-size Jacobian if dimension is known at compile time.
   * */
  VectorN2 vec(OptionalJacobian<internal::NSquaredSO(N), dimension> H =
                   {}) const;

  /// Calculate N^2 x dim matrix of vectorized Lie algebra generators for SO(N)
  template <int N_ = N, typename = IsFixed<N_>>
  static Matrix VectorizedGenerators() {
    constexpr size_t N2 = static_cast<size_t>(N * N);
    Eigen::Matrix<double, N2, dimension> G;
    for (size_t j = 0; j < dimension; j++) {
      const auto X = Hat(Vector::Unit(dimension, j));
      G.col(j) = Eigen::Map<const VectorN2>(X.data());
    }
    return G;
  }

  /// Calculate n^2 x dim matrix of vectorized Lie algebra generators for SO(n)
  template <int N_ = N, typename = IsDynamic<N_>>
  static Matrix VectorizedGenerators(size_t n = 0) {
    const size_t n2 = n * n, dim = Dimension(n);
    Matrix G(n2, dim);
    for (size_t j = 0; j < dim; j++) {
      const auto X = Hat(Vector::Unit(dim, j));
      G.col(j) = Eigen::Map<const Matrix>(X.data(), n2, 1);
    }
    return G;
  }

  /// @{
  /// @name Serialization
  /// @{

  template <class Archive>
  friend void save(Archive&, SO&, const unsigned int);
  template <class Archive>
  friend void load(Archive&, SO&, const unsigned int);
  template <class Archive>
  friend void serialize(Archive&, SO&, const unsigned int);
  friend class boost::serialization::access;
  friend class Rot3;  // for serialize

  /// @}
};

using SOn = SO<Eigen::Dynamic>;

/*
 * Specialize dynamic Hat and Vee, because recursion depends on dynamic nature.
 * The definition is in SOn.cpp. Fixed-size SO3 and SO4 have their own version,
 * and implementation for other fixed N is in SOn-inl.h.
 */

template <>
GTSAM_EXPORT
Matrix SOn::Hat(const Vector& xi);

template <>
GTSAM_EXPORT
Vector SOn::Vee(const Matrix& X);

/*
 * Specialize dynamic compose and between, because the derivative is unknowable
 * by the LieGroup implementations, who return a fixed-size matrix for H2.
 */

using DynamicJacobian = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;

template <>
GTSAM_EXPORT
SOn LieGroup<SOn, Eigen::Dynamic>::compose(const SOn& g, DynamicJacobian H1,
                                           DynamicJacobian H2) const;

template <>
GTSAM_EXPORT
SOn LieGroup<SOn, Eigen::Dynamic>::between(const SOn& g, DynamicJacobian H1,
                                           DynamicJacobian H2) const;

/*
 * Specialize dynamic vec.
 */
template <> 
GTSAM_EXPORT
typename SOn::VectorN2 SOn::vec(DynamicJacobian H) const;

/** Serialization function */
template<class Archive>
void serialize(
  Archive& ar, SOn& Q,
  const unsigned int file_version
) {
  Matrix& M = Q.matrix_;
  ar& BOOST_SERIALIZATION_NVP(M);
}

/*
 * Define the traits. internal::LieGroup provides both Lie group and Testable
 */

template <int N>
struct traits<SO<N>> : public internal::LieGroup<SO<N>> {};

template <int N>
struct traits<const SO<N>> : public internal::LieGroup<SO<N>> {};

}  // namespace gtsam

#include "SOn-inl.h"
