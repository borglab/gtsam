/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ExtendedPose3.h
 * @brief   Extended pose Lie group SE_k(3), with static or dynamic k.
 * @author  Frank Dellaert, et al.
 */

#pragma once

#include <gtsam/base/MatrixLieGroup.h>
#include <gtsam/geometry/Kernel.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

#include <cassert>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif

namespace gtsam {

template <int K, class Derived = void>
class ExtendedPose3;

/**
 * Lie group SE_k(3): semidirect product of SO(3) with k copies of R^3.
 * State ordering is (R, x_1, ..., x_k) with R in SO(3) and x_i in R^3.
 * Tangent ordering is [omega, rho_1, ..., rho_k], each block in R^3.
 *
 * The manifold dimension is 3+3k and the homogeneous matrix size is 3+k.
 * Template parameter K can be fixed (K >= 1) or Eigen::Dynamic.
 */
template <int K_, class Derived>
class ExtendedPose3 : public MatrixLieGroup<
                          std::conditional_t<std::is_void_v<Derived>,
                                             ExtendedPose3<K_, void>, Derived>,
                          (K_ == Eigen::Dynamic) ? Eigen::Dynamic : 3 + 3 * K_,
                          (K_ == Eigen::Dynamic) ? Eigen::Dynamic : 3 + K_> {
 public:
  static constexpr int K = K_;
  using This = std::conditional_t<std::is_void_v<Derived>,
                                  ExtendedPose3<K, void>, Derived>;
  inline constexpr static int dimension =
      (K == Eigen::Dynamic) ? Eigen::Dynamic : 3 + 3 * K;
  inline constexpr static int matrixDim =
      (K == Eigen::Dynamic) ? Eigen::Dynamic : 3 + K;

  using Base = MatrixLieGroup<This, dimension, matrixDim>;
  using TangentVector = typename Base::TangentVector;
  using Jacobian = typename Base::Jacobian;
  using ChartJacobian = typename Base::ChartJacobian;
  using ComponentJacobian =
      std::conditional_t<dimension == Eigen::Dynamic,
                         OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>,
                         OptionalJacobian<3, dimension>>;
  /// Homogeneous matrix representation in the group.
  using MatrixRep = Eigen::Matrix<double, matrixDim, matrixDim>;
  /// Lie algebra matrix type used by Hat/Vee.
  using LieAlgebra = Eigen::Matrix<double, matrixDim, matrixDim>;
  using Matrix3K = Eigen::Matrix<double, 3, K>;

  static_assert(K == Eigen::Dynamic || K >= 1,
                "ExtendedPose3<K>: K should be >= 1 or Eigen::Dynamic.");

 protected:
  Rot3 R_;      ///< Rotation component.
  Matrix3K t_;  ///< K translation-like columns in world frame.

  template <int K__>
  using IsDynamic = typename std::enable_if<K__ == Eigen::Dynamic, void>::type;
  template <int K__>
  using IsFixed = typename std::enable_if<K__ >= 1, void>::type;

 public:
  /// @name Constructors
  /// @{

  /**
   * Construct a fixed-size identity element.
   *
   * For fixed K, this creates R=I and x_i=0 for i=1..k.
   * The manifold dimension is 3+3k and matrix size is (3+k)x(3+k).
   */
  template <int K__ = K_, typename = IsFixed<K__>>
  ExtendedPose3() : R_(Rot3::Identity()), t_(Matrix3K::Zero()) {}

  /**
   * Construct a dynamic-size identity element.
   *
   * @param k Number of R^3 blocks.
   * Creates R=I and x_i=0 for i=1..k.
   * The manifold dimension is 3+3k and matrix size is (3+k)x(3+k).
   */
  template <int K__ = K_, typename = IsDynamic<K__>>
  explicit ExtendedPose3(size_t k = 0)
      : R_(Rot3::Identity()), t_(3, static_cast<Eigen::Index>(k)) {
    t_.setZero();
  }

  /** Copy constructor. */
  ExtendedPose3(const ExtendedPose3&) = default;

  /** Copy assignment. */
  ExtendedPose3& operator=(const ExtendedPose3&) = default;

  /**
   * Construct from rotation and 3xk block.
   *
   * @param R Rotation in SO(3).
   * @param x Matrix in R^(3xk), where column i stores x_i.
   */
  ExtendedPose3(const Rot3& R, const Matrix3K& x);

  /**
   * Construct a fixed-size state from rotation and K 3-vectors.
   *
   * The vectors are stored in order as x_1, ..., x_K.
   *
   * @param R Rotation in SO(3).
   * @param xs K vector blocks in R^3.
   */
  template <int FixedK = K, typename = IsFixed<FixedK>, typename... Vecs,
            typename = std::enable_if_t<
                sizeof...(Vecs) == FixedK &&
                (std::is_constructible_v<Point3, Vecs> && ...)>>
  ExtendedPose3(const Rot3& R, const Vecs&... xs);

  /**
   * Construct from homogeneous matrix representation.
   *
   * @param T Homogeneous matrix in R^((3+k)x(3+k)).
   * Top-left 3x3 is R, top-right 3xk stores x_1..x_k.
   */
  explicit ExtendedPose3(const MatrixRep& T);

  /// @}
  /// @name Access
  /// @{

  /**
   * Runtime manifold dimension helper.
   *
   * @param k Number of R^3 blocks.
   * @return 3+3k.
   */
  static size_t Dimension(size_t k) { return 3 + 3 * k; }

  /** @return Number of R^3 blocks, k. */
  size_t k() const { return static_cast<size_t>(t_.cols()); }

  /** @return Runtime manifold dimension, 3+3k. */
  size_t dim() const { return Dimension(k()); }

  /**
   * Rotation component.
   *
   * @param H Optional Jacobian in R^(3xdim) for local rotation coordinates.
   * @return Rotation R.
   */
  const Rot3& rotation(ComponentJacobian H = {}) const;

  /**
   * i-th R^3 component, returned by value.
   *
   * @param i Zero-based block index in [0, k).
   * @param H Optional Jacobian in R^(3xdim).
   * @return x_i in R^3.
   */
  Point3 x(size_t i, ComponentJacobian H = {}) const;

  /**
   * Access all x_i blocks.
   *
   * @return Matrix in R^(3xk) with columns x_1..x_k.
   */
  const Matrix3K& xMatrix() const;

  /**
   * Mutable access to all x_i blocks.
   *
   * @return Matrix in R^(3xk) with columns x_1..x_k.
   */
  Matrix3K& xMatrix();

  /// @}
  /// @name Testable
  /// @{

  /**
   * Print this state.
   *
   * @param s Optional prefix string.
   */
  void print(const std::string& s = "") const;

  /**
   * Equality check with tolerance.
   *
   * @param other Other state.
   * @param tol Absolute tolerance.
   * @return True if rotation and all x_i blocks are equal within tol.
   */
  bool equals(const ExtendedPose3& other, double tol = 1e-9) const;

  /// @}
  /// @name Group
  /// @{

  /**
   * Identity element for fixed-size K.
   *
   * @return Identity with manifold dimension 3+3k and matrix size 3+k.
   */
  template <int K__ = K_, typename = IsFixed<K__>>
  static This Identity() {
    return MakeReturn(ExtendedPose3());
  }

  /**
   * Identity element for dynamic-size K.
   *
   * @param k Number of R^3 blocks.
   * @return Identity with manifold dimension 3+3k and matrix size 3+k.
   */
  template <int K__ = K_, typename = IsDynamic<K__>>
  static This Identity(size_t k = 0) {
    return MakeReturn(ExtendedPose3(k));
  }

  /**
   * Group inverse.
   *
   * @return X^{-1}.
   */
  This inverse() const;

  /**
   * Group composition.
   *
   * @param other Right-hand operand with the same k.
   * @return this * other.
   */
  This operator*(const This& other) const;

  /// @}
  /// @name Lie Group
  /// @{

  /**
   * Exponential map from tangent to group.
   *
   * @param xi Tangent vector in R^dim.
   * @param Hxi Optional Jacobian in R^(dimxdim).
   * @return Group element in SE_k(3).
   */
  static This Expmap(const TangentVector& xi, ChartJacobian Hxi = {});

  /**
   * Logarithm map from group to tangent.
   *
   * @param pose Group element in SE_k(3).
   * @param Hpose Optional Jacobian in R^(dimxdim).
   * @return Tangent vector in R^dim.
   */
  static TangentVector Logmap(const This& pose, ChartJacobian Hpose = {});

  /**
   * Adjoint map.
   *
   * @return Matrix in R^(dimxdim).
   */
  Jacobian AdjointMap() const;

  /**
   * Lie algebra adjoint map.
   *
   * @param xi Tangent vector in R^dim.
   * @return ad_xi matrix in R^(dimxdim).
   */
  static Jacobian adjointMap(const TangentVector& xi);

  /**
   * Jacobian of Expmap.
   *
   * @param xi Tangent vector in R^dim.
   * @return Matrix in R^(dimxdim).
   */
  static Jacobian ExpmapDerivative(const TangentVector& xi);

  /**
   * Jacobian of Logmap evaluated from tangent coordinates.
   *
   * @param xi Tangent vector in R^dim.
   * @return Matrix in R^(dimxdim).
   */
  static Jacobian LogmapDerivative(const TangentVector& xi);

  /**
   * Jacobian of Logmap evaluated at a group element.
   *
   * @param pose Group element in SE_k(3).
   * @return Matrix in R^(dimxdim).
   */
  static Jacobian LogmapDerivative(const This& pose);

  /** Chart operations at identity for LieGroup/Manifold compatibility. */
  struct ChartAtOrigin {
    /**
     * Retract at identity.
     *
     * @param xi Tangent vector in R^dim.
     * @param Hxi Optional Jacobian in R^(dimxdim).
     * @return Expmap(xi).
     */
    static This Retract(const TangentVector& xi, ChartJacobian Hxi = {});

    /**
     * Local coordinates at identity.
     *
     * @param pose Group element in SE_k(3).
     * @param Hpose Optional Jacobian in R^(dimxdim).
     * @return Logmap(pose) in R^dim.
     */
    static TangentVector Local(const This& pose, ChartJacobian Hpose = {});
  };

  using LieGroup<This, dimension>::inverse;

  /// @}
  /// @name Matrix Lie Group
  /// @{

  /**
   * Homogeneous matrix representation.
   *
   * @return Matrix in R^((3+k)x(3+k)).
   */
  MatrixRep matrix() const;

  /**
   * Hat operator from tangent to Lie algebra.
   *
   * @param xi Tangent vector in R^dim.
   * @return Matrix in R^((3+k)x(3+k)).
   */
  static LieAlgebra Hat(const TangentVector& xi);

  /**
   * Vee operator from Lie algebra to tangent.
   *
   * @param X Matrix in R^((3+k)x(3+k)).
   * @return Tangent vector in R^dim.
   */
  static TangentVector Vee(const LieAlgebra& X);

  /// @}

  friend std::ostream& operator<<(std::ostream& os, const ExtendedPose3& p) {
    os << "R: " << p.R_ << "\n";
    os << "x: " << p.t_;
    return os;
  }

 protected:
  static This MakeReturn(const ExtendedPose3& value) {
    if constexpr (std::is_void_v<Derived>) {
      return value;
    } else {
      return This(value);
    }
  }

  static const ExtendedPose3& AsBase(const This& value) {
    if constexpr (std::is_void_v<Derived>) {
      return value;
    } else {
      return static_cast<const ExtendedPose3&>(value);
    }
  }

  static size_t RuntimeK(const TangentVector& xi);
  static void ZeroJacobian(ChartJacobian H, Eigen::Index d);

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(R_);
    ar& BOOST_SERIALIZATION_NVP(t_);
  }
#endif
};

/// Convenience typedef for dynamic-k ExtendedPose3.
using Se23 = ExtendedPose3<2>;
using ExtendedPose3d = ExtendedPose3<Eigen::Dynamic>;

template <int K, class Derived>
struct traits<ExtendedPose3<K, Derived>>
    : public internal::MatrixLieGroup<ExtendedPose3<K, Derived>,
                                      ExtendedPose3<K, Derived>::matrixDim> {};

template <int K, class Derived>
struct traits<const ExtendedPose3<K, Derived>>
    : public internal::MatrixLieGroup<ExtendedPose3<K, Derived>,
                                      ExtendedPose3<K, Derived>::matrixDim> {};

}  // namespace gtsam

#include "ExtendedPose3-inl.h"
