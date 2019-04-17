/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO4.h
 * @brief   4*4 matrix representation of SO(4)
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    March 2019
 */

#pragma once

#include <gtsam/base/Group.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>

#include <boost/random/mersenne_twister.hpp>

#include <iosfwd>
#include <string>

namespace gtsam {

/**
 *  True SO(4), i.e., 4*4 matrix subgroup
 */
class SO4 : public Matrix4, public LieGroup<SO4, 6> {
 public:
  enum { N = 4 };
  enum { dimension = 6 };

  typedef Eigen::Matrix<double, 16, 1> Vector16;

  /// @name Constructors
  /// @{

  /// Default constructor creates identity
  SO4() : Matrix4(I_4x4) {}

  /// Constructor from Eigen Matrix
  template <typename Derived>
  SO4(const MatrixBase<Derived> &R) : Matrix4(R.eval()) {}

  /// Random SO(4) element (no big claims about uniformity)
  static SO4 Random(boost::mt19937 &rng);

  /// @}
  /// @name Testable
  /// @{

  void print(const std::string &s) const;
  bool equals(const SO4 &R, double tol) const;

  /// @}
  /// @name Group
  /// @{

  /// identity rotation for group operation
  static SO4 identity() { return I_4x4; }

  /// inverse of a rotation = transpose
  SO4 inverse() const { return this->transpose(); }

  /// @}
  /// @name Lie Group
  /// @{

  static Matrix4 Hat(const Vector6 &xi);  ///< make skew symmetric matrix
  static Vector6 Vee(const Matrix4 &X);   ///< inverse of Hat
  static SO4 Expmap(const Vector6 &xi,
                    ChartJacobian H = boost::none);  ///< exponential map
  static Vector6 Logmap(const SO4 &Q,
                        ChartJacobian H = boost::none);  ///< and its inverse
  Matrix6 AdjointMap() const;

  // Chart at origin
  struct ChartAtOrigin {
    static SO4 Retract(const Vector6 &omega, ChartJacobian H = boost::none);
    static Vector6 Local(const SO4 &R, ChartJacobian H = boost::none);
  };

  using LieGroup<SO4, 6>::inverse;

  /// @}
  /// @name Other methods
  /// @{

  /// Vectorize
  Vector16 vec(OptionalJacobian<16, 6> H = boost::none) const;

  /// Project to top-left 3*3 matrix. Note this is *not* in general \in SO(3).
  Matrix3 topLeft(OptionalJacobian<9, 6> H = boost::none) const;

  /// Project to Stiefel manifold of 4*3 orthonormal 3-frames in R^4, i.e., pi(Q) -> S \in St(3,4).
  Matrix43 stiefel(OptionalJacobian<12, 6> H = boost::none) const;

  /// Return matrix (for wrapper)
  const Matrix4 &matrix() const { return *this; }

  /// @

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &boost::serialization::make_nvp("Q11", (*this)(0, 0));
    ar &boost::serialization::make_nvp("Q12", (*this)(0, 1));
    ar &boost::serialization::make_nvp("Q13", (*this)(0, 2));
    ar &boost::serialization::make_nvp("Q14", (*this)(0, 3));

    ar &boost::serialization::make_nvp("Q21", (*this)(1, 0));
    ar &boost::serialization::make_nvp("Q22", (*this)(1, 1));
    ar &boost::serialization::make_nvp("Q23", (*this)(1, 2));
    ar &boost::serialization::make_nvp("Q24", (*this)(1, 3));

    ar &boost::serialization::make_nvp("Q31", (*this)(2, 0));
    ar &boost::serialization::make_nvp("Q32", (*this)(2, 1));
    ar &boost::serialization::make_nvp("Q33", (*this)(2, 2));
    ar &boost::serialization::make_nvp("Q34", (*this)(2, 3));

    ar &boost::serialization::make_nvp("Q41", (*this)(3, 0));
    ar &boost::serialization::make_nvp("Q42", (*this)(3, 1));
    ar &boost::serialization::make_nvp("Q43", (*this)(3, 2));
    ar &boost::serialization::make_nvp("Q44", (*this)(3, 3));
  }

};  // SO4

template <>
struct traits<SO4> : Testable<SO4>, internal::LieGroupTraits<SO4> {};

template <>
struct traits<const SO4> : Testable<SO4>, internal::LieGroupTraits<SO4> {};

}  // end namespace gtsam
