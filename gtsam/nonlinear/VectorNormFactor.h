/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file VectorNormFactor.h
 * @brief Factor constraining the norm of a vector-space variable
 * @date July 2026
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Unary factor constraining the norm of an N-dimensional vector-space variable
 * to a given value, with error = ||v|| - norm.
 *
 * A typical use is a magnitude pseudo-observation on a physical vector whose
 * direction and magnitude are entangled in a single variable, e.g. an
 * estimated gravity vector (see ImuFactorWithGravityVector and
 * [Lupton and Sukkarieh, TRO 2012]) or a local magnetic field vector
 * (see MagFactor2). Add it once per variable: adding it per measurement
 * factor would count the same prior knowledge multiple times.
 *
 * @warning The error is not differentiable at ||v|| = 0; below kMinNorm the
 * Jacobian is set to zero, so the factor exerts no pull there. In particular,
 * an optimizer whose initial estimate for the variable is exactly zero will
 * receive no gradient from this factor and the variable will not move.
 * Always initialize the variable with a non-zero guess.
 *
 * This is a hand-written factor rather than an ExpressionFactor over norm3
 * for discoverability and to avoid the expression-tree evaluation overhead at
 * every linearization; see gtsam/slam/expressions.h for the expression
 * building blocks if composition with other expressions is needed.
 *
 * With a noiseModel::Constrained model this acts as a hard constraint under
 * QR-based elimination (not recommended with Cholesky factorization). If the
 * norm is known exactly, prefer a parametrization that fixes it instead,
 * e.g. Unit3 and a fixed magnitude (see ImuFactorWithGravityDirection).
 *
 * @ingroup nonlinear
 */
template <int N>
class VectorNormFactor : public NoiseModelFactorN<Eigen::Matrix<double, N, 1>> {
  static_assert(N > 0, "VectorNormFactor requires a fixed positive dimension");

 public:
  typedef Eigen::Matrix<double, N, 1> VectorN;

 private:
  typedef VectorNormFactor<N> This;
  typedef NoiseModelFactorN<VectorN> Base;

  double norm_;  ///< desired norm of the vector

 public:
  /// Below this norm the error is treated as non-differentiable and the
  /// Jacobian is zero (see class warning).
  static constexpr double kMinNorm = 1e-10;

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Shorthand for a smart pointer to a factor */
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor - only use for serialization */
  VectorNormFactor() : norm_(0.0) {}

  /**
   * Constructor
   * @param key of the vector-space variable
   * @param norm the desired norm of the vector
   * @param model of the (1-dimensional) noise on the norm error
   */
  VectorNormFactor(Key key, double norm, const SharedNoiseModel& model)
      : Base(model, key), norm_(norm) {}

  ~VectorNormFactor() override {}

  /// @return a deep copy of this factor
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  /// @name Testable
  /// @{
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? s : s + " ") << "VectorNormFactor on "
              << keyFormatter(this->template key<1>()) << ", norm = " << norm_
              << std::endl;
    this->noiseModel_->print("  noise model: ");
  }

  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           std::abs(norm_ - e->norm_) < tol;
  }
  /// @}

  /// The desired norm
  double norm() const { return norm_; }

  /// vector of errors: ||v|| - norm
  Vector evaluateError(const VectorN& v, OptionalMatrixType H) const override {
    const double vnorm = v.norm();
    if (H) {
      if (vnorm > kMinNorm) {
        *H = v.transpose() / vnorm;
      } else {
        // The norm is not differentiable at the origin. Return a zero
        // Jacobian: the factor exerts no pull on a (near-)zero vector.
        H->setZero(1, N);
      }
    }
    return Vector1(vnorm - norm_);
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(norm_);
  }
#endif
};

/// traits
template <int N>
struct traits<VectorNormFactor<N>> : public Testable<VectorNormFactor<N>> {};

}  // namespace gtsam
