/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QuadraticConstraint.h
 * @brief   Quadratic constraints for QCQP problems.
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>

namespace gtsam {

/**
 * Scalar quadratic constraint trace(X' A X) + 2 a' x - b with a relation sense.
 *
 * Direct Vector values are treated as one-column matrices. For Matrix values,
 * A is a row-space matrix and the constraint is applied across all columns:
 * trace(X' A X) = <A, X X'>. The linear term 2 a' x is defined only for
 * one-column values; it is zero unless given explicitly.
 *
 * The (A, a, b) convention x' A x + 2 a' x ~ b, with the factor 2 on the
 * linear term, follows Zhi-Quan Luo, Wing-Kin Ma, Anthony Man-Cho So, Yinyu
 * Ye, and Shuzhong Zhang, "Semidefinite Relaxation of Quadratic Optimization
 * Problems," IEEE Signal Processing Magazine 27(3):20-34, 2010,
 * https://doi.org/10.1109/MSP.2010.936019. FromPqr() accepts the Boyd and
 * Vandenberghe form 0.5 x' P x + q' x + r ~ 0 instead.
 */
class GTSAM_EXPORT QuadraticConstraint {
 public:
  using shared_ptr = std::shared_ptr<QuadraticConstraint>;

  /// Supported quadratic constraint relations.
  enum class Sense { Equal, LessEqual, GreaterEqual };

  /** Default constructor for I/O. */
  QuadraticConstraint() = default;

  /** Construct a scalar quadratic constraint. */
  QuadraticConstraint(Key key, const Matrix& A, double b, Sense sense,
                      double sigma);

  /** Construct a scalar quadratic constraint with unit sigma. */
  QuadraticConstraint(Key key, const Matrix& A, double b, Sense sense)
      : QuadraticConstraint(key, A, b, sense, 1.0) {}

  /**
   * Construct x' A x + 2 a' x - b for a one-column value x. An empty a means
   * no linear term.
   */
  QuadraticConstraint(Key key, const Matrix& A, const Vector& a, double b,
                      Sense sense, double sigma = 1.0);

  /**
   * Construct 0.5*x' P x + q' x + r ~ 0.
   * The relation is specified by sense.
   *
   * Uses Boyd and Vandenberghe's quadratic-function convention (Convex
   * Optimization, Cambridge University Press, 2004, Sec. 4.4, Eq. (4.35));
   * converts to the internal Luo et al. form x' A x + 2a' x ~ b.
   */
  static QuadraticConstraint FromPqr(Key key, const Matrix& P, const Vector& q,
                                     double r, Sense sense,
                                     double sigma = 1.0) {
    return QuadraticConstraint(key, 0.5 * P, 0.5 * q, -r, sense, sigma);
  }

  /// Create trace(X' A X) - b = 0.
  static QuadraticConstraint Equal(Key key, const Matrix& A, double b) {
    return QuadraticConstraint(key, A, b, Sense::Equal);
  }

  /// Create trace(X' A X) - b = 0 with explicit sigma.
  static QuadraticConstraint Equal(Key key, const Matrix& A, double b,
                                   double sigma) {
    return QuadraticConstraint(key, A, b, Sense::Equal, sigma);
  }

  /// Create x' A x + 2 a' x - b = 0.
  static QuadraticConstraint Equal(Key key, const Matrix& A, const Vector& a,
                                   double b) {
    return QuadraticConstraint(key, A, a, b, Sense::Equal);
  }

  /**
   * Create x' A x + 2 a' x - b = 0 with explicit sigma. MATLAB callers must
   * use this overload to distinguish it from Equal(key, A, b, sigma).
   */
  static QuadraticConstraint Equal(Key key, const Matrix& A, const Vector& a,
                                   double b, double sigma) {
    return QuadraticConstraint(key, A, a, b, Sense::Equal, sigma);
  }

  /// Create trace(X' A X) - b <= 0.
  static QuadraticConstraint LessEqual(Key key, const Matrix& A, double b) {
    return QuadraticConstraint(key, A, b, Sense::LessEqual);
  }

  /// Create trace(X' A X) - b <= 0 with explicit sigma.
  static QuadraticConstraint LessEqual(Key key, const Matrix& A, double b,
                                       double sigma) {
    return QuadraticConstraint(key, A, b, Sense::LessEqual, sigma);
  }

  /// Create trace(X' A X) - b >= 0.
  static QuadraticConstraint GreaterEqual(Key key, const Matrix& A, double b) {
    return QuadraticConstraint(key, A, b, Sense::GreaterEqual);
  }

  /// Create trace(X' A X) - b >= 0 with explicit sigma.
  static QuadraticConstraint GreaterEqual(Key key, const Matrix& A, double b,
                                          double sigma) {
    return QuadraticConstraint(key, A, b, Sense::GreaterEqual, sigma);
  }

  /// Return the constrained key.
  Key key() const { return key_; }

  /// Dense symmetric constraint matrix.
  const Matrix& A() const { return A_; }

  /// Linear term a of x' A x + 2 a' x, zero when none was given.
  const Vector& a() const { return a_; }

  /// Return true if the constraint has a nonzero linear term.
  bool hasLinearTerm() const { return !a_.isZero(0.0); }

  /// Right-hand side of trace(X' A X) = b.
  double b() const { return b_; }

  /// Return the relation sense.
  Sense sense() const { return sense_; }

  /// Return true if this is an equality constraint.
  bool isEquality() const { return sense_ == Sense::Equal; }

  /// Return the sigma used by constrained optimizers.
  double sigma() const { return sigma_; }

  /** Create the nonlinear equality factor for this constraint. */
  NonlinearEqualityConstraint::shared_ptr createEqualityFactor() const;

  /** Create the nonlinear inequality factor for this constraint. */
  NonlinearInequalityConstraint::shared_ptr createInequalityFactor() const;

 private:
  Key key_ = 0;
  Matrix A_;
  Vector a_;
  double b_ = 0.0;
  Sense sense_ = Sense::Equal;
  double sigma_ = 1.0;
};

/** Nonlinear equality wrapper created from a QuadraticConstraint. */
class GTSAM_EXPORT QuadraticEqualityConstraintFactor
    : public NonlinearEqualityConstraint {
 public:
  using shared_ptr = std::shared_ptr<QuadraticEqualityConstraintFactor>;

  /** Construct from a quadratic equality constraint. */
  explicit QuadraticEqualityConstraintFactor(
      const QuadraticConstraint& constraint)
      : NonlinearEqualityConstraint(
            constrainedNoise(Vector1(constraint.sigma())),
            KeyVector{constraint.key()}),
        constraint_(constraint) {}

  /// Return the wrapped quadratic constraint.
  const QuadraticConstraint& quadraticConstraint() const { return constraint_; }

  /** Evaluate the signed quadratic equality residual. */
  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override;

  /** Return a deep copy of this factor. */
  NonlinearFactor::shared_ptr clone() const override {
    return NonlinearFactor::shared_ptr(
        new QuadraticEqualityConstraintFactor(*this));
  }

 private:
  QuadraticConstraint constraint_;
};

/** Nonlinear inequality wrapper created from a QuadraticConstraint. */
class GTSAM_EXPORT QuadraticInequalityConstraintFactor
    : public NonlinearInequalityConstraint {
 public:
  using shared_ptr = std::shared_ptr<QuadraticInequalityConstraintFactor>;

  /** Construct from a quadratic inequality constraint. */
  explicit QuadraticInequalityConstraintFactor(
      const QuadraticConstraint& constraint)
      : NonlinearInequalityConstraint(
            constrainedNoise(Vector1(constraint.sigma())),
            KeyVector{constraint.key()}),
        constraint_(constraint) {}

  /// Return the wrapped quadratic constraint.
  const QuadraticConstraint& quadraticConstraint() const { return constraint_; }

  /** Evaluate the signed quadratic inequality expression. */
  Vector unwhitenedExpr(const Values& values,
                        OptionalMatrixVecType H = nullptr) const override;

  /** Return the corresponding boundary equality factor. */
  NonlinearEqualityConstraint::shared_ptr createEqualityConstraint()
      const override {
    return std::make_shared<QuadraticEqualityConstraintFactor>(constraint_);
  }

  /** Return a deep copy of this factor. */
  NonlinearFactor::shared_ptr clone() const override {
    return NonlinearFactor::shared_ptr(
        new QuadraticInequalityConstraintFactor(*this));
  }

 private:
  QuadraticConstraint constraint_;
};

inline NonlinearEqualityConstraint::shared_ptr
QuadraticConstraint::createEqualityFactor() const {
  return std::make_shared<QuadraticEqualityConstraintFactor>(*this);
}

}  // namespace gtsam
