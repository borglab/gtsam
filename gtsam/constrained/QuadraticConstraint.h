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
 * Scalar quadratic constraint trace(X' A X) - b with a relation sense.
 *
 * Direct Vector values are treated as one-column matrices. For Matrix values,
 * A is a row-space matrix and the constraint is applied across all columns:
 * trace(X' A X) = <A, X X'>.
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

  /// Create trace(X' A X) - b = 0.
  static QuadraticConstraint Equal(Key key, const Matrix& A, double b) {
    return QuadraticConstraint(key, A, b, Sense::Equal);
  }

  /// Create trace(X' A X) - b = 0 with explicit sigma.
  static QuadraticConstraint Equal(Key key, const Matrix& A, double b,
                                   double sigma) {
    return QuadraticConstraint(key, A, b, Sense::Equal, sigma);
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
