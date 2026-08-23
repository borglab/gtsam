/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearConstraint.h
 * @brief   Linear constraints for LP, QP, and QCQP problems.
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/constrained/NonlinearInequalityConstraint.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>

namespace gtsam {

/**
 * Linear constraint over direct Vector and Matrix Values entries.
 *
 * A LinearConstraint stores A*x-b with a relation sense. Matrix entries are
 * interpreted by column-major vectorization. The owning problem converts the
 * constraint into the equality or inequality container required by the
 * constrained optimizers.
 */
class GTSAM_EXPORT LinearConstraint {
 public:
  using shared_ptr = std::shared_ptr<LinearConstraint>;

  /// Supported linear constraint relations.
  enum class Sense { Equal, LessEqual, GreaterEqual };

  /** Construct from a Jacobian factor, relation sense, and sigma vector. */
  LinearConstraint(const JacobianFactor& factor, Sense sense,
                   const Vector& sigmas);

  /** Construct from a Jacobian factor with unit sigmas. */
  LinearConstraint(const JacobianFactor& factor, Sense sense)
      : LinearConstraint(factor, sense, Vector::Ones(factor.getb().size())) {}

  /// Create A*d-b = 0.
  static LinearConstraint Equal(const JacobianFactor& factor) {
    return LinearConstraint(factor, Sense::Equal);
  }

  /// Create A*d-b = 0 with explicit sigmas.
  static LinearConstraint Equal(const JacobianFactor& factor,
                                const Vector& sigmas) {
    return LinearConstraint(factor, Sense::Equal, sigmas);
  }

  /// Create A*d-b <= 0.
  static LinearConstraint LessEqual(const JacobianFactor& factor) {
    return LinearConstraint(factor, Sense::LessEqual);
  }

  /// Create A*d-b <= 0 with explicit sigmas.
  static LinearConstraint LessEqual(const JacobianFactor& factor,
                                    const Vector& sigmas) {
    return LinearConstraint(factor, Sense::LessEqual, sigmas);
  }

  /// Create A*d-b >= 0.
  static LinearConstraint GreaterEqual(const JacobianFactor& factor) {
    return LinearConstraint(factor, Sense::GreaterEqual);
  }

  /// Create A*d-b >= 0 with explicit sigmas.
  static LinearConstraint GreaterEqual(const JacobianFactor& factor,
                                       const Vector& sigmas) {
    return LinearConstraint(factor, Sense::GreaterEqual, sigmas);
  }

  /// Return the relation sense.
  Sense sense() const { return sense_; }

  /// Return true if this is an equality constraint.
  bool isEquality() const { return sense_ == Sense::Equal; }

  /// Return the stored Jacobian factor.
  const JacobianFactor& factor() const { return *factor_; }

  /// Return the sigma vector used by constrained optimizers.
  const Vector& sigmas() const { return sigmas_; }

  /** Create the nonlinear equality factor for this constraint. */
  NonlinearEqualityConstraint::shared_ptr createEqualityFactor() const;

  /** Create the nonlinear inequality factor for this constraint. */
  NonlinearInequalityConstraint::shared_ptr createInequalityFactor() const;

 private:
  JacobianFactor::shared_ptr factor_;
  Sense sense_;
  Vector sigmas_;
};

/** Nonlinear equality wrapper created from a LinearConstraint. */
class GTSAM_EXPORT LinearEqualityConstraintFactor
    : public NonlinearEqualityConstraint {
 public:
  using shared_ptr = std::shared_ptr<LinearEqualityConstraintFactor>;

  /** Construct from a linear equality constraint. */
  explicit LinearEqualityConstraintFactor(const LinearConstraint& constraint)
      : NonlinearEqualityConstraint(constrainedNoise(constraint.sigmas()),
                                    constraint.factor().keys()),
        constraint_(constraint) {}

  /** Return the wrapped linear constraint. */
  const LinearConstraint& linearConstraint() const { return constraint_; }

  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override;

  NonlinearFactor::shared_ptr clone() const override {
    return NonlinearFactor::shared_ptr(
        new LinearEqualityConstraintFactor(*this));
  }

 private:
  LinearConstraint constraint_;
};

/** Nonlinear inequality wrapper created from a LinearConstraint. */
class GTSAM_EXPORT LinearInequalityConstraintFactor
    : public NonlinearInequalityConstraint {
 public:
  using shared_ptr = std::shared_ptr<LinearInequalityConstraintFactor>;

  /** Construct from a linear inequality constraint. */
  explicit LinearInequalityConstraintFactor(const LinearConstraint& constraint)
      : NonlinearInequalityConstraint(constrainedNoise(constraint.sigmas()),
                                      constraint.factor().keys()),
        constraint_(constraint) {}

  /** Return the wrapped linear constraint. */
  const LinearConstraint& linearConstraint() const { return constraint_; }

  Vector unwhitenedExpr(const Values& values,
                        OptionalMatrixVecType H = nullptr) const override;

  NonlinearEqualityConstraint::shared_ptr createEqualityConstraint()
      const override {
    return std::make_shared<LinearEqualityConstraintFactor>(constraint_);
  }

  NonlinearFactor::shared_ptr clone() const override {
    return NonlinearFactor::shared_ptr(
        new LinearInequalityConstraintFactor(*this));
  }

 private:
  LinearConstraint constraint_;
};

inline NonlinearEqualityConstraint::shared_ptr
LinearConstraint::createEqualityFactor() const {
  return std::make_shared<LinearEqualityConstraintFactor>(*this);
}

}  // namespace gtsam
