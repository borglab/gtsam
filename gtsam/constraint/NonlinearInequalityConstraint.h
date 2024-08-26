/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearInequalityConstraint.h
 * @brief   Nonlinear inequality constraints in constrained optimization.
 * @author  Yetong Zhang, Frank Dellaert
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constraint/InequalityPenaltyFunction.h>
#include <gtsam/constraint/NonlinearEqualityConstraint.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {

/**
 * Inequality constraint base class, enforcing g(x) <= 0.
 */
class GTSAM_EXPORT NonlinearInequalityConstraint : public NonlinearConstraint {
 public:
  typedef NonlinearConstraint Base;
  typedef NonlinearInequalityConstraint This;
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor. */
  using Base::Base;

  /** Destructor. */
  virtual ~NonlinearInequalityConstraint() {}

  /** Return g(x). */
  virtual Vector unwhitenedExpr(const Values& x, OptionalMatrixVecType H = nullptr) const = 0;

  virtual Vector whitenedExpr(const Values& x) const;

  /** Return ramp(g(x)). */
  virtual Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = nullptr) const override;

  /** Return true if g(x)>=0 in any dimension. */
  virtual bool active(const Values& x) const override;

  /** Return an equality constraint corresponding to g(x)=0. */
  virtual NonlinearEqualityConstraint::shared_ptr createEqualityConstraint() const;

  /** Cost factor using a customized penalty function. */
  virtual NoiseModelFactor::shared_ptr penaltyFactorCustom(
      InequalityPenaltyFunction::shared_ptr func, const double mu = 1.0) const;

  /** penalty function as if the constraint is equality, 0.5 * mu * ||g(x)||^2 */
  virtual NoiseModelFactor::shared_ptr penaltyFactorEquality(const double mu = 1.0) const;

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("NonlinearInequalityConstraint",
                                       boost::serialization::base_object<Base>(*this));
  }
#endif
};

/** Inequality constraint that force g(x) <= 0, where g(x) is a scalar-valued
 * nonlinear function.
 */
class GTSAM_EXPORT ScalarExpressionInequalityConstraint : public NonlinearInequalityConstraint {
 public:
  typedef NonlinearInequalityConstraint Base;
  typedef ScalarExpressionInequalityConstraint This;
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  Double_ expression_;
  FastVector<int> dims_;

 public:
  /**
   * @brief Constructor.
   *
   * @param expression  expression representing g(x) (or -g(x) for GeqZero).
   * @param sigma   scalar representing sigma.
   */
  ScalarExpressionInequalityConstraint(const Double_& expression, const double& sigma);

  /** Create an inequality constraint g(x)/sigma >= 0, internally represented as -g(x)/sigma <= 0.
   */
  static ScalarExpressionInequalityConstraint::shared_ptr GeqZero(const Double_& expression,
                                                                  const double& sigma);

  /** Create an inequality constraint g(x)/sigma <= 0. */
  static ScalarExpressionInequalityConstraint::shared_ptr LeqZero(const Double_& expression,
                                                                  const double& sigma);

  /** Compute g(x), or -g(x) for objects constructed from GeqZero. */
  virtual Vector unwhitenedExpr(const Values& x, OptionalMatrixVecType H = nullptr) const override;

  /** Equality constraint representing g(x)/sigma = 0. */
  NonlinearEqualityConstraint::shared_ptr createEqualityConstraint() const override;

  /** Penalty function 0.5*mu*||ramp(g(x)/sigma||^2. */
  NoiseModelFactor::shared_ptr penaltyFactor(const double mu = 1.0) const override;

  /** Penalty function using a smooth approxiamtion of the ramp funciton. */
  NoiseModelFactor::shared_ptr penaltyFactorCustom(InequalityPenaltyFunction::shared_ptr func,
                                                   const double mu = 1.0) const override;

  /** Penalty function as if the constraint is equality, 0.5 * mu * ||g(x)/sigma||^2. */
  virtual NoiseModelFactor::shared_ptr penaltyFactorEquality(const double mu = 1.0) const override;

  /** Return expression g(x), or -g(x) for objects constructed from GeqZero. */
  const Double_& expression() const { return expression_; }

  /** return a deep copy of this factor. */
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("ExpressionEqualityConstraint",
                                       boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(expression_);
    ar& BOOST_SERIALIZATION_NVP(dims_);
  }
#endif
};

/// Container of NonlinearInequalityConstraint.
class GTSAM_EXPORT NonlinearInequalityConstraints
    : public FactorGraph<NonlinearInequalityConstraint> {
 public:
  typedef FactorGraph<NonlinearInequalityConstraint> Base;
  typedef NonlinearInequalityConstraints This;
  typedef std::shared_ptr<This> shared_ptr;

  using Base::Base;

  /** Return the total dimension of constraints. */
  size_t dim() const;

  /** Evaluate the constraint violation as a vector. */
  Vector violationVector(const Values& values, bool whiten = true) const;

  /** Evaluate the constraint violation (as L2 norm). */
  double violationNorm(const Values& values) const;

  /** Return the penalty function corresponding to \sum_i||ramp(g_i(x))||^2. */
  NonlinearFactorGraph penaltyGraph(const double mu = 1.0) const;

  /** Return the cost graph constructed using a customized penalty function. */
  NonlinearFactorGraph penaltyGraphCustom(InequalityPenaltyFunction::shared_ptr func,
                                          const double mu = 1.0) const;

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("NonlinearInequalityConstraints",
                                       boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gtsam
