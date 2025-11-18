/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearEqualityConstraint.h
 * @brief   Nonlinear equality constraints in constrained optimization.
 * @author  Yetong Zhang, Frank Dellaert
 * @date    Aug 3, 2024 */

#pragma once

#include <gtsam/constrained/NonlinearConstraint.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/**
 * Equality constraint base class.
 */
class GTSAM_EXPORT NonlinearEqualityConstraint : public NonlinearConstraint {
 public:
  using Base = NonlinearConstraint;
  using This = NonlinearEqualityConstraint;
  using shared_ptr = std::shared_ptr<This>;

  /** Default constructor. */
  using Base::Base;

  /** Destructor. */
  virtual ~NonlinearEqualityConstraint() {}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("NonlinearEqualityConstraint",
                                       boost::serialization::base_object<Base>(*this));
  }
#endif
};

/** Equality constraint that force g(x) = M. */
template <typename T>
class ExpressionEqualityConstraint : public NonlinearEqualityConstraint {
 public:
  using Base = NonlinearEqualityConstraint;
  using This = ExpressionEqualityConstraint;
  using shared_ptr = std::shared_ptr<This>;

 protected:
  Expression<T> expression_;
  T rhs_;
  FastVector<int> dims_;

 public:
  /**
   * @brief Constructor.
   *
   * @param expression  expression representing g(x).
   * @param tolerance   vector representing tolerance in each dimension.
   */
  ExpressionEqualityConstraint(const Expression<T>& expression, const T& rhs, const Vector& sigmas);

  virtual Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = nullptr) const override;

  virtual NoiseModelFactor::shared_ptr penaltyFactor(const double mu = 1.0) const override;

  const Expression<T>& expression() const { return expression_; }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("ExpressionEqualityConstraint",
                                       boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(expression_);
    ar& BOOST_SERIALIZATION_NVP(rhs_);
    ar& BOOST_SERIALIZATION_NVP(dims_);
  }
#endif
};

/** Equality constraint that enforce the cost factor with zero error. 
 * e.g., for a factor with unwhitened cost 2x-1, the constraint enforces the
 * equlity 2x-1=0.
 */
class GTSAM_EXPORT ZeroCostConstraint : public NonlinearEqualityConstraint {
 public:
  using Base = NonlinearEqualityConstraint;
  using This = ZeroCostConstraint;
  using shared_ptr = std::shared_ptr<This>;

 protected:
  NoiseModelFactor::shared_ptr factor_;

 public:
  /**
   * @brief Constructor.
   *
   * @param factor  NoiseModel factor.
   */
  ZeroCostConstraint(const NoiseModelFactor::shared_ptr& factor);

  virtual Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = nullptr) const override;

  virtual NoiseModelFactor::shared_ptr penaltyFactor(const double mu = 1.0) const override;

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("ZeroCostConstraint",
                                       boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(factor_);
  }
#endif
};

/// Container of NonlinearEqualityConstraint.
class GTSAM_EXPORT NonlinearEqualityConstraints : public FactorGraph<NonlinearEqualityConstraint> {
 public:
  using shared_ptr = std::shared_ptr<NonlinearEqualityConstraints>;
  using Base = FactorGraph<NonlinearEqualityConstraint>;

 public:
  using Base::Base;

  /// Create constraints ensuring the cost of factors of a graph is zero.
  static NonlinearEqualityConstraints FromCostGraph(const NonlinearFactorGraph& graph);

  size_t dim() const;

  /// Evaluate the constraint violation as a vector
  Vector violationVector(const Values& values, bool whiten = true) const;

  /// Evaluate the constraint violation (as 2-norm of the violation vector).
  double violationNorm(const Values& values) const;

  NonlinearFactorGraph penaltyGraph(const double mu = 1.0) const;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("NonlinearEqualityConstraints",
                                       boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gtsam

#include <gtsam/constrained/NonlinearEqualityConstraint-inl.h>
