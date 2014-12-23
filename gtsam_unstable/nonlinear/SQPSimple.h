/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SQPSimple.h
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#pragma once
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearEqualityFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearInequalityFactorGraph.h>
#include <gtsam_unstable/linear/LinearInequalityFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearConstraint.h>
#include <gtsam_unstable/linear/QPSolver.h>

namespace gtsam {

struct NLP {
  NonlinearFactorGraph cost;
  NonlinearEqualityFactorGraph linearEqualities;
  NonlinearEqualityFactorGraph nonlinearEqualities;
  NonlinearInequalityFactorGraph linearInequalities;
};

struct SQPSimpleState {
  Values values;
  VectorValues duals;
  bool converged;
  size_t iterations;

  /// Default constructor
  SQPSimpleState() : values(), duals(), converged(false), iterations(0) {}

  /// Constructor with an initialValues
  SQPSimpleState(const Values& initialValues) :
      values(initialValues), duals(VectorValues()), converged(false), iterations(0) {
  }
};

/**
 * Simple SQP optimizer to solve nonlinear constrained problems.
 * This simple version won't care about nonconvexity, which needs
 * more advanced techniques to solve, e.g., merit function, line search, second-order correction etc.
 */
class SQPSimple {
  NLP nlp_;
  static const double errorTol = 1e-5;
public:
  SQPSimple(const NLP& nlp) :
      nlp_(nlp) {
  }

  /// Check if \nabla f(x) - \lambda * \nabla c(x) == 0
  bool isStationary(const VectorValues& delta) const;

  /// Check if c_E(x) == 0
  bool isPrimalFeasible(const SQPSimpleState& state) const;

  /**
   * Dual variables of inequality constraints need to be >=0
   * For active inequalities, the dual needs to be > 0
   * For inactive inequalities, they need to be == 0. However, we don't compute
   * dual variables for inactive constraints in the qp subproblem, so we don't care.
   */
  bool isDualFeasible(const VectorValues& duals) const;

  /**
   * Check complimentary slackness condition:
   * For all inequality constraints,
   *        dual * constraintError(primals) == 0.
   * If the constraint is active, we need to check constraintError(primals) == 0, and ignore the dual
   * If it is inactive, the dual should be 0, regardless of the error. However, we don't compute
   * dual variables for inactive constraints in the QP subproblem, so we don't care.
   */
  bool isComplementary(const SQPSimpleState& state) const;

  /// Check convergence
  bool checkConvergence(const SQPSimpleState& state, const VectorValues& delta) const;

  /**
   * Single iteration of SQP
   */
  SQPSimpleState iterate(const SQPSimpleState& state) const;

  VectorValues initializeDuals() const;
  /**
   * Main optimization function.
   */
  std::pair<Values, VectorValues> optimize(const Values& initialValues) const;

};
}
