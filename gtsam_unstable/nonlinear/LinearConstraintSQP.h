/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearConstraintSQP.h
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam_unstable/nonlinear/LinearEqualityFactorGraph.h>
#include <gtsam_unstable/nonlinear/LinearInequalityFactorGraph.h>

namespace gtsam {

/**
 * Nonlinear Programming problem with
 * only linear constraints, encoded in factor graphs
 */
struct LinearConstraintNLP {
  NonlinearFactorGraph cost;
  LinearEqualityFactorGraph linearEqualities;
  LinearInequalityFactorGraph linearInequalities;
};

/**
 * State of LinearConstraintSQP before/after each iteration
 */
struct LinearConstraintNLPState {
  Values values;        //!< current solution
  VectorValues duals;   //!< current guess of the dual variables
  bool converged;       //!< convergence flag
  size_t iterations;    //!< number of iterations

  /// Default constructor
  LinearConstraintNLPState() : values(), duals(), converged(false), iterations(0) {}

  /// Constructor with an initialValues
  LinearConstraintNLPState(const Values& initialValues) :
      values(initialValues), duals(VectorValues()), converged(false), iterations(0) {
  }

  /// print
  void print(const std::string& s = "") const {
    std::cout << s << std::endl;
    values.print("Values: ");
    duals.print("Duals: ");
    if (converged) std::cout << "Converged!" << std::endl;
    else std::cout << "Not converged" << std::endl;
    std::cout << "Iterations: " << iterations << std::endl;
  }
};

/** Parameters for Gauss-Newton optimization, inherits from
 * NonlinearOptimizationParams.
 */
class GTSAM_EXPORT LinearConstraintSQPParams : public NonlinearOptimizerParams {
public:
  bool warmStart;

  LinearConstraintSQPParams() : NonlinearOptimizerParams(), warmStart(false) {}

  void setWarmStart(bool _warmStart) {
    _warmStart = warmStart;
  }
};

/**
 * Simple SQP optimizer to solve nonlinear constrained problems
 * ONLY linear constraints are supported.
 */
class LinearConstraintSQP {
  LinearConstraintNLP lcnlp_;
  LinearConstraintSQPParams params_;

public:
  LinearConstraintSQP(const LinearConstraintNLP& lcnlp,
      const LinearConstraintSQPParams& params = LinearConstraintSQPParams()) :
        lcnlp_(lcnlp), params_(params) {
  }

  /// Check if \nabla f(x) - \lambda * \nabla c(x) == 0
  bool isStationary(const VectorValues& delta) const;

  /// Check if c_E(x) == 0
  bool isPrimalFeasible(const LinearConstraintNLPState& state) const;

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
  bool isComplementary(const LinearConstraintNLPState& state) const;

  /// Check convergence
  bool checkConvergence(const LinearConstraintNLPState& state, const VectorValues& delta) const;

  /**
   * Single iteration of SQP
   */
  LinearConstraintNLPState iterate(const LinearConstraintNLPState& state) const;

  /// Intialize all dual variables to zeros
  VectorValues initializeDuals() const;

  /**
   * Main optimization function. new
   */
  std::pair<Values, VectorValues> optimize(const Values& initialValues) const;

};
}
