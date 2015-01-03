/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LCNLPSolver.h
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearEqualityFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearInequalityFactorGraph.h>

namespace gtsam {

/**
 * Nonlinear Programming problem with
 * only linear constraints, encoded in factor graphs
 */
struct LCNLP {
  NonlinearFactorGraph cost;
  NonlinearEqualityFactorGraph linearEqualities;
  NonlinearInequalityFactorGraph linearInequalities;
};

/**
 * State of LCNLPSolver before/after each iteration
 */
struct LCNLPState {
  Values values;
  VectorValues duals;
  bool converged;
  size_t iterations;

  /// Default constructor
  LCNLPState() : values(), duals(), converged(false), iterations(0) {}

  /// Constructor with an initialValues
  LCNLPState(const Values& initialValues) :
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

/**
 * Simple SQP optimizer to solve nonlinear constrained problems
 * ONLY linear constraints are supported.
 */
class LCNLPSolver {
  LCNLP lcnlp_;
  static const double errorTol = 1e-5;
public:
  LCNLPSolver(const LCNLP& lcnlp) :
      lcnlp_(lcnlp) {
  }

  /// Check if \nabla f(x) - \lambda * \nabla c(x) == 0
  bool isStationary(const VectorValues& delta) const;

  /// Check if c_E(x) == 0
  bool isPrimalFeasible(const LCNLPState& state) const;

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
  bool isComplementary(const LCNLPState& state) const;

  /// Check convergence
  bool checkConvergence(const LCNLPState& state, const VectorValues& delta) const;

  /**
   * Single iteration of SQP
   */
  LCNLPState iterate(const LCNLPState& state, bool useWarmStart = true, bool debug = false) const;

  VectorValues initializeDuals() const;
  /**
   * Main optimization function. new
   */
  std::pair<Values, VectorValues> optimize(const Values& initialValues, bool useWarmStart = true, bool debug = false) const;

};
}
