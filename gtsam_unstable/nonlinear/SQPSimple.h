/**
 * @file	SQPSimple.h
 * @author 	Krunal Chande
 * @date	Dec 22, 2014
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
  bool isStationary(const VectorValues& delta) const {
    return delta.vector().lpNorm<Eigen::Infinity>() < errorTol;
  }

  /// Check if c_E(x) == 0
  bool isPrimalFeasible(const SQPSimpleState& state) const {
    return nlp_.linearEqualities.checkFeasibility(state.values, errorTol)
        && nlp_.nonlinearEqualities.checkFeasibility(state.values, errorTol);
  }

  /**
   * Dual variables of inequality constraints need to be >=0
   * For active inequalities, the dual needs to be > 0
   * For inactive inequalities, they need to be == 0. However, we don't compute
   * dual variables for inactive constraints in the qp subproblem, so we don't care.
   */
  bool isDualFeasible(const VectorValues& duals) const {
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, nlp_.linearInequalities) {
      NonlinearConstraint::shared_ptr inequality = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
      Key dualKey = inequality->dualKey();
      if (!duals.exists(dualKey)) continue; // should be inactive constraint!
      double dual = duals.at(dualKey)[0];   // because we only support single-valued inequalities
      if (dual < 0.0)
        return false;
    }
    return true;
  }

  /**
   * Check complimentary slackness condition:
   * For all inequality constraints,
   *        dual * constraintError(primals) == 0.
   * If the constraint is active, we need to check constraintError(primals) == 0, and ignore the dual
   * If it is inactive, the dual should be 0, regardless of the error. However, we don't compute
   * dual variables for inactive constraints in the QP subproblem, so we don't care.
   */
  bool isComplementary(const SQPSimpleState& state) const {
    return nlp_.linearInequalities.checkFeasibilityAndComplimentary(state.values, state.duals, errorTol);
  }

  /// Check convergence
  bool checkConvergence(const SQPSimpleState& state, const VectorValues& delta) const {
    return isStationary(delta) && isPrimalFeasible(state) && isDualFeasible(state.duals) && isComplementary(state);
  }

  /**
   * Single iteration of SQP
   */
  SQPSimpleState iterate(const SQPSimpleState& state) const {
    static const bool debug = false;

    // construct the qp subproblem
    QP qp;
    qp.cost = *nlp_.cost.linearize(state.values);
    GaussianFactorGraph::shared_ptr multipliedHessians = nlp_.nonlinearEqualities.multipliedHessians(state.values, state.duals);
    qp.cost.push_back(*multipliedHessians);

    qp.equalities.add(*nlp_.linearEqualities.linearize(state.values));
    qp.equalities.add(*nlp_.nonlinearEqualities.linearize(state.values));

    qp.inequalities.add(*nlp_.linearInequalities.linearize(state.values));

    if (debug)
      qp.print("QP subproblem:");

    // solve the QP subproblem
    VectorValues delta, duals;
    QPSolver qpSolver(qp);
    boost::tie(delta, duals) = qpSolver.optimize();

    if (debug)
      delta.print("delta = ");

    if (debug)
      duals.print("duals = ");

    // update new state
    SQPSimpleState newState;
    newState.values = state.values.retract(delta);
    newState.duals = duals;
    newState.converged = checkConvergence(newState, delta);
    newState.iterations = state.iterations + 1;

    return newState;
  }

  VectorValues initializeDuals() const {
    VectorValues duals;
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, nlp_.linearEqualities) {
      NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
      duals.insert(constraint->dualKey(), zero(factor->dim()));
    }

    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, nlp_.nonlinearEqualities) {
      NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
      duals.insert(constraint->dualKey(), zero(factor->dim()));
    }
    return duals;
  }

  /**
   * Main optimization function.
   */
  std::pair<Values, VectorValues> optimize(const Values& initialValues) const {
    SQPSimpleState state(initialValues);
    state.duals = initializeDuals();

    while (!state.converged && state.iterations < 100) {
      state = iterate(state);
    }

    return std::make_pair(state.values, state.duals);
  }


};
}
