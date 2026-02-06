/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AugmentedLagrangianOptimizer.h
 * @brief   Augmented Lagrangian method for nonlinear constrained optimization.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#include <gtsam/constrained/AugmentedLagrangian.h>
#include <gtsam/constrained/BoundConstrainedLagrangian.h>

#include <cmath>

using std::setw, std::cout, std::endl, std::setprecision;

namespace gtsam {

/* ************************************************************************* */
BoundConstrainedLagrangian::State BoundConstrainedLagrangian::iterate(
    const State& state) const {
  /// Solve one step of LM iteration.
  // Construct merit function.
  const NonlinearFactorGraph augmentedLagrangian =
      augmentedLagrangianFunction(state);

  // Run unconstrained optimization.
  auto optimizer =
      createUnconstrainedOptimizer(augmentedLagrangian, state.values);

  State newState(state.iteration + 1);
  newState.setValues(optimizer->optimize(), problem_);
  newState.unconstrainedIterations = optimizer->iterations();

  // Update Lagrangian multipliers, penalty parameters, and omega.
  updateMultipliers(state, &newState);

  return newState;
}

double InfNorm(const VectorValues& vector_values) {
  double inf_norm = 0;
  for (const auto& [key, value] : vector_values) {
    inf_norm = std::max(inf_norm, value.lpNorm<Eigen::Infinity>());
  }
  return inf_norm;
}

/* ************************************************************************* */
void BoundConstrainedLagrangian::updateMultipliers(const State& previousState,
                                                   State* state) const {
  // Compute the inf norm of the cost gradient
  const NonlinearFactorGraph& cost = problem_.costs();
  auto gauss_graph = cost.linearize(previousState.values);
  VectorValues cost_gradient = gauss_graph->gradientAtZero();
  double gradient_inf_norm = InfNorm(cost_gradient);

  if (gradient_inf_norm < previousState.omega) {
    // compute constraint violations
    const NonlinearEqualityConstraints& eqConstraints = problem_.eConstraints();
    std::vector<Vector> constraint_violations;
    constraint_violations.reserve(eqConstraints.size());
    for (const auto& constraint : eqConstraints) {
      constraint_violations.push_back(
          constraint->whitenedError(previousState.values));
    }

    // Compute violation norm
    double constraint_violation_inf_norm = 0;
    for (const auto& violation : constraint_violations) {
      constraint_violation_inf_norm = std::max(
          constraint_violation_inf_norm, violation.lpNorm<Eigen::Infinity>());
    }

    if (constraint_violation_inf_norm < previousState.ita) {
      state->lambdaEq.resize(eqConstraints.size());
      for (size_t i = 0; i < eqConstraints.size(); i++) {
        state->lambdaEq[i] = previousState.lambdaEq[i] +
                             previousState.muEq * constraint_violations[i];
      }
      state->muEq = previousState.muEq;
      state->ita = previousState.ita / std::pow(previousState.muEq, p_->alpha);
      state->omega = previousState.omega / previousState.muEq;
    } else {
      state->lambdaEq = previousState.lambdaEq;
      state->muEq = previousState.muEq * p_->k;
      state->ita = previousState.ita;
      state->omega = previousState.omega;
    }
  }
}

/* ************************************************************************* */
ConstrainedOptimizer::SharedOptimizer
BoundConstrainedLagrangian::createUnconstrainedOptimizer(
    const NonlinearFactorGraph& graph, const Values& values) const {
  // TODO(yetong): make compatible with all NonlinearOptimizers.
  return std::make_shared<LevenbergMarquardtOptimizer>(graph, values,
                                                       p_->lmParams);
}

/* ************************************************************************* */
// TODO(yetong): currently copy-pasted from AugmentedLagrangianOptimizer.cpp,
// need to redesign.
NonlinearFactorGraph BoundConstrainedLagrangian::augmentedLagrangianFunction(
    const State& state, const double epsilon) const {
  // Initialize by adding in cost factors.
  return AugmentedLagrangianFunction(
      problem_, state.lambdaEq, state.lambdaIneq, state.muEq, state.muIneq,
      p_->ineqConstraintPenaltyFunction, epsilon);
}

/* ************************************************************************* */
Values BoundConstrainedLagrangian::optimize() const {
  /// Construct initial state
  State previousState;
  State state(0, initialValues_, problem_);
  state.initializeLagrangeMultipliers(problem_);
  state.muEq = p_->initialMuEq;
  state.ita = p_->ita0;
  state.omega = p_->omega0;
  logInitialState(state);

  /// iterates
  do {
    previousState = std::move(state);
    state = iterate(previousState);
    logIteration(state);
  } while (!checkConvergence(state, previousState, *p_));

  return state.values;
}

/* ************************************************************************* */
void BoundConstrainedLagrangian::logInitialState(const State& state) const {
  if (p_->verbose) {
    // Log title line.
    cout << setw(10) << "Iter"
         << "|" << setw(10) << "muEq"
         << "|" << setw(10) << "cost"
         << "|" << setw(10) << "vio_e"
         << "|" << setw(10) << "uopt_iters"
         << "|" << setw(10) << "time"
         << "|" << endl;

    // Log initial value line.
    cout << setw(10) << state.iteration;
    cout << "|" << setw(10) << "-";
    cout << "|" << setw(10) << setprecision(4) << state.cost;
    cout << "|" << setw(10) << setprecision(4) << state.eqConstraintViolation;
    cout << "|" << setw(10) << "-";
    cout << "|" << setw(10) << "-";
    cout << "|" << endl;
  }

  // Store state
  if (p_->storeOptProgress) {
    progress_.emplace_back(state);
  }
}

/* ************************************************************************* */
void BoundConstrainedLagrangian::logIteration(const State& state) const {
  if (p_->verbose) {
    cout << setw(10) << state.iteration;
    cout << "|" << setw(10) << state.muEq;
    cout << "|" << setw(10) << setprecision(4) << state.cost;
    cout << "|" << setw(10) << setprecision(4) << state.eqConstraintViolation;
    cout << "|" << setw(10) << state.unconstrainedIterations;
    cout << "|" << setw(10) << state.time;
    cout << "|" << endl;
  }

  // Store state
  if (p_->storeOptProgress) {
    progress_.emplace_back(state);
  }
}

}  // namespace gtsam
