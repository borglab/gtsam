/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BoundConstrainedLagrangian.cpp
 * @brief   Bound-constrained augmented Lagrangian method implementation.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#include <gtsam/constrained/AugmentedLagrangian.h>
#include <gtsam/constrained/BoundConstrainedLagrangian.h>

#include <algorithm>
#include <cmath>
#include <iomanip>

#include "ConstrainedOptimizer.h"

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

  // The paper tests and updates using the point returned by the inner solve.
  // This implementation instead evaluates those tests at `state.values` in
  // updateMultipliers(), so the outer decision is one iterate behind.
  // Update Lagrangian multipliers, penalty parameters, and omega.
  updateMultipliers(state, &newState);

  return newState;
}

static double InfNorm(const VectorValues& vector_values) {
  double inf_norm = 0;
  for (const auto& [key, value] : vector_values) {
    inf_norm = std::max(inf_norm, value.lpNorm<Eigen::Infinity>());
  }
  return inf_norm;
}

/* ************************************************************************* */
void BoundConstrainedLagrangian::updateMultipliers(const State& previousState,
                                                   State* state) const {
  // The paper uses a projected gradient of the augmented Lagrangian, including
  // simple bounds. This implementation uses the infinity norm of the original
  // cost gradient and has no projected/bound-constrained stationarity test.
  const NonlinearFactorGraph& cost = problem_.costs();
  auto gauss_graph = cost.linearize(previousState.values);
  VectorValues cost_gradient = gauss_graph->gradientAtZero();
  double gradient_inf_norm = InfNorm(cost_gradient);

  if (gradient_inf_norm < previousState.omega) {
    // The paper tests the constraint violation at the inner-solver result.
    // This implementation tests the previous outer iterate instead.
    // The BCL decision logic here also handles equality constraints only;
    // the paper's general-constraint treatment is not implemented here.
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

    if (constraint_violation_inf_norm < previousState.eta) {
      state->lambdaEq.resize(eqConstraints.size());
      for (size_t i = 0; i < eqConstraints.size(); i++) {
        // The sign follows GTSAM's constraint residual convention. The paper
        // writes its update using its own c(x) orientation, so this is not a
        // literal sign-level transcription of the displayed formula there.
        state->lambdaEq[i] = previousState.lambdaEq[i] +
                             previousState.muEq * constraint_violations[i];
      }
      state->muEq = previousState.muEq;
      // These fixed threshold updates are a simplified variant of the paper's
      // adaptive parameter rules.
      state->eta = previousState.eta / std::pow(previousState.muEq, p_->alpha);
      state->omega = previousState.omega / previousState.muEq;
    } else {
      state->lambdaEq = previousState.lambdaEq;
      // The paper has algorithm-specific penalty reduction/update rules; this
      // implementation uses a fixed multiplicative increase instead.
      state->muEq = previousState.muEq * p_->k;
      state->eta = previousState.eta;
      state->omega = previousState.omega;
    }
  } else {
    // keep parameters unchanged.
    state->lambdaEq = previousState.lambdaEq;
    state->muEq = previousState.muEq;
    state->eta = previousState.eta;
    state->omega = previousState.omega;
  }
}

/* ************************************************************************* */
ConstrainedOptimizer::SharedOptimizer
BoundConstrainedLagrangian::createUnconstrainedOptimizer(
    const NonlinearFactorGraph& graph, const Values& values) const {
  // TODO(yetong): make compatible with all NonlinearOptimizers.
  // Unlike the paper's projected/bound-constrained inner iteration, this is an
  // unconstrained LM solve, limited by the default parameter to one step.
  return std::make_shared<LevenbergMarquardtOptimizer>(graph, values,
                                                       p_->lmParams);
}

/* ************************************************************************* */
// TODO(yetong): currently copy-pasted from AugmentedLagrangianOptimizer.cpp,
// need to redesign.
// The factor construction uses GTSAM's residual/sign convention; the paper's
// c(x) notation uses a different residual orientation, so the multiplier
// signs should not be compared as a literal transcription.
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
  state.eta = p_->eta0;
  state.omega = p_->omega0;
  logInitialState(state);

  /// iterates
  do {
    previousState = std::move(state);
    state = iterate(previousState);
    logIteration(state);
  } while (!checkConvergenceBC(state, previousState, *p_));

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

bool BoundConstrainedLagrangian::checkConvergenceBC(
    const State& state, const State& previousState,
    const Params& params) const {
  if (state.eta < params.eta_threshold ||
      state.omega < params.omega_threshold) {
    return true;
  }
  return ConstrainedOptimizer::checkConvergence(state, previousState, params);
}

}  // namespace gtsam
