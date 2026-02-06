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
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>

#include <iomanip>

using std::setw, std::cout, std::endl, std::setprecision;

namespace gtsam {

/* ************************************************************************* */
void AugmentedLagrangianState::initializeLagrangeMultipliers(
    const ConstrainedOptProblem& problem) {
  lambdaEq = std::vector<Vector>();
  lambdaEq.reserve(problem.eConstraints().size());
  for (const auto& constraint : problem.eConstraints()) {
    lambdaEq.push_back(Vector::Zero(constraint->dim()));
  }
  lambdaIneq = std::vector<double>(problem.iConstraints().size(), 0);
}

/* ************************************************************************* */
std::tuple<AugmentedLagrangianOptimizer::State, double, double>
AugmentedLagrangianOptimizer::iterate(const State& state, const double muEq,
                                      const double muIneq) const {
  State newState(state.iteration + 1);
  newState.muEq = muEq;
  newState.muIneq = muIneq;

  // Update Lagrangian multipliers.
  updateLagrangeMultiplier(state, &newState);

  // Construct merit function.
  const NonlinearFactorGraph augmentedLagrangian =
      augmentedLagrangianFunction(newState);

  // Run unconstrained optimization.
  auto optimizer =
      createUnconstrainedOptimizer(augmentedLagrangian, state.values);
  newState.setValues(optimizer->optimize(), problem_);
  newState.unconstrainedIterations = optimizer->iterations();

  // Update penalty parameters for next iteration.
  double next_muEq, next_muIneq;
  std::tie(next_muEq, next_muIneq) = updatePenaltyParameter(state, newState);

  return {newState, next_muEq, next_muIneq};
}

/* ************************************************************************* */
Values AugmentedLagrangianOptimizer::optimize() const {
  /// Construct initial state
  State previousState;
  State state(0, initialValues_, problem_);
  state.initializeLagrangeMultipliers(problem_);
  logInitialState(state);

  /// Set penalty parameters for the first iteration.
  double muEq = p_->initialMuEq;
  double muIneq = p_->initialMuIneq;

  /// iterates
  do {
    previousState = std::move(state);
    std::tie(state, muEq, muIneq) = iterate(previousState, muEq, muIneq);
    logIteration(state);
  } while (!checkConvergence(state, previousState, *p_));

  return state.values;
}

/* ************************************************************************* */
NonlinearFactorGraph AugmentedLagrangianOptimizer::augmentedLagrangianFunction(
    const State& state, const double epsilon) const {
  return AugmentedLagrangianFunction(
      problem_, state.lambdaEq, state.lambdaIneq, state.muEq, state.muIneq,
      p_->ineqConstraintPenaltyFunction, epsilon);
}

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::updateLagrangeMultiplier(
    const State& previousState, State* state) const {
  // Perform dual ascent on Lagrange multipliers for e-constriants.
  const NonlinearEqualityConstraints& eqConstraints = problem_.eConstraints();
  state->lambdaEq.resize(eqConstraints.size());
  for (size_t i = 0; i < eqConstraints.size(); i++) {
    const auto& constraint = eqConstraints.at(i);
    // Compute constraint violation as the gradient of the dual function.
    Vector violation = constraint->whitenedError(previousState.values);
    double step_size = std::min(p_->maxDualStepSizeEq,
                                previousState.muEq * p_->dualStepSizeFactorEq);
    state->lambdaEq[i] = previousState.lambdaEq[i] + step_size * violation;
  }

  // Perform dual ascent on Lagrange multipliers for i-constriants.
  const NonlinearInequalityConstraints& ineqConstraints =
      problem_.iConstraints();
  state->lambdaIneq.resize(ineqConstraints.size());
  // Update Lagrangian multipliers.
  for (size_t i = 0; i < ineqConstraints.size(); i++) {
    const auto& constraint = ineqConstraints.at(i);
    double violation = constraint->whitenedExpr(previousState.values)(0);
    double step_size =
        std::min(p_->maxDualStepSizeIneq,
                 previousState.muIneq * p_->dualStepSizeFactorIneq);
    state->lambdaIneq[i] =
        std::max(0.0, previousState.lambdaIneq[i] + step_size * violation);
  }
}

/* ************************************************************************* */
std::pair<double, double> AugmentedLagrangianOptimizer::updatePenaltyParameter(
    const State& previousState, const State& state) const {
  double muEq = state.muEq;
  if (problem_.eConstraints().size() > 0 &&
      state.eqConstraintViolation >=
          p_->muIncreaseThreshold * previousState.eqConstraintViolation) {
    muEq *= p_->muEqIncreaseRate;
  }

  double muIneq = state.muIneq;
  if (problem_.iConstraints().size() > 0 &&
      state.ineqConstraintViolation >=
          p_->muIncreaseThreshold * previousState.ineqConstraintViolation) {
    muIneq *= p_->muIneqIncreaseRate;
  }
  return {muEq, muIneq};
}

/* ************************************************************************* */
ConstrainedOptimizer::SharedOptimizer
AugmentedLagrangianOptimizer::createUnconstrainedOptimizer(
    const NonlinearFactorGraph& graph, const Values& values) const {
  // TODO(yetong): make compatible with all NonlinearOptimizers.
  return std::make_shared<LevenbergMarquardtOptimizer>(graph, values,
                                                       p_->lm_params);
}

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::logInitialState(const State& state) const {
  if (p_->verbose) {
    // Log title line.
    cout << setw(10) << "Iter"
         << "|" << setw(10) << "muEq"
         << "|" << setw(10) << "muIneq"
         << "|" << setw(10) << "cost"
         << "|" << setw(10) << "vio_e"
         << "|" << setw(10) << "vio_i"
         << "|" << setw(10) << "uopt_iters"
         << "|" << setw(10) << "time"
         << "|" << endl;

    // Log initial value line.
    cout << setw(10) << state.iteration;
    cout << "|" << setw(10) << "-";
    cout << "|" << setw(10) << "-";
    cout << "|" << setw(10) << setprecision(4) << state.cost;
    cout << "|" << setw(10) << setprecision(4) << state.eqConstraintViolation;
    cout << "|" << setw(10) << setprecision(4) << state.ineqConstraintViolation;
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
void AugmentedLagrangianOptimizer::logIteration(const State& state) const {
  if (p_->verbose) {
    cout << setw(10) << state.iteration;
    cout << "|" << setw(10) << state.muEq;
    cout << "|" << setw(10) << state.muIneq;
    cout << "|" << setw(10) << setprecision(4) << state.cost;
    cout << "|" << setw(10) << setprecision(4) << state.eqConstraintViolation;
    cout << "|" << setw(10) << setprecision(4) << state.ineqConstraintViolation;
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
