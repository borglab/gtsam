/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  PenaltyOptimizer.cpp
 * @brief Penalty method optimization routines.
 * @author: Yetong Zhang
 */

#include <gtsam/constrained/PenaltyOptimizer.h>

#include <iomanip>

using std::setw, std::cout, std::endl, std::setprecision;

namespace gtsam {

/* ************************************************************************* */
PenaltyOptimizer::State PenaltyOptimizer::iterate(const State& state) const {
  State newState(state.iteration + 1);

  // Update penalty parameter.
  newState.muEq = state.iteration == 0 ? p_->initialMuEq
                                       : state.muEq * p_->muEqIncreaseRate;
  newState.muIneq = state.iteration == 0
                        ? p_->initialMuIneq
                        : state.muIneq * p_->muIneqIncreaseRate;

  // Construct merit function.
  NonlinearFactorGraph meritGraph =
      meritFunction(newState.muEq, newState.muIneq);

  // Run unconstrained optimization.
  auto optimizer = createUnconstrainedOptimizer(meritGraph, state.values);
  newState.setValues(optimizer->optimize(), problem_);
  newState.unconstrainedIterationss = optimizer->iterations();

  return newState;
}

/* ************************************************************************* */
Values PenaltyOptimizer::optimize() const {
  /// Construct initial state
  State previousState;
  State state(0, initialValues_, problem_);
  logInitialState(state);

  do {
    previousState = std::move(state);
    state = iterate(previousState);
    logIteration(state);
  } while (!checkConvergence(state, previousState, *p_));

  return state.values;
}

/* ************************************************************************* */
NonlinearFactorGraph PenaltyOptimizer::meritFunction(
    const double muEq, const double muIneq) const {
  NonlinearFactorGraph graph = problem_.costs();
  graph.add(problem_.eConstraints().penaltyGraph(muEq));
  graph.add(problem_.iConstraints().penaltyGraphCustom(
      p_->ineqConstraintPenaltyFunction, muIneq));
  return graph;
}

/* ************************************************************************* */
PenaltyOptimizer::SharedOptimizer
PenaltyOptimizer::createUnconstrainedOptimizer(
    const NonlinearFactorGraph& graph, const Values& values) const {
  // TODO(yetong): make compatible with all NonlinearOptimizers.
  return std::make_shared<LevenbergMarquardtOptimizer>(graph, values,
                                                       p_->lm_params);
}

/* ************************************************************************* */
void PenaltyOptimizer::logInitialState(const State& state) const {
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
void PenaltyOptimizer::logIteration(const State& state) const {
  if (p_->verbose) {
    cout << setw(10) << state.iteration;
    cout << "|" << setw(10) << state.muEq;
    cout << "|" << setw(10) << state.muIneq;
    cout << "|" << setw(10) << setprecision(4) << state.cost;
    cout << "|" << setw(10) << setprecision(4) << state.eqConstraintViolation;
    cout << "|" << setw(10) << setprecision(4) << state.ineqConstraintViolation;
    cout << "|" << setw(10) << state.unconstrainedIterationss;
    cout << "|" << setw(10) << state.time;
    cout << "|" << endl;
  }

  // Store state
  if (p_->storeOptProgress) {
    progress_.emplace_back(state);
  }
}

}  // namespace gtsam
