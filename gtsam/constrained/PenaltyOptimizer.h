/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    PenaltyOptimizer.h
 * @brief   Penalty method optimizer for nonlinear constrained optimization.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constrained/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtsam {

/// Parameters for penalty method
class GTSAM_EXPORT PenaltyOptimizerParams : public ConstrainedOptimizerParams {
 public:
  using Base = ConstrainedOptimizerParams;
  using This = PenaltyOptimizerParams;
  using shared_ptr = std::shared_ptr<This>;

  double initialMuEq = 1.0;     // initial penalty parameter
  double initialMuIneq = 1.0;   // initial penalty parameter
  double muEqIncreaseRate = 2;  // increase rate of penalty parameter
  double muIneqIncreaseRate = 2;
  InequalityPenaltyFunction::shared_ptr ineqConstraintPenaltyFunction = nullptr;
  LevenbergMarquardtParams lm_params;

  /** Constructor. */
  PenaltyOptimizerParams() : Base() {}
};

/// Details for each iteration.
class GTSAM_EXPORT PenaltyOptimizerState : public ConstrainedOptimizerState {
 public:
  using Base = ConstrainedOptimizerState;
  using This = PenaltyOptimizerState;
  using shared_ptr = std::shared_ptr<This>;

  double muEq = 0.0;
  double muIneq = 0.0;
  size_t unconstrainedIterationss = 0;

  using Base::Base;
};

/** Penalty method optimizer for solving constrained nonlinear optimization
 * problems. In each iteration, it solves an unconstrained nonlinear
 * optimization problem that minimize a merit function. The merit function is
 * constructed as the sum of the cost function and penalty functions for
 * constraints.
 * Example problem: 
 * argmin_x  0.5 * ||x1-1||^2 + 0.5 * ||x2-1||^2 
 *      s.t.  x1^2 + x2^2 - 1 = 0
 *      s.t.  4*x1^2 + 0.25*x2^2 - 1 <= 0
 */
class GTSAM_EXPORT PenaltyOptimizer : public ConstrainedOptimizer {
 public:
  using Base = ConstrainedOptimizer;
  using This = PenaltyOptimizer;
  using shared_ptr = std::shared_ptr<This>;

  using Params = PenaltyOptimizerParams;
  using State = PenaltyOptimizerState;
  using Progress = std::vector<PenaltyOptimizerState>;

 protected:
  Params::shared_ptr p_;
  mutable Progress progress_;

 public:
  /// Constructor.
  PenaltyOptimizer(const ConstrainedOptProblem& problem,
                   const Values& initialValues,
                   Params::shared_ptr p = std::make_shared<Params>())
      : Base(problem, initialValues), p_(p), progress_() {}

  /// Constructor that packs costs and constraints into a single factor graph.
  PenaltyOptimizer(const NonlinearFactorGraph& graph,
                   const Values& initialValues,
                   Params::shared_ptr p = std::make_shared<Params>())
      : Base(graph, initialValues), p_(p), progress_() {}

  /// Run optimization with equality constraints only.
  Values optimize() const override;

  State iterate(const State& state) const;

  /// Return progress of iterations.
  const Progress& progress() const { return progress_; }

 protected:
  /// Merit function in the form
  ///  m(x) = f(x) + 0.5 muEq * ||h(x)||^2 + 0.5 * muIneq * ||g(x)_+||^2
  NonlinearFactorGraph meritFunction(const double muEq, const double muIneq) const;

  /// Create an unconstrained optimizer that solves the merit function (cost +
  /// penalty functions).
  SharedOptimizer createUnconstrainedOptimizer(
      const NonlinearFactorGraph& graph, const Values& values) const;

  /// Store and log the initial state of the optimization.
  void logInitialState(const State& state) const;

  /// Store and log the state after any iteration of the optimization.
  void logIteration(const State& state) const;
};

}  // namespace gtsam
