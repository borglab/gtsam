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

#pragma once

#include <gtsam/constrained/ConstrainedOptimizer.h>
#include <gtsam/constrained/PenaltyOptimizer.h>

namespace gtsam {

/// Parameters for Augmented Lagrangian method
class GTSAM_EXPORT AugmentedLagrangianParams : public PenaltyOptimizerParams {
 public:
  using Base = PenaltyOptimizerParams;
  using This = AugmentedLagrangianParams;
  using shared_ptr = std::shared_ptr<AugmentedLagrangianParams>;

  double maxDualStepSizeEq = 10;    // maximum step size for dual ascent
  double maxDualStepSizeIneq = 10;  // maximum step size for dual ascent
  double dualStepSizeFactorEq = 1.0;
  double dualStepSizeFactorIneq = 1.0;
  double muIncreaseThreshold = 0.25;

  using Base::Base;
};

/// Details for each iteration.
class GTSAM_EXPORT AugmentedLagrangianState : public PenaltyOptimizerState {
 public:
  using Base = PenaltyOptimizerState;
  using This = AugmentedLagrangianState;
  using shared_ptr = std::shared_ptr<This>;

  std::vector<Vector> lambdaEq;    // Lagrange multipliers for e-constraints
  std::vector<double> lambdaIneq;  // Lagrange multipliers for i-constraints

  /** Initialize Lagrange multipliers as zeros. */
  void initializeLagrangeMultipliers(const ConstrainedOptProblem& problem);

  using Base::Base;
};

/** Augmented Lagrangian method to solve constrained nonlinear least squares
 * problems. The implementation follows
 * https://www.seas.ucla.edu/~vandenbe/133B/lectures/nllseq.pdf for problems
 * with equality constraints only. We further generalize the implementation to
 * incorporate inequality constraints with reference to
 * https://www.stat.cmu.edu/~ryantibs/convexopt/scribes/dual-decomp-scribed.pdf.
 * Example problem:
 * argmin_x  0.5 * ||x1-1||^2 + 0.5 * ||x2-1||^2
 *      s.t.  x1^2 + x2^2 - 1 = 0
 *      s.t.  4*x1^2 + 0.25*x2^2 - 1 <= 0
 */
class GTSAM_EXPORT AugmentedLagrangianOptimizer : public ConstrainedOptimizer {
 public:
  using Base = ConstrainedOptimizer;
  using This = PenaltyOptimizer;
  using shared_ptr = std::shared_ptr<This>;

  using Params = AugmentedLagrangianParams;
  using State = AugmentedLagrangianState;
  using Progress = std::vector<AugmentedLagrangianState>;

 protected:
  Params::shared_ptr p_;
  mutable Progress progress_;

 public:
  /// Constructor.
  AugmentedLagrangianOptimizer(
      const ConstrainedOptProblem& problem, const Values& initialValues,
      Params::shared_ptr p = std::make_shared<Params>())
      : Base(problem, initialValues), p_(p), progress_() {}

  /// Constructor that packs costs and constraints into a single factor graph.
  AugmentedLagrangianOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      Params::shared_ptr p = std::make_shared<Params>())
      : Base(graph, initialValues), p_(p), progress_() {}

  /// Run optimization with equality constraints only.
  Values optimize() const override;

  std::tuple<State, double, double> iterate(const State& state,
                                            const double muEq,
                                            const double muIneq) const;

  /// Return progress of iterations.
  const Progress& progress() const { return progress_; }

  /**
   * Lagrange dual function for equality constraints and inequality constraints
   *   m(x) = 0.5 * ||f(x)||^2 - lambdaEq * h(x) + 0.5 * muEq * ||h(x)||^2
   *                           - lambdaIneq * g(x) + 0.5 * muIneq * ||g(x)_-||^2
   * To express in nonlinear least squares form, it is rewritten as
   *     m(x)
   *   = m(x) + 0.5 * epsilon * ||g(x)||^2
   *   = 0.5 * ||f(x)||^2
   *     + (0.5 * muEq * ||h(x)||^2 - lambdaEq * h(x))
   *     + (0.5 * epsilon * ||g(x)||^2 - lambdaIneq * g(x))
   *     + 0.5 * muIneq * ||g(x)_-||^2
   *     - 0.5 * epsilon * ||g(x)||^2
   *   = 0.5 * ||f(x)||^2
   *     + 0.5 * muEq * ||h(x)- lambdaEq/muEq||^2
   *     + 0.5 * epsilon * ||g(x)-lambdaIneq/muIneq||^2
   *     + 0.5 * muIneq * ||g(x)_-||^2
   *     - 0.5 * epsilon * ||g(x)||^2
   *     - c
   * where
   *   c = ||lambdaEq||^2 / (2 * muEq) + ||lambdaIneq||^2 / (2 * epsilon)
   * is a constant term,
   * and epsilon can be any positive scalar value.
   *
   * Notice: the purpose of epsilon is to incorporate (-lambdaIneq * g(x)) in
   * nonlinear least squares form. To do so, we manually create an additional
   * term (0.5 * epsilon * ||g(x)||^2), which is added and then subtracted in
   * the merit function. The term (-lambdaIneq * g(x)) and (0.5 * epsilon *
   * ||g(x)||^2) can be combined as a least-square term, and the subtraction of
   * (0.5 * epsilon * ||g(x)||^2) can be performed with anti-factor.
   * @return: factor graph representing m(x) + 0.5d * ||g(x)||^2 + c
   */
  NonlinearFactorGraph augmentedLagrangianFunction(
      const State& state, const double epsilon = 1.0) const;

 protected:
  /// Create an unconstrained optimizer that solves the augmented Lagrangian.
  SharedOptimizer createUnconstrainedOptimizer(
      const NonlinearFactorGraph& graph, const Values& values) const;

  /** Update the Lagrange multipliers using dual ascent. */
  void updateLagrangeMultiplier(const State& prev_state, State* state) const;

  /// Set the penalty parameters for the state using results from prev state.
  std::pair<double, double> updatePenaltyParameter(const State& prev_state,
                                                   const State& state) const;

  /// Store and log the initial state of the optimization.
  void logInitialState(const State& state) const;

  /// Store and log the state after any iteration of the optimization.
  void logIteration(const State& state) const;
};

}  // namespace gtsam
