/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  ConstrainedOptimizer.h
 * @brief Base class constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/** Base class for constrained optimization parameters. */
class GTSAM_EXPORT ConstrainedOptimizerParams {
 public:
  using shared_ptr = std::shared_ptr<ConstrainedOptimizerParams>;

  size_t maxIterations = 20;  // maximum number of iterations
  double absoluteViolationTolerance = 1e-5;
  double relativeViolationTolerance = 1e-5;
  double absoluteCostTolerance = 1e-5;
  double relativeCostTolerance = 1e-5;
  bool verbose = false;
  bool storeOptProgress = false;  // Store detailed info of each iter

  /** Constructor. */
  ConstrainedOptimizerParams() {}
};

/** Base class for constrained optimizer state. */
class GTSAM_EXPORT ConstrainedOptimizerState {
 public:
  size_t iteration = 0;
  Values values;
  double cost = 0;
  double eqConstraintViolation = 0;
  double ineqConstraintViolation = 0;
  double time = 0.0;

  ConstrainedOptimizerState() {}

  ConstrainedOptimizerState(const size_t _iteration) : iteration(_iteration) {}

  ConstrainedOptimizerState(const size_t _iteration, const Values& _values,
                            const ConstrainedOptProblem& problem)
      : iteration(_iteration), values(_values) {
    std::tie(cost, eqConstraintViolation, ineqConstraintViolation) =
        problem.evaluate(_values);
  }

  void setValues(const Values& _values, const ConstrainedOptProblem& problem) {
    values = _values;
    std::tie(cost, eqConstraintViolation, ineqConstraintViolation) =
        problem.evaluate(_values);
  }

  double violation() const {
    return sqrt(pow(eqConstraintViolation, 2) +
                pow(ineqConstraintViolation, 2));
  }
};

/** Base class for constrained optimizer. This class is different from
 * NonlinearOptimizer because it also includes constraints in the optimization
 * problem. Although NonlinearOptimizer and ConstrainedOptimizer both solve the
 * optimization problem iteratively, ConstrainedOptimizer does not inherit
 * NonlinearOptimizer because the sub-problems solved in each iteration is
 * different. For NonlinearOptimizer, each iteration solves an unconstrained
 * linear least-squares problem (i.e., GaussianFactorGraph). For certain
 * ConstrainedOptimizer (PenaltyOptimizer and AugmentedLagrangianOptimizer),
 * each iteration solves an unconstrained nonlinear least-squares problem (i.e.,
 * NonlinearFactorGraph).*/
class GTSAM_EXPORT ConstrainedOptimizer {
 public:
  typedef std::shared_ptr<NonlinearOptimizer> SharedOptimizer;
  typedef ConstrainedOptimizerParams Params;
  typedef ConstrainedOptimizerState State;

 protected:
  ConstrainedOptProblem problem_;
  Values initialValues_;

 public:
  /** Default constructor. */
  ConstrainedOptimizer() {}

  /** Constructor. */
  ConstrainedOptimizer(const ConstrainedOptProblem& problem,
                       const Values& initialValues)
      : problem_(problem), initialValues_(initialValues) {}

  /** Constructor that packs the cost and constraints into a single factor
   * graph. */
  ConstrainedOptimizer(const NonlinearFactorGraph& graph,
                       const Values& initialValues)
      : problem_(graph), initialValues_(initialValues) {}

  virtual ~ConstrainedOptimizer() {}

  /// Solve a constrained optimization problem.
  virtual Values optimize() const = 0;

 protected:
  virtual bool checkConvergence(const State& state, const State& previousState,
                                const Params& params) const {
    if (state.iteration >= params.maxIterations) {
      return true;
    }

    if (state.violation() < params.absoluteViolationTolerance &&
        state.cost < params.absoluteCostTolerance) {
      return true;
    }

    if (abs(state.violation() - previousState.violation()) <
            params.relativeViolationTolerance &&
        abs(state.cost - previousState.cost) < params.relativeCostTolerance) {
      return true;
    }

    return false;
  }
};

}  // namespace gtsam
