/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QPSolver.cpp
 * @brief   Compatibility wrapper for constrained QP solving.
 * @date    Apr 15, 2014
 * @author  Duy-Nguyen Ta
 */

#include <gtsam/config.h>

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43

#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam_unstable/linear/ConstrainedSolverAdapter.h>
#include <gtsam_unstable/linear/InfeasibleInitialValues.h>
#include <gtsam_unstable/linear/InfeasibleOrUnboundedProblem.h>
#include <gtsam_unstable/linear/QPSolver.h>

#include <stdexcept>
#include <tuple>

namespace gtsam {

/* ************************************************************************* */
QPSolver::QPSolver(const QP& qp) : qp_(qp) {}

/* ************************************************************************* */
std::pair<VectorValues, VectorValues> QPSolver::optimize(
    const VectorValues& initialValues, const VectorValues& duals,
    bool useWarmStart) const {
  using namespace unstable_solver_adapter;

  const QpProblem problem = ToConstrainedProblem(qp_);
  const auto keyDims = CollectKeyDims(qp_);
  const Values initial = ToValues(initialValues);
  // The legacy overload requires a feasible initial point. Automatic may select
  // Dual, which deliberately accepts infeasible starts, so retain Primal here.
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  ActiveSetSolver solver(problem, params);

  try {
    Values values;
    ActiveSetSolver::State state;
    if (useWarmStart) {
      const auto warmStart = WarmStartState(qp_.inequalities, duals);
      std::tie(values, state) = solver.optimizeWithState(initial, warmStart);
    } else {
      std::tie(values, state) = solver.optimizeWithState(initial);
    }
    return {ToVectorValues(values, keyDims), ToLegacyDuals(qp_, state)};
  } catch (const std::invalid_argument&) {
    throw InfeasibleInitialValues();
  }
}

/* ************************************************************************* */
std::pair<VectorValues, VectorValues> QPSolver::optimize() const {
  using namespace unstable_solver_adapter;

  const QpProblem problem = ToConstrainedProblem(qp_);
  const auto keyDims = CollectKeyDims(qp_);
  ActiveSetSolver solver(problem);

  try {
    const auto [values, state] = solver.optimizeWithState();
    return {ToVectorValues(values, keyDims), ToLegacyDuals(qp_, state)};
  } catch (const std::invalid_argument&) {
    throw InfeasibleOrUnboundedProblem();
  } catch (const std::runtime_error&) {
    throw InfeasibleOrUnboundedProblem();
  }
}

}  // namespace gtsam

#endif  // GTSAM_ALLOW_DEPRECATED_SINCE_V43
