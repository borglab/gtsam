/**
 * @file     LPSolver.cpp
 * @brief    
 * @author   Duy Nguyen Ta
 * @author   Ivan Dario Jimenez
 * @date     1/26/16
 */

#include <gtsam_unstable/linear/LPSolver.h>
#include <gtsam_unstable/linear/InfeasibleInitialValues.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam_unstable/linear/LPInitSolverMatlab.h>

namespace gtsam {
LPSolver::LPSolver(const LP &lp) :
    lp_(lp), addedZeroPriorsIndex_(){
  // Push back factors that are the same in every iteration to the base graph.
  // Those include the equality constraints and zero priors for keys that are
  // not in the cost
  baseGraph_.push_back(lp_.equalities);

  // Collect key-dim map of all variables in the constraints to create their
  // zero priors later
  keysDim_ = collectKeysDim(lp_.equalities);
  KeyDimMap keysDim2 = collectKeysDim(lp_.inequalities);
  keysDim_.insert(keysDim2.begin(), keysDim2.end());

  // Create and push zero priors of constrained variables that do not exist in
  // the cost function
  auto addedZeroPriorsGraph = *createZeroPriors(lp_.cost.keys(), keysDim_);
  //before we add them we fill an array with the indeces of the added zero priors
  addedZeroPriorsIndex_.resize(addedZeroPriorsGraph.size());
  std::iota(addedZeroPriorsIndex_.begin(), addedZeroPriorsIndex_.end(), baseGraph_.size());
  baseGraph_.push_back(addedZeroPriorsGraph);

  // Variable index
  equalityVariableIndex_ = VariableIndex(lp_.equalities);
  inequalityVariableIndex_ = VariableIndex(lp_.inequalities);
  constrainedKeys_ = lp_.equalities.keys();
  constrainedKeys_.merge(lp_.inequalities.keys());
}

GaussianFactorGraph::shared_ptr LPSolver::createZeroPriors(
    const KeyVector &costKeys, const KeyDimMap &keysDim) const {
  GaussianFactorGraph::shared_ptr graph(new GaussianFactorGraph());
  for (Key key : keysDim | boost::adaptors::map_keys) {
    if (find(costKeys.begin(), costKeys.end(), key) == costKeys.end()) {
      size_t dim = keysDim.at(key);
      graph->push_back(JacobianFactor(key, eye(dim), zero(dim)));
    }
  }
  return graph;
}

LPState LPSolver::iterate(const LPState &state) const {
  // Solve with the current working set
  // LP: project the objective neg. gradient to the constraint's null space
  // to find the direction to move
  VectorValues newValues = solveWithCurrentWorkingSet(state.values,
      state.workingSet);
  // If we CAN'T move further
  // LP: projection on the constraints' nullspace is zero: we are at a vertex
  if (newValues.equals(state.values, 1e-7)) {
    // Find and remove the bad inequality constraint by computing its lambda
    // Compute lambda from the dual graph
    // LP: project the objective's gradient onto each constraint gradient to
    // obtain the dual scaling factors
    //	is it true??
    GaussianFactorGraph::shared_ptr dualGraph = buildDualGraph(state.workingSet,
        newValues);
    VectorValues duals = dualGraph->optimize();
    // LP: see which inequality constraint has wrong pulling direction, i.e., dual < 0
    int leavingFactor = identifyLeavingConstraint(state.workingSet, duals);
    // If all inequality constraints are satisfied: We have the solution!!
    if (leavingFactor < 0) {
      // TODO If we still have infeasible equality constraints: the problem is
      // over-constrained. No solution!
      // ...
      return LPState(newValues, duals, state.workingSet, true,
          state.iterations + 1);
    } else {
      // Inactivate the leaving constraint
      // LP: remove the bad ineq constraint out of the working set
      InequalityFactorGraph newWorkingSet = state.workingSet;
      newWorkingSet.at(leavingFactor)->inactivate();
      return LPState(newValues, duals, newWorkingSet, false,
          state.iterations + 1);
    }
  } else {
    // If we CAN make some progress, i.e. p_k != 0
    // Adapt stepsize if some inactive constraints complain about this move
    // LP: projection on nullspace is NOT zero:
    // 		find and put a blocking inactive constraint to the working set,
    // 		otherwise the problem is unbounded!!!
    double alpha;
    int factorIx;
    VectorValues p = newValues - state.values;
//    GTSAM_PRINT(p);
    boost::tie(alpha, factorIx) = // using 16.41
        computeStepSize(state.workingSet, state.values, p);
    // also add to the working set the one that complains the most
    InequalityFactorGraph newWorkingSet = state.workingSet;
    if (factorIx >= 0)
      newWorkingSet.at(factorIx)->activate();
    // step!
    newValues = state.values + alpha * p;
    return LPState(newValues, state.duals, newWorkingSet, false,
        state.iterations + 1);
  }
}

GaussianFactorGraph::shared_ptr LPSolver::createLeastSquareFactors(
    const LinearCost &cost, const VectorValues &xk) const {
  GaussianFactorGraph::shared_ptr graph(new GaussianFactorGraph());
  for (LinearCost::const_iterator it = cost.begin(); it != cost.end(); ++it) {
    size_t dim = cost.getDim(it);
    Vector b = xk.at(*it) - cost.getA(it).transpose(); // b = xk-g
    graph->push_back(JacobianFactor(*it, eye(dim), b));
  }

  KeySet allKeys = lp_.inequalities.keys();
  allKeys.merge(lp_.equalities.keys());
  allKeys.merge(KeySet(lp_.cost.keys()));
// add the corresponding factors for all variables that are not explicitly in the cost
  //function
//   for vars that are not in the cost, the cost gradient is zero (g=0), so b=xk
  if (cost.keys().size() != allKeys.size()) {
    KeySet difference;
    std::set_difference(allKeys.begin(), allKeys.end(), lp_.cost.begin(),
                        lp_.cost.end(), std::inserter(difference, difference.end()));
    for (Key k : difference) {
      graph->push_back(JacobianFactor(k, eye(keysDim_.at(k)), xk.at(k)));
    }
  }
  return graph;
}

VectorValues LPSolver::solveWithCurrentWorkingSet(const VectorValues &xk,
    const InequalityFactorGraph &workingSet) const {
  GaussianFactorGraph workingGraph = baseGraph_; // || X - Xk + g ||^2
//  We remove the old zero priors from the base graph we are going to use to solve
  //This iteration's problem
  for (size_t index : addedZeroPriorsIndex_) {
    workingGraph.remove(index);
  }

  workingGraph.push_back(*createLeastSquareFactors(lp_.cost, xk));
  for (const LinearInequality::shared_ptr &factor : workingSet) {
    if (factor->active())
      workingGraph.push_back(factor);
  }
  return workingGraph.optimize();
}

boost::shared_ptr<JacobianFactor> LPSolver::createDualFactor(Key key,
    const InequalityFactorGraph &workingSet, const VectorValues &delta) const {
  // Transpose the A matrix of constrained factors to have the jacobian of the
  // dual key
  TermsContainer Aterms = collectDualJacobians < LinearEquality
      > (key, lp_.equalities, equalityVariableIndex_);
  TermsContainer AtermsInequalities = collectDualJacobians < LinearInequality
      > (key, workingSet, inequalityVariableIndex_);
  Aterms.insert(Aterms.end(), AtermsInequalities.begin(),
      AtermsInequalities.end());

  // Collect the gradients of unconstrained cost factors to the b vector
  if (Aterms.size() > 0) {
    Vector b = zero(delta.at(key).size());
    Factor::const_iterator it = lp_.cost.find(key);
    if (it != lp_.cost.end())
      b = lp_.cost.getA(it).transpose();
    return boost::make_shared < JacobianFactor > (Aterms, b); // compute the least-square approximation of dual variables
  } else {
    return boost::make_shared<JacobianFactor>();
  }
}

InequalityFactorGraph LPSolver::identifyActiveConstraints(
    const InequalityFactorGraph &inequalities,
    const VectorValues &initialValues, const VectorValues &duals) const {
  InequalityFactorGraph workingSet;
  for (const LinearInequality::shared_ptr &factor : inequalities) {
    LinearInequality::shared_ptr workingFactor(new LinearInequality(*factor));

    double error = workingFactor->error(initialValues);
    // TODO: find a feasible initial point for LPSolver.
    // For now, we just throw an exception
    if (error > 0)
      throw InfeasibleInitialValues();

    if (fabs(error) < 1e-7) {
      workingFactor->activate();
    } else {
      workingFactor->inactivate();
    }
    workingSet.push_back(workingFactor);
  }
  return workingSet;
}

std::pair<VectorValues, VectorValues> LPSolver::optimize(
    const VectorValues &initialValues, const VectorValues &duals) const {
  {
    // Initialize workingSet from the feasible initialValues
    InequalityFactorGraph workingSet = identifyActiveConstraints(
        lp_.inequalities, initialValues, duals);
    LPState state(initialValues, duals, workingSet, false, 0);

    /// main loop of the solver
    while (!state.converged) {
      state = iterate(state);
    }
    return make_pair(state.values, state.duals);
  }
}

boost::tuples::tuple<double, int> LPSolver::computeStepSize(
    const InequalityFactorGraph &workingSet, const VectorValues &xk,
    const VectorValues &p) const {
  return ActiveSetSolver::computeStepSize(workingSet, xk, p,
      std::numeric_limits<double>::infinity());
}

pair<VectorValues, VectorValues> LPSolver::optimize() const {
  LPInitSolverMatlab initSolver(*this);
  VectorValues initValues = initSolver.solve();
  return optimize(initValues);
}
}

