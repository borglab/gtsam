/*
 * QPSolver.cpp
 * @brief:
 * @date: Apr 15, 2014
 * @author: thduynguyen
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/linear/LPSolver.h>

#include <boost/range/adaptor/map.hpp>

using namespace std;

namespace gtsam {

//******************************************************************************
QPSolver::QPSolver(const QP& qp) : qp_(qp) {
  baseGraph_ = qp_.cost;
  baseGraph_.push_back(qp_.equalities.begin(), qp_.equalities.end());
  costVariableIndex_ = VariableIndex(qp_.cost);
  equalityVariableIndex_ = VariableIndex(qp_.equalities);
  inequalityVariableIndex_ = VariableIndex(qp_.inequalities);
  constrainedKeys_ = qp_.equalities.keys();
  constrainedKeys_.merge(qp_.inequalities.keys());
}

//******************************************************************************
VectorValues QPSolver::solveWithCurrentWorkingSet(
    const LinearInequalityFactorGraph& workingSet) const {
  GaussianFactorGraph workingGraph = baseGraph_;
  BOOST_FOREACH(const LinearInequality::shared_ptr& factor, workingSet) {
    if (factor->active())
      workingGraph.push_back(factor);
  }
  return workingGraph.optimize();
}

//******************************************************************************
JacobianFactor::shared_ptr QPSolver::createDualFactor(Key key,
    const LinearInequalityFactorGraph& workingSet, const VectorValues& delta) const {

  // Transpose the A matrix of constrained factors to have the jacobian of the dual key
  std::vector<std::pair<Key, Matrix> > Aterms = collectDualJacobians
      < LinearEquality > (key, qp_.equalities, equalityVariableIndex_);
  std::vector<std::pair<Key, Matrix> > AtermsInequalities = collectDualJacobians
      < LinearInequality > (key, workingSet, inequalityVariableIndex_);
  Aterms.insert(Aterms.end(), AtermsInequalities.begin(),
      AtermsInequalities.end());

  // Collect the gradients of unconstrained cost factors to the b vector
  if (Aterms.size() > 0) {
    Vector b = zero(delta.at(key).size());
    if (costVariableIndex_.find(key) != costVariableIndex_.end()) {
      BOOST_FOREACH(size_t factorIx, costVariableIndex_[key]) {
        GaussianFactor::shared_ptr factor = qp_.cost.at(factorIx);
        b += factor->gradient(key, delta);
      }
    }
    return boost::make_shared<JacobianFactor>(Aterms, b);   // compute the least-square approximation of dual variables
  }
  else {
    return boost::make_shared<JacobianFactor>();
  }
}

//******************************************************************************
GaussianFactorGraph::shared_ptr QPSolver::buildDualGraph(
    const LinearInequalityFactorGraph& workingSet, const VectorValues& delta) const {
  GaussianFactorGraph::shared_ptr dualGraph(new GaussianFactorGraph());
  BOOST_FOREACH(Key key, constrainedKeys_) {
    // Each constrained key becomes a factor in the dual graph
    JacobianFactor::shared_ptr dualFactor = createDualFactor(key, workingSet, delta);
    if (!dualFactor->empty())
      dualGraph->push_back(dualFactor);
  }
  return dualGraph;
}

//******************************************************************************
int QPSolver::identifyLeavingConstraint(
    const LinearInequalityFactorGraph& workingSet,
    const VectorValues& lambdas) const {
  int worstFactorIx = -1;
  // preset the maxLambda to 0.0: if lambda is <= 0.0, the constraint is either
  // inactive or a good inequality constraint, so we don't care!
  double maxLambda = 0.0;
  for (size_t factorIx = 0; factorIx < workingSet.size(); ++factorIx) {
    const LinearInequality::shared_ptr& factor = workingSet.at(factorIx);
    if (factor->active()) {
      double lambda = lambdas.at(factor->dualKey())[0];
      if (lambda > maxLambda) {
        worstFactorIx = factorIx;
        maxLambda = lambda;
      }
    }
  }
  return worstFactorIx;
}

//******************************************************************************
/* We have to make sure the new solution with alpha satisfies all INACTIVE inequality constraints
 * If some inactive inequality constraints complain about the full step (alpha = 1),
 * we have to adjust alpha to stay within the inequality constraints' feasible regions.
 *
 * For each inactive inequality j:
 *  - We already have: aj'*xk - bj <= 0, since xk satisfies all inequality constraints
 *  - We want: aj'*(xk + alpha*p) - bj <= 0
 *  - If aj'*p <= 0, we have: aj'*(xk + alpha*p) <= aj'*xk <= bj, for all alpha>0
 *  it's good!
 *  - We only care when aj'*p > 0. In this case, we need to choose alpha so that
 *  aj'*xk + alpha*aj'*p - bj <= 0  --> alpha <= (bj - aj'*xk) / (aj'*p)
 *  We want to step as far as possible, so we should choose alpha = (bj - aj'*xk) / (aj'*p)
 *
 * We want the minimum of all those alphas among all inactive inequality.
 */
boost::tuple<double, int> QPSolver::computeStepSize(
    const LinearInequalityFactorGraph& workingSet, const VectorValues& xk,
    const VectorValues& p) const {
  static bool debug = false;

  double minAlpha = 1.0;
  int closestFactorIx = -1;
  for(size_t factorIx = 0; factorIx<workingSet.size(); ++factorIx) {
    const LinearInequality::shared_ptr& factor = workingSet.at(factorIx);
    double b = factor->getb()[0];
    // only check inactive factors
    if (!factor->active()) {
      // Compute a'*p
      double aTp = factor->dotProductRow(p);

      // Check if  a'*p >0. Don't care if it's not.
      if (aTp <= 0)
        continue;

      // Compute a'*xk
      double aTx = factor->dotProductRow(xk);

      // alpha = (b - a'*xk) / (a'*p)
      double alpha = (b - aTx) / aTp;
      if (debug)
        cout << "alpha: " << alpha << endl;

      // We want the minimum of all those max alphas
      if (alpha < minAlpha) {
        closestFactorIx = factorIx;
        minAlpha = alpha;
      }
    }

  }

  return boost::make_tuple(minAlpha, closestFactorIx);
}

//******************************************************************************
QPState QPSolver::iterate(const QPState& state) const {
  static bool debug = false;

  // Algorithm 16.3 from Nocedal06book.
  // Solve with the current working set eqn 16.39, but instead of solving for p solve for x
  VectorValues newValues = solveWithCurrentWorkingSet(state.workingSet);
  if (debug)
    newValues.print("New solution:");

  // If we CAN'T move further
  // if p_k = 0 is the original condition, modified by Duy to say that the state update is zero.
  if (newValues.equals(state.values, 1e-7)) {
    // Compute lambda from the dual graph
    if (debug)
      cout << "Building dual graph..." << endl;
    GaussianFactorGraph::shared_ptr dualGraph = buildDualGraph(state.workingSet, newValues);
    if (debug)
      dualGraph->print("Dual graph: ");
    VectorValues duals = dualGraph->optimize();
    if (debug)
      duals.print("Duals :");

    int leavingFactor = identifyLeavingConstraint(state.workingSet, duals);
    if (debug)
      cout << "leavingFactor: " << leavingFactor << endl;

    // If all inequality constraints are satisfied: We have the solution!!
    if (leavingFactor < 0) {
      return QPState(newValues, duals, state.workingSet, true, state.iterations+1);
    }
    else {
      // Inactivate the leaving constraint
      LinearInequalityFactorGraph newWorkingSet = state.workingSet;
      newWorkingSet.at(leavingFactor)->inactivate();
      return QPState(newValues, duals, newWorkingSet, false, state.iterations+1);
    }
  }
  else {
    // If we CAN make some progress, i.e. p_k != 0
    // Adapt stepsize if some inactive constraints complain about this move
    double alpha;
    int factorIx;
    VectorValues p = newValues - state.values;
    boost::tie(alpha, factorIx) = // using 16.41
        computeStepSize(state.workingSet, state.values, p);
    if (debug)
      cout << "alpha, factorIx: " << alpha << " " << factorIx << " "
           << endl;

    // also add to the working set the one that complains the most
    LinearInequalityFactorGraph newWorkingSet = state.workingSet;
    if (factorIx >= 0)
      newWorkingSet.at(factorIx)->activate();

    // step!
    newValues = state.values + alpha * p;

    return QPState(newValues, state.duals, newWorkingSet, false, state.iterations+1);
  }
}

//******************************************************************************
LinearInequalityFactorGraph QPSolver::identifyActiveConstraints(
    const LinearInequalityFactorGraph& inequalities,
    const VectorValues& initialValues, const VectorValues& duals, bool useWarmStart) const {
  LinearInequalityFactorGraph workingSet;
  BOOST_FOREACH(const LinearInequality::shared_ptr& factor, inequalities) {
    LinearInequality::shared_ptr workingFactor(new LinearInequality(*factor));
    if (useWarmStart == true && duals.exists(workingFactor->dualKey())) {
      workingFactor->activate();
    }
    else {
      if (useWarmStart == true && duals.size() > 0) {
        workingFactor->inactivate();
      } else {
        double error = workingFactor->error(initialValues);
        if (fabs(error)<1e-7) {
          workingFactor->activate();
        }
        else {
          workingFactor->inactivate();
        }
      }
    }
    workingSet.push_back(workingFactor);
  }
  return workingSet;
}

//******************************************************************************
pair<VectorValues, VectorValues> QPSolver::optimize(
    const VectorValues& initialValues, const VectorValues& duals, bool useWarmStart) const {

  // Initialize workingSet from the feasible initialValues
  LinearInequalityFactorGraph workingSet =
      identifyActiveConstraints(qp_.inequalities, initialValues, duals, useWarmStart);
  QPState state(initialValues, duals, workingSet, false, 0);

  /// main loop of the solver
  while (!state.converged) {
    state = iterate(state);
  }
  std::cout << "Final Number of inner iterations: " << state.iterations << std::endl;
  return make_pair(state.values, state.duals);
}

//******************************************************************************
std::pair<bool, Key> QPSolver::maxKey(const FastSet<Key>& keys) const {
  KeySet::iterator maxEl = std::max_element(keys.begin(), keys.end());
  if (maxEl==keys.end())
    return make_pair(false, 0);
  return make_pair(true, *maxEl);
}

//******************************************************************************
boost::tuple<VectorValues, Key, Key> QPSolver::initialValuesLP() const {
  // Key for the first slack variable =  maximum key + 1
  size_t firstSlackKey;
  bool found;
  KeySet allKeys = qp_.cost.keys();
  allKeys.merge(qp_.equalities.keys());
  allKeys.merge(qp_.inequalities.keys());
  boost::tie(found, firstSlackKey) = maxKey(allKeys);
  firstSlackKey += 1;

  VectorValues initialValues;
  // Create zero values for constrained vars
  BOOST_FOREACH(const LinearEquality::shared_ptr& factor, qp_.equalities) {
    KeyVector keys = factor->keys();
    BOOST_FOREACH(Key key, keys) {
      if (!initialValues.exists(key)) {
        size_t dim = factor->getDim(factor->find(key));
        initialValues.insert(key, zero(dim));
      }
    }
  }

  BOOST_FOREACH(const LinearInequality::shared_ptr& factor, qp_.inequalities) {
    KeyVector keys = factor->keys();
    BOOST_FOREACH(Key key, keys) {
      if (!initialValues.exists(key)) {
        size_t dim = factor->getDim(factor->find(key));
        initialValues.insert(key, zero(dim));
      }
    }
  }

  // Insert initial values for slack variables
  Key slackKey = firstSlackKey;
  // Equality: zi = |bi|
  BOOST_FOREACH(const LinearEquality::shared_ptr& factor, qp_.equalities) {
    Vector errorAtZero = factor->getb();
    Vector slackInit = errorAtZero.cwiseAbs();
    initialValues.insert(slackKey, slackInit);
    slackKey++;
  }
  // Inequality: zi = max(bi, 0)
  BOOST_FOREACH(const LinearInequality::shared_ptr& factor, qp_.inequalities) {
    Vector errorAtZero = factor->getb();
    Vector zeroVec = zero(errorAtZero.size());
    Vector slackInit = errorAtZero.cwiseMax(zeroVec);
    initialValues.insert(slackKey, slackInit);
    slackKey++;
  }

  return boost::make_tuple(initialValues, firstSlackKey, slackKey - 1);
}

//******************************************************************************
VectorValues QPSolver::objectiveCoeffsLP(Key firstSlackKey) const {
  VectorValues slackObjective;

  Key slackKey = firstSlackKey;
  // Equalities
  BOOST_FOREACH(const LinearEquality::shared_ptr& factor, qp_.equalities) {
    size_t dim = factor->rows();
    slackObjective.insert(slackKey, ones(dim));
    slackKey++;
  }

  // Inequalities
  BOOST_FOREACH(const LinearInequality::shared_ptr& factor, qp_.inequalities) {
    size_t dim = factor->rows();
    slackObjective.insert(slackKey, ones(dim));
    slackKey++;
  }

  return slackObjective;
}

//******************************************************************************
boost::tuple<LinearEqualityFactorGraph::shared_ptr,
    LinearInequalityFactorGraph::shared_ptr, VectorValues> QPSolver::constraintsLP(
    Key firstSlackKey) const {
  // Create constraints and zero lower bounds (zi>=0)
  LinearEqualityFactorGraph::shared_ptr equalities(new LinearEqualityFactorGraph());
  LinearInequalityFactorGraph::shared_ptr inequalities(new LinearInequalityFactorGraph());
  VectorValues slackLowerBounds;

  Key slackKey = firstSlackKey;

  // Equalities
  BOOST_FOREACH(const LinearEquality::shared_ptr& factor, qp_.equalities) {
    // Collect old terms to form a new factor
    // TODO: it might be faster if we can get the whole block matrix at once
    // but I don't know how to extend the current VerticalBlockMatrix
    vector<pair<Key, Matrix> > terms;
    for (Factor::iterator it = factor->begin(); it != factor->end(); ++it) {
      terms.push_back(make_pair(*it, factor->getA(it)));
    }

    Vector b = factor->getb();
    Vector sign_b = b.cwiseQuotient(b.cwiseAbs());
    terms.push_back(make_pair(slackKey, sign_b));
    equalities->push_back(LinearEquality(terms, b, factor->dualKey()));

    // Add lower bound for this slack key
    slackLowerBounds.insert(slackKey, zero(b.rows()));
    // Increase slackKey for the next slack variable
    slackKey++;
  }

  // Inequalities
  BOOST_FOREACH(const LinearInequality::shared_ptr& factor, qp_.inequalities) {
    // Collect old terms to form a new factor
    // TODO: it might be faster if we can get the whole block matrix at once
    // but I don't know how to extend the current VerticalBlockMatrix
    vector<pair<Key, Matrix> > terms;
    for (Factor::iterator it = factor->begin(); it != factor->end(); ++it) {
      terms.push_back(make_pair(*it, factor->getA(it)));
    }

    // Add the slack term to the constraint
    // Unlike Nocedal06book, pg.473, we want ax-z <= b, since we always assume
    // LE constraints ax <= b.
    terms.push_back(make_pair(slackKey, -eye(1)));
    inequalities->push_back(LinearInequality(terms, factor->getb()[0],
        factor->dualKey()));

    // Add lower bound for this slack key
    slackLowerBounds.insert(slackKey, zero(1));
    // Increase slackKey for the next slack variable
    slackKey++;
  }

  return boost::make_tuple(equalities, inequalities, slackLowerBounds);
}

//******************************************************************************
pair<bool, VectorValues> QPSolver::findFeasibleInitialValues() const {
  static const bool debug = false;
  // Initial values with slack variables for the LP subproblem, Nocedal06book, pg.473
  VectorValues initialValues;
  size_t firstSlackKey, lastSlackKey;
  boost::tie(initialValues, firstSlackKey, lastSlackKey) = initialValuesLP();

  // Coefficients for the LP subproblem objective function, min \sum_i z_i
  VectorValues objectiveLP = objectiveCoeffsLP(firstSlackKey);

  // Create constraints and lower bounds of slack variables
  LinearEqualityFactorGraph::shared_ptr equalities;
  LinearInequalityFactorGraph::shared_ptr inequalities;
  VectorValues slackLowerBounds;
  boost::tie(equalities, inequalities, slackLowerBounds) = constraintsLP(firstSlackKey);

  // Solve the LP subproblem
  LPSolver lpSolver(objectiveLP, equalities, inequalities, slackLowerBounds);
  VectorValues solution = lpSolver.solve();

  if (debug)
    initialValues.print("Initials LP: ");
  if (debug)
    objectiveLP.print("Objective LP: ");
  if (debug)
    equalities->print("Equalities LP: ");
  if (debug)
    inequalities->print("Inequalities LP: ");
  if (debug)
    solution.print("LP solution: ");

  // feasible when all slack values are 0s.
  double slackSumAbs = 0.0;
  for (Key key = firstSlackKey; key <= lastSlackKey; ++key) {
    slackSumAbs += solution.at(key).cwiseAbs().sum();
  }

  // Remove slack variables from solution
  for (Key key = firstSlackKey; key <= lastSlackKey; ++key) {
    solution.erase(key);
  }

  // Insert zero vectors for free variables that are not in the constraints
  BOOST_FOREACH(Key key, costVariableIndex_ | boost::adaptors::map_keys) {
    if (!solution.exists(key)) {
      GaussianFactor::shared_ptr factor = qp_.cost.at(
          *costVariableIndex_[key].begin());
      size_t dim = factor->getDim(factor->find(key));
      solution.insert(key, zero(dim));
    }
  }

  return make_pair(slackSumAbs < 1e-5, solution);
}

//******************************************************************************
pair<VectorValues, VectorValues> QPSolver::optimize() const {
  bool isFeasible;
  VectorValues initialValues;
  boost::tie(isFeasible, initialValues) = findFeasibleInitialValues();
  BOOST_FOREACH(const VectorValues::KeyValuePair& key_delta, initialValues) {
    double maxDelta = key_delta.second.lpNorm<Eigen::Infinity>();
    if(maxDelta >= 1e-2)
      std::cout << "LP returned non-zero initialization: " << key_delta.second << std::endl;
  }

  if (!isFeasible) {
    throw runtime_error("LP subproblem is infeasible!");
  }
  std::cout << "Completed LPSolve initialization" << std::endl;
  return optimize(initialValues);
}

} /* namespace gtsam */
