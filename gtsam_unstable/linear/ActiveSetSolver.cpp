/**
 * @file     ActiveSetSolver.cpp
 * @brief    Implmentation of ActiveSetSolver.
 * @author   Ivan Dario Jimenez
 * @author   Duy Nguyen Ta
 * @date     2/11/16
 */

#include <gtsam_unstable/linear/ActiveSetSolver.h>

namespace gtsam {

/*
 * Iterates through each factor in the factor graph and checks 
 * whether it's active. If the factor is active it reutrns the A 
 * term of the factor.
 */
template<typename FACTOR>
ActiveSetSolver::TermsContainer ActiveSetSolver::collectDualJacobians(Key key,
    const FactorGraph<FACTOR>& graph,
    const VariableIndex& variableIndex) const {
  ActiveSetSolver::TermsContainer Aterms;
  if (variableIndex.find(key) != variableIndex.end()) {
  BOOST_FOREACH (size_t factorIx, variableIndex[key]) {
    typename FACTOR::shared_ptr factor = graph.at(factorIx);
    if (!factor->active()) continue;
    Matrix Ai = factor->getA(factor->find(key)).transpose();
    Aterms.push_back(std::make_pair(factor->dualKey(), Ai));
  }
}
return Aterms;
}

/*
 * The goal of this function is to find currently active inequality constraints
 * that violate the condition to be active. The one that violates the condition
 * the most will be removed from the active set. See Nocedal06book, pg 469-471
 *
 * Find the BAD active inequality that pulls x strongest to the wrong direction
 * of its constraint (i.e. it is pulling towards >0, while its feasible region is <=0)
 *
 * For active inequality constraints (those that are enforced as equality constraints
 * in the current working set), we want lambda < 0.
 * This is because:
 *   - From the Lagrangian L = f - lambda*c, we know that the constraint force
 *     is (lambda * \grad c) = \grad f. Intuitively, to keep the solution x stay
 *     on the constraint surface, the constraint force has to balance out with
 *     other unconstrained forces that are pulling x towards the unconstrained
 *     minimum point. The other unconstrained forces are pulling x toward (-\grad f),
 *     hence the constraint force has to be exactly \grad f, so that the total
 *     force is 0.
 *   - We also know that  at the constraint surface c(x)=0, \grad c points towards + (>= 0),
 *     while we are solving for - (<=0) constraint.
 *   - We want the constraint force (lambda * \grad c) to pull x towards the - (<=0) direction
 *     i.e., the opposite direction of \grad c where the inequality constraint <=0 is satisfied.
 *     That means we want lambda < 0.
 *   - This is because when the constrained force pulls x towards the infeasible region (+),
 *     the unconstrained force is pulling x towards the opposite direction into
 *     the feasible region (again because the total force has to be 0 to make x stay still)
 *     So we can drop this constraint to have a lower error but feasible solution.
 *
 * In short, active inequality constraints with lambda > 0 are BAD, because they
 * violate the condition to be active.
 *
 * And we want to remove the worst one with the largest lambda from the active set.
 *
 */
int ActiveSetSolver::identifyLeavingConstraint(
  const InequalityFactorGraph& workingSet, const VectorValues& lambdas) const {
int worstFactorIx = -1;
// preset the maxLambda to 0.0: if lambda is <= 0.0, the constraint is
// either
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

/*  This function will create a dual graph that solves for the
 *  lagrange multipliers for the current working set.
 *  You can use lagrange multipliers as a necessary condition for optimality.
 *  The factor graph that is being solved is f' = -lambda * g'
 *  where f is the optimized function and g is the function resulting from
 *  aggregating the working set.
 *  The lambdas give you information about the feasibility of a constraint.
 *  if lambda < 0  the constraint is Ok
 *  if lambda = 0  you are on the constraint
 *  if lambda > 0  you are violating the constraint.
 */
GaussianFactorGraph::shared_ptr ActiveSetSolver::buildDualGraph(
  const InequalityFactorGraph& workingSet, const VectorValues& delta) const {
GaussianFactorGraph::shared_ptr dualGraph(new GaussianFactorGraph());
BOOST_FOREACH (Key key, constrainedKeys_) {
  // Each constrained key becomes a factor in the dual graph
  JacobianFactor::shared_ptr dualFactor =
  createDualFactor(key, workingSet, delta);
  if (!dualFactor->empty()) dualGraph->push_back(dualFactor);
}
return dualGraph;
}

/*
 * Compute step size alpha for the new solution x' = xk + alpha*p, where alpha \in [0,1]
 *
 *    @return a tuple of (alpha, factorIndex, sigmaIndex) where (factorIndex, sigmaIndex)
 *            is the constraint that has minimum alpha, or (-1,-1) if alpha = 1.
 *            This constraint will be added to the working set and become active
 *            in the next iteration.
 */
boost::tuple<double, int> ActiveSetSolver::computeStepSize(
  const InequalityFactorGraph& workingSet, const VectorValues& xk,
  const VectorValues& p, const double& startAlpha) const {
double minAlpha = startAlpha;
int closestFactorIx = -1;
for (size_t factorIx = 0; factorIx < workingSet.size(); ++factorIx) {
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
    // We want the minimum of all those max alphas
    if (alpha < minAlpha) {
      closestFactorIx = factorIx;
      minAlpha = alpha;
    }
  }
}
return boost::make_tuple(minAlpha, closestFactorIx);
}

}
