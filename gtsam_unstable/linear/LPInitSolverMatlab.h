/**
 * @file     LPInitSolverMatlab.h
 * @brief    This LPInitSolver implements the strategy in Matlab:
 * @author   Ivan Dario Jimenez
 * @date     1/24/16
 */

#pragma once

#include <gtsam_unstable/linear/LPInitSolver.h>
#include <gtsam_unstable/linear/InfeasibleOrUnboundedProblem.h>
#include <gtsam_unstable/linear/QPSolver.h>
#include <CppUnitLite/Test.h>

namespace gtsam {
/**
 * This LPInitSolver implements the strategy in Matlab:
 * http://www.mathworks.com/help/optim/ug/linear-programming-algorithms.html#brozyzb-9
 * Solve for x and y:
 *    min y
 *    st Ax = b
 *       Cx - y <= d
 * where y \in R, x \in R^n, and Ax = b and Cx <= d is the constraints of the original problem.
 *
 * If the solution for this problem {x*,y*} has y* <= 0, we'll have x* a feasible initial point
 * of the original problem
 * otherwise, if y* > 0 or the problem has no solution, the original problem is infeasible.
 *
 * The initial value of this initial problem can be found by solving
 *    min   ||x||^2
 *    s.t.   Ax = b
 * to have a solution x0
 * then y = max_j ( Cj*x0  - dj )  -- due to the constraints y >= Cx - d
 *
 * WARNING: If some xj in the inequality constraints does not exist in the equality constraints,
 * set them as zero for now. If that is the case, the original problem doesn't have a unique
 * solution (it could be either infeasible or unbounded).
 * So, if the initialization fails because we enforce xj=0 in the problematic
 * inequality constraint, we can't conclude that the problem is infeasible.
 * However, whether it is infeasible or unbounded, we don't have a unique solution anyway.
 */
class LPInitSolverMatlab: public LPInitSolver {
  typedef LPInitSolver Base;
public:
  LPInitSolverMatlab(const LPSolver& lpSolver) :
      Base(lpSolver) {
  }
  virtual ~LPInitSolverMatlab() {
  }

  virtual VectorValues solve() const {
    // Build the graph to solve for the initial value of the initial problem
    GaussianFactorGraph::shared_ptr initOfInitGraph = buildInitOfInitGraph();
    VectorValues x0 = initOfInitGraph->optimize();
    double y0 = compute_y0(x0);
    Key yKey = maxKey(lpSolver_.keysDim()) + 1; // the unique key for y0
    VectorValues xy0(x0);
    xy0.insert(yKey, Vector::Constant(1, y0));

    // Formulate and solve the initial LP
    LP::shared_ptr initLP = buildInitialLP(yKey);

    // solve the initialLP
    LPSolver lpSolveInit(*initLP);
    VectorValues xyInit = lpSolveInit.optimize(xy0).first;
    double yOpt = xyInit.at(yKey)[0];
    xyInit.erase(yKey);
    if (yOpt > 0)
      throw InfeasibleOrUnboundedProblem();
    else
      return xyInit;
  }

private:
  /// build initial LP
  LP::shared_ptr buildInitialLP(Key yKey) const {
    LP::shared_ptr initLP(new LP());
    initLP->cost = LinearCost(yKey, ones(1)); // min y
    initLP->equalities = lp_.equalities; // st. Ax = b
    initLP->inequalities = addSlackVariableToInequalities(yKey,
        lp_.inequalities); // Cx-y <= d
    return initLP;
  }

  /// Find the max key in the problem to determine unique keys for additional slack variables
  Key maxKey(const KeyDimMap& keysDim) const {
    Key maxK = 0;
    BOOST_FOREACH(Key key, keysDim | boost::adaptors::map_keys)
    if (maxK < key)
    maxK = key;
    return maxK;
  }

  /**
   * Build the following graph to solve for an initial value of the initial problem
   *    min   ||x||^2    s.t.   Ax = b
   */
  GaussianFactorGraph::shared_ptr buildInitOfInitGraph() const {
    // first add equality constraints Ax = b
    GaussianFactorGraph::shared_ptr initGraph(
        new GaussianFactorGraph(lp_.equalities));

    // create factor ||x||^2 and add to the graph
    const KeyDimMap& keysDim = lpSolver_.keysDim();
    BOOST_FOREACH(Key key, keysDim | boost::adaptors::map_keys) {
      size_t dim = keysDim.at(key);
      initGraph->push_back(JacobianFactor(key, eye(dim), zero(dim)));
    }
    return initGraph;
  }

  /// y = max_j ( Cj*x0  - dj )  -- due to the inequality constraints y >= Cx - d
  double compute_y0(const VectorValues& x0) const {
    double y0 = -std::numeric_limits<double>::infinity();
    BOOST_FOREACH(const LinearInequality::shared_ptr& factor, lp_.inequalities) {
      double error = factor->error(x0);
      if (error > y0)
      y0 = error;
    }
    return y0;
  }

  /// Collect all terms of a factor into a container.
  std::vector<std::pair<Key, Matrix> > collectTerms(
      const LinearInequality& factor) const {
    std::vector < std::pair<Key, Matrix> > terms;
    for (Factor::const_iterator it = factor.begin(); it != factor.end(); it++) {
      terms.push_back(make_pair(*it, factor.getA(it)));
    }
    return terms;
  }

  /// Turn Cx <= d into Cx - y <= d factors
  InequalityFactorGraph addSlackVariableToInequalities(Key yKey,
      const InequalityFactorGraph& inequalities) const {
    InequalityFactorGraph slackInequalities;
    BOOST_FOREACH(const LinearInequality::shared_ptr& factor, lp_.inequalities) {
      std::vector<std::pair<Key, Matrix> > terms = collectTerms(*factor); // Cx
      terms.push_back(make_pair(yKey, Matrix::Constant(1, 1, -1.0)));// -y
      double d = factor->getb()[0];
      slackInequalities.push_back(LinearInequality(terms, d, factor->dualKey()));
    }
    return slackInequalities;
  }

  // friend class for unit-testing private methods
  FRIEND_TEST(LPInitSolverMatlab, initialization);
};
}
