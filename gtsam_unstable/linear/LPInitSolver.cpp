/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     LPInitSolver.h
 * @brief    This finds a feasible solution for an LP problem
 * @author   Duy Nguyen Ta
 * @author   Ivan Dario Jimenez
 * @date     6/16/16
 */

#include <gtsam_unstable/linear/LPInitSolver.h>
#include <gtsam_unstable/linear/LPSolver.h>
#include <gtsam_unstable/linear/InfeasibleOrUnboundedProblem.h>

namespace gtsam {

/******************************************************************************/
VectorValues LPInitSolver::solve() const {
  // Build the graph to solve for the initial value of the initial problem
  GaussianFactorGraph::shared_ptr initOfInitGraph = buildInitOfInitGraph();
  VectorValues x0 = initOfInitGraph->optimize();
  double y0 = compute_y0(x0);
  Key yKey = maxKey(lp_) + 1;  // the unique key for y0
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

/******************************************************************************/
LP::shared_ptr LPInitSolver::buildInitialLP(Key yKey) const {
  LP::shared_ptr initLP(new LP());
  initLP->cost = LinearCost(yKey, I_1x1);  // min y
  initLP->equalities = lp_.equalities;     // st. Ax = b
  initLP->inequalities =
      addSlackVariableToInequalities(yKey,
                                     lp_.inequalities);  // Cx-y <= d
  return initLP;
}

/******************************************************************************/
GaussianFactorGraph::shared_ptr LPInitSolver::buildInitOfInitGraph() const {
  // first add equality constraints Ax = b
  GaussianFactorGraph::shared_ptr initGraph(
      new GaussianFactorGraph(lp_.equalities));

  // create factor ||x||^2 and add to the graph
  const KeyDimMap& constrainedKeyDim = lp_.constrainedKeyDimMap();
  for (const auto& [key, _] : constrainedKeyDim) {
    size_t dim = constrainedKeyDim.at(key);
    initGraph->push_back(
        JacobianFactor(key, Matrix::Identity(dim, dim), Vector::Zero(dim)));
  }
  return initGraph;
}

/******************************************************************************/
double LPInitSolver::compute_y0(const VectorValues& x0) const {
  double y0 = -std::numeric_limits<double>::infinity();
  for (const auto& factor : lp_.inequalities) {
    double error = factor->error(x0);
    if (error > y0) y0 = error;
  }
  return y0;
}

/******************************************************************************/
std::vector<std::pair<Key, Matrix> > LPInitSolver::collectTerms(
    const LinearInequality& factor) const {
  std::vector<std::pair<Key, Matrix> > terms;
  for (Factor::const_iterator it = factor.begin(); it != factor.end(); it++) {
    terms.push_back(make_pair(*it, factor.getA(it)));
  }
  return terms;
}

/******************************************************************************/
InequalityFactorGraph LPInitSolver::addSlackVariableToInequalities(
    Key yKey, const InequalityFactorGraph& inequalities) const {
  InequalityFactorGraph slackInequalities;
  for (const auto& factor : lp_.inequalities) {
    std::vector<std::pair<Key, Matrix> > terms = collectTerms(*factor);  // Cx
    terms.push_back(make_pair(yKey, Matrix::Constant(1, 1, -1.0)));      // -y
    double d = factor->getb()[0];
    slackInequalities.push_back(LinearInequality(terms, d, factor->dualKey()));
  }
  return slackInequalities;
}

}
