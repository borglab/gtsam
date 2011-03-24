/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SequentialSolver.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 19, 2010
 */

#include <gtsam/base/timing.h>
#include <gtsam/linear/GaussianSequentialSolver.h>

#include <gtsam/inference/GenericSequentialSolver-inl.h>

#include <boost/numeric/ublas/matrix.hpp>

namespace ublas = boost::numeric::ublas;

namespace gtsam {

/* ************************************************************************* */
GaussianSequentialSolver::GaussianSequentialSolver(const FactorGraph<GaussianFactor>& factorGraph) :
    Base(factorGraph) {}

/* ************************************************************************* */
GaussianSequentialSolver::GaussianSequentialSolver(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph,
    const VariableIndex::shared_ptr& variableIndex) :
    Base(factorGraph, variableIndex) {}

/* ************************************************************************* */
GaussianSequentialSolver::shared_ptr GaussianSequentialSolver::Create(
    const FactorGraph<GaussianFactor>::shared_ptr& factorGraph, const VariableIndex::shared_ptr& variableIndex) {
  return shared_ptr(new GaussianSequentialSolver(factorGraph, variableIndex));
}

/* ************************************************************************* */
void GaussianSequentialSolver::replaceFactors(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph) {
  Base::replaceFactors(factorGraph);
}

/* ************************************************************************* */
GaussianBayesNet::shared_ptr GaussianSequentialSolver::eliminate() const {
  return Base::eliminate(&EliminateQR);
}

/* ************************************************************************* */
VectorValues::shared_ptr GaussianSequentialSolver::optimize() const {

  static const bool debug = false;

  if(debug) this->factors_->print("GaussianSequentialSolver, eliminating ");
  if(debug) this->eliminationTree_->print("GaussianSequentialSolver, elimination tree ");

  tic(1,"eliminate");
  // Eliminate using the elimination tree
  GaussianBayesNet::shared_ptr bayesNet(this->eliminate());
  toc(1,"eliminate");

  if(debug) bayesNet->print("GaussianSequentialSolver, Bayes net ");

  // Allocate the solution vector if it is not already allocated
//  VectorValues::shared_ptr solution = allocateVectorValues(*bayesNet);

  tic(2,"optimize");
  // Back-substitute
  VectorValues::shared_ptr solution(gtsam::optimize_(*bayesNet));
  toc(2,"optimize");

  if(debug) solution->print("GaussianSequentialSolver, solution ");

  return solution;
}

/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianSequentialSolver::marginalFactor(Index j) const {
  return Base::marginalFactor(j,&EliminateQR);
}

/* ************************************************************************* */
std::pair<Vector, Matrix> GaussianSequentialSolver::marginalCovariance(Index j) const {
	FactorGraph<GaussianFactor> fg;
	fg.push_back(Base::marginalFactor(j, &EliminateQR));
	GaussianConditional::shared_ptr conditional = EliminateQR(fg, 1).first->front();
	Matrix R = conditional->get_R();
	return make_pair(conditional->get_d(), inverse(ublas::prod(ublas::trans(R),R)));
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr 
GaussianSequentialSolver::jointFactorGraph(const std::vector<Index>& js) const {
	return GaussianFactorGraph::shared_ptr(new GaussianFactorGraph(
			*Base::jointFactorGraph(js, &EliminateQR)));
}

}
