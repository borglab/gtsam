/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianSequentialSolver.cpp
 * @author  Richard Roberts
 * @date    Oct 19, 2010
 */

#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <gtsam/base/timing.h>
#include <gtsam/linear/GaussianSequentialSolver.h>

namespace gtsam {

/* ************************************************************************* */
GaussianSequentialSolver::GaussianSequentialSolver(
		const FactorGraph<GaussianFactor>& factorGraph, bool useQR) :
		Base(factorGraph), useQR_(useQR) {
}

/* ************************************************************************* */
GaussianSequentialSolver::GaussianSequentialSolver(
		const FactorGraph<GaussianFactor>::shared_ptr& factorGraph,
		const VariableIndex::shared_ptr& variableIndex, bool useQR) :
		Base(factorGraph, variableIndex), useQR_(useQR) {
}

/* ************************************************************************* */
GaussianSequentialSolver::shared_ptr GaussianSequentialSolver::Create(
		const FactorGraph<GaussianFactor>::shared_ptr& factorGraph,
		const VariableIndex::shared_ptr& variableIndex, bool useQR) {
	return shared_ptr(
			new GaussianSequentialSolver(factorGraph, variableIndex, useQR));
}

/* ************************************************************************* */
void GaussianSequentialSolver::replaceFactors(
		const FactorGraph<GaussianFactor>::shared_ptr& factorGraph) {
	Base::replaceFactors(factorGraph);
}

/* ************************************************************************* */
GaussianBayesNet::shared_ptr GaussianSequentialSolver::eliminate() const {
	if (useQR_)
		return Base::eliminate(&EliminateQR);
	else
		return Base::eliminate(&EliminatePreferCholesky);
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
  VectorValues::shared_ptr solution(
      new VectorValues(gtsam::optimize(*bayesNet)));
  toc(2,"optimize");

  if(debug) solution->print("GaussianSequentialSolver, solution ");

  return solution;
}

/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianSequentialSolver::marginalFactor(Index j) const {
	if (useQR_)
		return Base::marginalFactor(j,&EliminateQR);
	else
		return Base::marginalFactor(j,&EliminatePreferCholesky);
}

/* ************************************************************************* */
Matrix GaussianSequentialSolver::marginalCovariance(Index j) const {
	FactorGraph<GaussianFactor> fg;
	GaussianConditional::shared_ptr conditional;
	if (useQR_) {
		fg.push_back(Base::marginalFactor(j, &EliminateQR));
		conditional = EliminateQR(fg, 1).first;
	} else {
		fg.push_back(Base::marginalFactor(j, &EliminatePreferCholesky));
		conditional = EliminatePreferCholesky(fg, 1).first;
	}
	return conditional->computeInformation().inverse();
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr 
GaussianSequentialSolver::jointFactorGraph(const std::vector<Index>& js) const {
	if (useQR_)
		return GaussianFactorGraph::shared_ptr(new GaussianFactorGraph(
				*Base::jointFactorGraph(js, &EliminateQR)));
	else
		return GaussianFactorGraph::shared_ptr(new GaussianFactorGraph(
				*Base::jointFactorGraph(js, &EliminatePreferCholesky)));
}

} /// namespace gtsam
