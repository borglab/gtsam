/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianMultifrontalSolver.cpp
 * @author  Richard Roberts
 * @date    Oct 21, 2010
 */

#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <gtsam/linear/GaussianMultifrontalSolver.h>

namespace gtsam {

/* ************************************************************************* */
GaussianMultifrontalSolver::GaussianMultifrontalSolver(const FactorGraph<GaussianFactor>& factorGraph, bool useQR) :
    Base(factorGraph), useQR_(useQR) {}

/* ************************************************************************* */
GaussianMultifrontalSolver::GaussianMultifrontalSolver(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph,
		const VariableIndex::shared_ptr& variableIndex, bool useQR) :
    Base(factorGraph, variableIndex), useQR_(useQR) {}

/* ************************************************************************* */
GaussianMultifrontalSolver::shared_ptr
GaussianMultifrontalSolver::Create(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph,
    const VariableIndex::shared_ptr& variableIndex, bool useQR) {
  return shared_ptr(new GaussianMultifrontalSolver(factorGraph, variableIndex, useQR));
}

/* ************************************************************************* */
void GaussianMultifrontalSolver::replaceFactors(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph) {
  Base::replaceFactors(factorGraph);
}

/* ************************************************************************* */
GaussianBayesTree::shared_ptr GaussianMultifrontalSolver::eliminate() const {
	if (useQR_)
		return Base::eliminate(&EliminateQR);
	else
		return Base::eliminate(&EliminatePreferCholesky);
}

/* ************************************************************************* */
VectorValues::shared_ptr GaussianMultifrontalSolver::optimize() const {
  tic(2,"optimize");
  VectorValues::shared_ptr values;
  if (useQR_)
  	values = VectorValues::shared_ptr(new VectorValues(junctionTree_->optimize(&EliminateQR)));
  else
  	values= VectorValues::shared_ptr(new VectorValues(junctionTree_->optimize(&EliminatePreferCholesky)));
  toc(2,"optimize");
  return values;
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr GaussianMultifrontalSolver::jointFactorGraph(const std::vector<Index>& js) const {
	if (useQR_)
		return GaussianFactorGraph::shared_ptr(new GaussianFactorGraph(*Base::jointFactorGraph(js,&EliminateQR)));
	else
		return GaussianFactorGraph::shared_ptr(new GaussianFactorGraph(*Base::jointFactorGraph(js,&EliminatePreferCholesky)));
}

/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianMultifrontalSolver::marginalFactor(Index j) const {
	if (useQR_)
		return Base::marginalFactor(j,&EliminateQR);
	else
		return Base::marginalFactor(j,&EliminatePreferCholesky);
}

/* ************************************************************************* */
Matrix GaussianMultifrontalSolver::marginalCovariance(Index j) const {
  FactorGraph<GaussianFactor> fg;
  GaussianConditional::shared_ptr conditional;
  if (useQR_) {
  	fg.push_back(Base::marginalFactor(j,&EliminateQR));
  	conditional = EliminateQR(fg,1).first;
  } else {
  	fg.push_back(Base::marginalFactor(j,&EliminatePreferCholesky));
  	conditional = EliminatePreferCholesky(fg,1).first;
  }
  return conditional->computeInformation().inverse();
}

}
