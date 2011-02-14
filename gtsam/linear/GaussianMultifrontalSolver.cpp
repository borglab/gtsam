/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianMultifrontalSolver.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#include <gtsam/linear/GaussianMultifrontalSolver.h>

#include <gtsam/inference/GenericMultifrontalSolver-inl.h>

#include <boost/numeric/ublas/matrix.hpp>

namespace ublas = boost::numeric::ublas;

namespace gtsam {

/* ************************************************************************* */
GaussianMultifrontalSolver::GaussianMultifrontalSolver(const FactorGraph<GaussianFactor>& factorGraph) :
    Base(factorGraph) {}

/* ************************************************************************* */
GaussianMultifrontalSolver::GaussianMultifrontalSolver(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph, const VariableIndex::shared_ptr& variableIndex) :
    Base(factorGraph, variableIndex) {}

/* ************************************************************************* */
GaussianMultifrontalSolver::shared_ptr
GaussianMultifrontalSolver::Create(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph,
    const VariableIndex::shared_ptr& variableIndex) {
  return shared_ptr(new GaussianMultifrontalSolver(factorGraph, variableIndex));
}

/* ************************************************************************* */
void GaussianMultifrontalSolver::replaceFactors(const FactorGraph<GaussianFactor>::shared_ptr& factorGraph) {
  Base::replaceFactors(factorGraph);
}

/* ************************************************************************* */
BayesTree<GaussianConditional>::shared_ptr GaussianMultifrontalSolver::eliminate() const {
  return Base::eliminate();
}

/* ************************************************************************* */
VectorValues::shared_ptr GaussianMultifrontalSolver::optimize() const {
  tic(2,"optimize");
  VectorValues::shared_ptr values(new VectorValues(junctionTree_->optimize()));
  toc(2,"optimize");
  return values;
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr GaussianMultifrontalSolver::jointFactorGraph(const std::vector<Index>& js) const {
  return GaussianFactorGraph::shared_ptr(new GaussianFactorGraph(*Base::jointFactorGraph(js)));
}

/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianMultifrontalSolver::marginalFactor(Index j) const {
  return Base::marginalFactor(j);
}

/* ************************************************************************* */
std::pair<Vector, Matrix> GaussianMultifrontalSolver::marginalCovariance(Index j) const {
  FactorGraph<GaussianFactor> fg;
  fg.push_back(Base::marginalFactor(j));
  GaussianConditional::shared_ptr conditional = GaussianFactor::CombineAndEliminate(fg,1).first->front();
  Matrix R = conditional->get_R();
  return make_pair(conditional->get_d(), inverse(ublas::prod(ublas::trans(R), R)));
}

}
