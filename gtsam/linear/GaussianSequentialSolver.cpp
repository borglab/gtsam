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

#include <gtsam/base/timing.h>
#include <gtsam/linear/GaussianSequentialSolver.h>

namespace gtsam {

/* ************************************************************************* */
GaussianSequentialSolver::GaussianSequentialSolver(
    const FactorGraphOrdered<GaussianFactorOrdered>& factorGraph, bool useQR) :
    Base(factorGraph), useQR_(useQR) {
}

/* ************************************************************************* */
GaussianSequentialSolver::GaussianSequentialSolver(
    const FactorGraphOrdered<GaussianFactorOrdered>::shared_ptr& factorGraph,
    const VariableIndexOrdered::shared_ptr& variableIndex, bool useQR) :
    Base(factorGraph, variableIndex), useQR_(useQR) {
}

/* ************************************************************************* */
GaussianSequentialSolver::shared_ptr GaussianSequentialSolver::Create(
    const FactorGraphOrdered<GaussianFactorOrdered>::shared_ptr& factorGraph,
    const VariableIndexOrdered::shared_ptr& variableIndex, bool useQR) {
  return shared_ptr(
      new GaussianSequentialSolver(factorGraph, variableIndex, useQR));
}

/* ************************************************************************* */
void GaussianSequentialSolver::replaceFactors(
    const FactorGraphOrdered<GaussianFactorOrdered>::shared_ptr& factorGraph) {
  Base::replaceFactors(factorGraph);
}

/* ************************************************************************* */
GaussianBayesNetOrdered::shared_ptr GaussianSequentialSolver::eliminate() const {
  if (useQR_)
    return Base::eliminate(&EliminateQROrdered);
  else
    return Base::eliminate(&EliminatePreferCholeskyOrdered);
}

/* ************************************************************************* */
VectorValuesOrdered::shared_ptr GaussianSequentialSolver::optimize() const {

  static const bool debug = false;

  if(debug) this->factors_->print("GaussianSequentialSolver, eliminating ");
  if(debug) this->eliminationTree_->print("GaussianSequentialSolver, elimination tree ");

  gttic(eliminate);
  // Eliminate using the elimination tree
  GaussianBayesNetOrdered::shared_ptr bayesNet(this->eliminate());
  gttoc(eliminate);

  if(debug) bayesNet->print("GaussianSequentialSolver, Bayes net ");

  // Allocate the solution vector if it is not already allocated
//  VectorValuesOrdered::shared_ptr solution = allocateVectorValues(*bayesNet);

  gttic(optimize);
  // Back-substitute
  VectorValuesOrdered::shared_ptr solution(
      new VectorValuesOrdered(gtsam::optimize(*bayesNet)));
  gttoc(optimize);

  if(debug) solution->print("GaussianSequentialSolver, solution ");

  return solution;
}

/* ************************************************************************* */
GaussianFactorOrdered::shared_ptr GaussianSequentialSolver::marginalFactor(Index j) const {
  if (useQR_)
    return Base::marginalFactor(j,&EliminateQROrdered);
  else
    return Base::marginalFactor(j,&EliminatePreferCholeskyOrdered);
}

/* ************************************************************************* */
Matrix GaussianSequentialSolver::marginalCovariance(Index j) const {
  FactorGraphOrdered<GaussianFactorOrdered> fg;
  GaussianConditionalOrdered::shared_ptr conditional;
  if (useQR_) {
    fg.push_back(Base::marginalFactor(j, &EliminateQROrdered));
    conditional = EliminateQROrdered(fg, 1).first;
  } else {
    fg.push_back(Base::marginalFactor(j, &EliminatePreferCholeskyOrdered));
    conditional = EliminatePreferCholeskyOrdered(fg, 1).first;
  }
  return conditional->information().inverse();
}

/* ************************************************************************* */
GaussianFactorGraphOrdered::shared_ptr 
GaussianSequentialSolver::jointFactorGraph(const std::vector<Index>& js) const {
  if (useQR_)
    return GaussianFactorGraphOrdered::shared_ptr(new GaussianFactorGraphOrdered(
        *Base::jointFactorGraph(js, &EliminateQROrdered)));
  else
    return GaussianFactorGraphOrdered::shared_ptr(new GaussianFactorGraphOrdered(
        *Base::jointFactorGraph(js, &EliminatePreferCholeskyOrdered)));
}

} /// namespace gtsam
