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

#include <gtsam/linear/GaussianMultifrontalSolver.h>

namespace gtsam {

/* ************************************************************************* */
GaussianMultifrontalSolver::GaussianMultifrontalSolver(const FactorGraphOrdered<GaussianFactorOrdered>& factorGraph, bool useQR) :
    Base(factorGraph), useQR_(useQR) {}

/* ************************************************************************* */
GaussianMultifrontalSolver::GaussianMultifrontalSolver(const FactorGraphOrdered<GaussianFactorOrdered>::shared_ptr& factorGraph,
    const VariableIndexOrdered::shared_ptr& variableIndex, bool useQR) :
    Base(factorGraph, variableIndex), useQR_(useQR) {}

/* ************************************************************************* */
GaussianMultifrontalSolver::shared_ptr
GaussianMultifrontalSolver::Create(const FactorGraphOrdered<GaussianFactorOrdered>::shared_ptr& factorGraph,
    const VariableIndexOrdered::shared_ptr& variableIndex, bool useQR) {
  return shared_ptr(new GaussianMultifrontalSolver(factorGraph, variableIndex, useQR));
}

/* ************************************************************************* */
void GaussianMultifrontalSolver::replaceFactors(const FactorGraphOrdered<GaussianFactorOrdered>::shared_ptr& factorGraph) {
  Base::replaceFactors(factorGraph);
}

/* ************************************************************************* */
GaussianBayesTreeOrdered::shared_ptr GaussianMultifrontalSolver::eliminate() const {
  if (useQR_)
    return Base::eliminate(&EliminateQROrdered);
  else
    return Base::eliminate(&EliminatePreferCholeskyOrdered);
}

/* ************************************************************************* */
VectorValuesOrdered::shared_ptr GaussianMultifrontalSolver::optimize() const {
  gttic(GaussianMultifrontalSolver_optimize);
  VectorValuesOrdered::shared_ptr values;
  if (useQR_)
    values = VectorValuesOrdered::shared_ptr(new VectorValuesOrdered(junctionTree_->optimize(&EliminateQROrdered)));
  else
    values= VectorValuesOrdered::shared_ptr(new VectorValuesOrdered(junctionTree_->optimize(&EliminatePreferCholeskyOrdered)));
  return values;
}

/* ************************************************************************* */
GaussianFactorGraphOrdered::shared_ptr GaussianMultifrontalSolver::jointFactorGraph(const std::vector<Index>& js) const {
  if (useQR_)
    return GaussianFactorGraphOrdered::shared_ptr(new GaussianFactorGraphOrdered(*Base::jointFactorGraph(js,&EliminateQROrdered)));
  else
    return GaussianFactorGraphOrdered::shared_ptr(new GaussianFactorGraphOrdered(*Base::jointFactorGraph(js,&EliminatePreferCholeskyOrdered)));
}

/* ************************************************************************* */
GaussianFactorOrdered::shared_ptr GaussianMultifrontalSolver::marginalFactor(Index j) const {
  if (useQR_)
    return Base::marginalFactor(j,&EliminateQROrdered);
  else
    return Base::marginalFactor(j,&EliminatePreferCholeskyOrdered);
}

/* ************************************************************************* */
Matrix GaussianMultifrontalSolver::marginalCovariance(Index j) const {
  FactorGraphOrdered<GaussianFactorOrdered> fg;
  GaussianConditionalOrdered::shared_ptr conditional;
  if (useQR_) {
    fg.push_back(Base::marginalFactor(j,&EliminateQROrdered));
    conditional = EliminateQROrdered(fg,1).first;
  } else {
    fg.push_back(Base::marginalFactor(j,&EliminatePreferCholeskyOrdered));
    conditional = EliminatePreferCholeskyOrdered(fg,1).first;
  }
  return conditional->information().inverse();
}

}
