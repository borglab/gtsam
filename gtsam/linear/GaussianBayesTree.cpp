/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianBayesTree.cpp
 * @brief   Gaussian Bayes Tree, the result of eliminating a GaussianJunctionTree
 * @brief   GaussianBayesTree
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

/* ************************************************************************* */
VectorValues optimize(const GaussianBayesTree& bayesTree) {
  VectorValues result = *allocateVectorValues(bayesTree);
  optimizeInPlace(bayesTree, result);
  return result;
}

/* ************************************************************************* */
void optimizeInPlace(const GaussianBayesTree& bayesTree, VectorValues& result) {
  internal::optimizeInPlace<GaussianBayesTree>(bayesTree.root(), result);
}

/* ************************************************************************* */
VectorValues optimizeGradientSearch(const GaussianBayesTree& bayesTree) {
  gttic(Allocate_VectorValues);
  VectorValues grad = *allocateVectorValues(bayesTree);
  gttoc(Allocate_VectorValues);

  optimizeGradientSearchInPlace(bayesTree, grad);

  return grad;
}

/* ************************************************************************* */
void optimizeGradientSearchInPlace(const GaussianBayesTree& bayesTree, VectorValues& grad) {
  gttic(Compute_Gradient);
  // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
  gradientAtZero(bayesTree, grad);
  double gradientSqNorm = grad.dot(grad);
  gttoc(Compute_Gradient);

  gttic(Compute_Rg);
  // Compute R * g
  FactorGraph<JacobianFactor> Rd_jfg(bayesTree);
  Errors Rg = Rd_jfg * grad;
  gttoc(Compute_Rg);

  gttic(Compute_minimizing_step_size);
  // Compute minimizing step size
  double step = -gradientSqNorm / dot(Rg, Rg);
  gttoc(Compute_minimizing_step_size);

  gttic(Compute_point);
  // Compute steepest descent point
  scal(step, grad);
  gttoc(Compute_point);
}

/* ************************************************************************* */
VectorValues gradient(const GaussianBayesTree& bayesTree, const VectorValues& x0) {
  return gradient(FactorGraph<JacobianFactor>(bayesTree), x0);
}

/* ************************************************************************* */
void gradientAtZero(const GaussianBayesTree& bayesTree, VectorValues& g) {
  gradientAtZero(FactorGraph<JacobianFactor>(bayesTree), g);
}

/* ************************************************************************* */
double determinant(const GaussianBayesTree& bayesTree) {
  return exp(logDeterminant(bayesTree));
}

/* ************************************************************************* */
double logDeterminant(const GaussianBayesTree& bayesTree) {
  if (!bayesTree.root())
    return 0.0;

  return internal::logDeterminant<GaussianBayesTree>(bayesTree.root());
}
/* ************************************************************************* */

} // \namespace gtsam




