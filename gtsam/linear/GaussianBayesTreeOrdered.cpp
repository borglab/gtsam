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

#include <gtsam/linear/GaussianBayesTreeOrdered.h>
#include <gtsam/linear/GaussianFactorGraphOrdered.h>

namespace gtsam {

/* ************************************************************************* */
VectorValuesOrdered optimize(const GaussianBayesTreeOrdered& bayesTree) {
  VectorValuesOrdered result = *allocateVectorValues(bayesTree);
  optimizeInPlace(bayesTree, result);
  return result;
}

/* ************************************************************************* */
void optimizeInPlace(const GaussianBayesTreeOrdered& bayesTree, VectorValuesOrdered& result) {
  internal::optimizeInPlace<GaussianBayesTreeOrdered>(bayesTree.root(), result);
}

/* ************************************************************************* */
VectorValuesOrdered optimizeGradientSearch(const GaussianBayesTreeOrdered& bayesTree) {
  gttic(Allocate_VectorValues);
  VectorValuesOrdered grad = *allocateVectorValues(bayesTree);
  gttoc(Allocate_VectorValues);

  optimizeGradientSearchInPlace(bayesTree, grad);

  return grad;
}

/* ************************************************************************* */
void optimizeGradientSearchInPlace(const GaussianBayesTreeOrdered& bayesTree, VectorValuesOrdered& grad) {
  gttic(Compute_Gradient);
  // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
  gradientAtZero(bayesTree, grad);
  double gradientSqNorm = grad.dot(grad);
  gttoc(Compute_Gradient);

  gttic(Compute_Rg);
  // Compute R * g
  FactorGraphOrdered<JacobianFactorOrdered> Rd_jfg(bayesTree);
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
VectorValuesOrdered gradient(const GaussianBayesTreeOrdered& bayesTree, const VectorValuesOrdered& x0) {
  return gradient(FactorGraphOrdered<JacobianFactorOrdered>(bayesTree), x0);
}

/* ************************************************************************* */
void gradientAtZero(const GaussianBayesTreeOrdered& bayesTree, VectorValuesOrdered& g) {
  gradientAtZero(FactorGraphOrdered<JacobianFactorOrdered>(bayesTree), g);
}

/* ************************************************************************* */
double determinant(const GaussianBayesTreeOrdered& bayesTree) {
  if (!bayesTree.root())
    return 0.0;

  return exp(internal::logDeterminant<GaussianBayesTreeOrdered>(bayesTree.root()));
}
/* ************************************************************************* */

} // \namespace gtsam




