/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianISAM.cpp
 * @brief   Linear ISAM only
 * @author  Michael Kaess
 */

#include <gtsam/linear/GaussianISAMOrdered.h>
#include <gtsam/linear/GaussianBayesTreeOrdered.h>

#include <gtsam/inference/ISAMOrdered.h>

using namespace std;
using namespace gtsam;

namespace gtsam {

// Explicitly instantiate so we don't have to include everywhere
template class ISAMOrdered<GaussianConditionalOrdered>;

/* ************************************************************************* */
GaussianFactorOrdered::shared_ptr GaussianISAMOrdered::marginalFactor(Index j) const {
  return Super::marginalFactor(j, &EliminateQROrdered);
}

/* ************************************************************************* */
BayesNetOrdered<GaussianConditionalOrdered>::shared_ptr GaussianISAMOrdered::marginalBayesNet(Index j) const {
  return Super::marginalBayesNet(j, &EliminateQROrdered);
}

/* ************************************************************************* */
Matrix GaussianISAMOrdered::marginalCovariance(Index j) const {
  GaussianConditionalOrdered::shared_ptr conditional = marginalBayesNet(j)->front();
  return conditional->information().inverse();
}

/* ************************************************************************* */
  BayesNetOrdered<GaussianConditionalOrdered>::shared_ptr GaussianISAMOrdered::jointBayesNet(
      Index key1, Index key2) const {
  return Super::jointBayesNet(key1, key2, &EliminateQROrdered);
}

/* ************************************************************************* */
VectorValuesOrdered optimize(const GaussianISAMOrdered& isam) {
  VectorValuesOrdered result(isam.dims_);
  // Call optimize for BayesTree
  optimizeInPlace((const BayesTreeOrdered<GaussianConditionalOrdered>&)isam, result);
  return result;
}

/* ************************************************************************* */
BayesNetOrdered<GaussianConditionalOrdered> GaussianISAMOrdered::shortcut(sharedClique clique, sharedClique root) {
  return clique->shortcut(root,&EliminateQROrdered);
}
/* ************************************************************************* */

} // \namespace gtsam
