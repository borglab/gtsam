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

#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/linear/GaussianBayesTree.h>

#include <gtsam/inference/ISAM.h>

using namespace std;
using namespace gtsam;

namespace gtsam {

// Explicitly instantiate so we don't have to include everywhere
template class ISAM<GaussianConditional>;

/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianISAM::marginalFactor(Index j) const {
	return Super::marginalFactor(j, &EliminateQR);
}

/* ************************************************************************* */
BayesNet<GaussianConditional>::shared_ptr GaussianISAM::marginalBayesNet(Index j) const {
	return Super::marginalBayesNet(j, &EliminateQR);
}

/* ************************************************************************* */
Matrix GaussianISAM::marginalCovariance(Index j) const {
	GaussianConditional::shared_ptr conditional = marginalBayesNet(j)->front();
	return conditional->computeInformation().inverse();
}

/* ************************************************************************* */
	BayesNet<GaussianConditional>::shared_ptr GaussianISAM::jointBayesNet(
			Index key1, Index key2) const {
	return Super::jointBayesNet(key1, key2, &EliminateQR);
}

/* ************************************************************************* */
VectorValues optimize(const GaussianISAM& isam) {
  VectorValues result(isam.dims_);
  // Call optimize for BayesTree
  optimizeInPlace((const BayesTree<GaussianConditional>&)isam, result);
  return result;
}

/* ************************************************************************* */
BayesNet<GaussianConditional> GaussianISAM::shortcut(sharedClique clique, sharedClique root) {
	return clique->shortcut(root,&EliminateQR);
}
/* ************************************************************************* */

} // \namespace gtsam
