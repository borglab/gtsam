/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianISAM
 * @brief   Linear ISAM only
 * @author  Michael Kaess
 */

#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <gtsam/linear/GaussianISAM.h>

#include <gtsam/inference/ISAM-inl.h>

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
void optimize(const BayesTree<GaussianConditional>::sharedClique& clique, VectorValues& result) {
	// parents are assumed to already be solved and available in result
	// RHS for current conditional should already be in place in result
  clique->conditional()->solveInPlace(result);

	BOOST_FOREACH(const BayesTree<GaussianConditional>::sharedClique& child, clique->children_)
		optimize(child, result);
}

/* ************************************************************************* */
void treeRHS(const BayesTree<GaussianConditional>::sharedClique& clique, VectorValues& result) {
  clique->conditional()->rhs(result);
	BOOST_FOREACH(const BayesTree<GaussianConditional>::sharedClique& child, clique->children_)
		treeRHS(child, result);
}

/* ************************************************************************* */
VectorValues rhs(const BayesTree<GaussianConditional>& bayesTree, boost::optional<const GaussianISAM::Dims&> dims) {
	VectorValues result;
	if(dims)
	  result = VectorValues(*dims);
	else
	  result = *allocateVectorValues(bayesTree); // allocate
	treeRHS(bayesTree.root(), result);    // recursively fill
	return result;
}

/* ************************************************************************* */
VectorValues optimize(const GaussianISAM& isam) {
  VectorValues result = rhs(isam, isam.dims_);
  // starting from the root, call optimize on each conditional
  optimize(isam.root(), result);
  return result;
}

/* ************************************************************************* */
VectorValues optimize(const BayesTree<GaussianConditional>& bayesTree) {
	VectorValues result = rhs(bayesTree);
	// starting from the root, call optimize on each conditional
	optimize(bayesTree.root(), result);
	return result;
}

/* ************************************************************************* */
BayesNet<GaussianConditional> GaussianISAM::shortcut(sharedClique clique, sharedClique root) {
	return clique->shortcut(root,&EliminateQR);
}
/* ************************************************************************* */

} // \namespace gtsam
