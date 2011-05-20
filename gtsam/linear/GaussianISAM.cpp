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

#include <gtsam/linear/GaussianISAM.h>

using namespace std;
using namespace gtsam;

// Explicitly instantiate so we don't have to include everywhere
#include <gtsam/inference/ISAM-inl.h>
template class ISAM<GaussianConditional>;

//namespace ublas = boost::numeric::ublas;

namespace gtsam {

/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianISAM::marginalFactor(Index j) const {
	return Super::marginalFactor(j, &EliminateQR);
}

/* ************************************************************************* */
BayesNet<GaussianConditional>::shared_ptr GaussianISAM::marginalBayesNet(Index j) const {
	return Super::marginalBayesNet(j, &EliminateQR);
}

/* ************************************************************************* */
std::pair<Vector, Matrix> GaussianISAM::marginal(Index j) const {
	GaussianConditional::shared_ptr conditional = marginalBayesNet(j)->front();
	Matrix R = conditional->get_R();
	return make_pair(conditional->get_d(), (R.transpose() * R).inverse());
}

/* ************************************************************************* */
	BayesNet<GaussianConditional>::shared_ptr GaussianISAM::jointBayesNet(
			Index key1, Index key2) const {
	return Super::jointBayesNet(key1, key2, &EliminateQR);
}

/* ************************************************************************* */
void optimize(const GaussianISAM::sharedClique& clique, VectorValues& result) {
	// parents are assumed to already be solved and available in result
	GaussianISAM::Clique::const_reverse_iterator it;
	for (it = clique->rbegin(); it!=clique->rend(); it++) {
		GaussianConditional::shared_ptr cg = *it;
    Vector x = cg->solve(result); // Solve for that variable
    result[cg->key()] = x;   // store result in partial solution
  }
	BOOST_FOREACH(const GaussianISAM::sharedClique& child, clique->children_) {
		optimize(child, result);
	}
}

/* ************************************************************************* */
VectorValues optimize(const GaussianISAM& bayesTree) {
	VectorValues result(bayesTree.dims_);
	// starting from the root, call optimize on each conditional
	optimize(bayesTree.root(), result);
	return result;
}

/* ************************************************************************* */
BayesNet<GaussianConditional> GaussianISAM::shortcut(sharedClique clique, sharedClique root) {
	return clique->shortcut(root,&EliminateQR);
}
/* ************************************************************************* */

} /// namespace gtsam
