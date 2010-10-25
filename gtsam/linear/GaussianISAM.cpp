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

namespace gtsam {

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

} /// namespace gtsam
