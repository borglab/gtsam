/**
 * @file    GaussianISAM
 * @brief   Linear ISAM only
 * @author  Michael Kaess
 */

#include "GaussianISAM.h"

using namespace std;
using namespace gtsam;

// Explicitly instantiate so we don't have to include everywhere
#include "ISAM-inl.h"
template class ISAM<GaussianConditional>;

namespace gtsam {

/* ************************************************************************* */
void optimize(const GaussianISAM::sharedClique& clique, VectorConfig& result) {
	// parents are assumed to already be solved and available in result
	GaussianISAM::Clique::const_reverse_iterator it;
	for (it = clique->rbegin(); it!=clique->rend(); it++) {
		GaussianConditional::shared_ptr cg = *it;
    Vector x = cg->solve(result); // Solve for that variable
    result.insert(cg->key(), x);   // store result in partial solution
  }
	BOOST_FOREACH(const GaussianISAM::sharedClique& child, clique->children_) {
//	list<GaussianISAM::Clique::shared_ptr>::const_iterator child;
//	for (child = clique->children_.begin(); child != clique->children_.end(); child++) {
		optimize(child, result);
	}
}

/* ************************************************************************* */
VectorConfig optimize(const GaussianISAM& bayesTree) {
	VectorConfig result;
	// starting from the root, call optimize on each conditional
	optimize(bayesTree.root(), result);
	return result;
}

} /// namespace gtsam
