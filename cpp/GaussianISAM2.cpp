/**
 * @file    GaussianISAM2
 * @brief   Full non-linear ISAM
 * @author  Michael Kaess
 */

#include "GaussianISAM2.h"

using namespace std;
using namespace gtsam;

// Explicitly instantiate so we don't have to include everywhere
#include "ISAM2-inl.h"
//template class ISAM2<GaussianConditional, VectorConfig>;

namespace gtsam {

/* ************************************************************************* */
void optimize2(const GaussianISAM2::sharedClique& clique, VectorConfig& result) {
	// parents are assumed to already be solved and available in result
	GaussianISAM2::Clique::const_reverse_iterator it;
	for (it = clique->rbegin(); it!=clique->rend(); it++) {
		GaussianConditional::shared_ptr cg = *it;
    Vector x = cg->solve(result); // Solve for that variable
    result.insert(cg->key(), x);   // store result in partial solution
  }
	BOOST_FOREACH(GaussianISAM2::sharedClique child, clique->children_) {
//	list<GaussianISAM2::Clique::shared_ptr>::const_iterator child;
//	for (child = clique->children_.begin(); child != clique->children_.end(); child++) {
		optimize2(child, result);
	}
}

/* ************************************************************************* */
VectorConfig optimize2(const GaussianISAM2& bayesTree) {
	VectorConfig result;
	// starting from the root, call optimize on each conditional
	optimize2(bayesTree.root(), result);
	return result;
}

} /// namespace gtsam
