/**
 * @file    GaussianBayesTree
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>

#include "GaussianBayesTree.h"
#include "VectorConfig.h"

using namespace std;
using namespace gtsam;

// Explicitly instantiate so we don't have to include everywhere
#include "BayesTree-inl.h"
template class BayesTree<GaussianConditional>;

namespace gtsam {

/* ************************************************************************* */
void optimize(const GaussianBayesTree::sharedClique& clique, VectorConfig& result) {
	// parents are assumed to already be solved and available in result
	GaussianBayesTree::Clique::const_reverse_iterator it;
	for(it = clique->rend(); it!=clique->rbegin(); it++) {
		GaussianConditional::shared_ptr cg = *it;
    Vector x = cg->solve(result); // Solve for that variable
    result.insert(cg->key(),x);   // store result in partial solution
  }
	BOOST_FOREACH(GaussianBayesTree::sharedClique child, clique->children_) {
		optimize(child, result);
	}
}

/* ************************************************************************* */
VectorConfig optimize(const GaussianBayesTree& bayesTree) {
	VectorConfig result;
	// starting from the root, call optimize on each conditional
	optimize(bayesTree.root(), result);
}

} /// namespace gtsam
