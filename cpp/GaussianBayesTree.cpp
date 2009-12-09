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
void optimize(GaussianBayesTree::sharedClique clique, VectorConfig& result) {
#if 0
	// parents are assumed to already be solved and available in result
	BOOST_REVERSE_FOREACH(GaussianConditional::shared_ptr cg, clique) {
    Vector x = cg->solve(result); // Solve for that variable
    result.insert(cg->key(),x);   // store result in partial solution
  }
	BOOST_FOREACH(GaussianBayesTree::sharedClique child, clique->children_) {
		optimize(child, result);
	}
#endif
}

/* ************************************************************************* */
VectorConfig optimize(GaussianBayesTree& bayesTree) {
	VectorConfig result;
	// starting from the root, call optimize on each conditional
	optimize(bayesTree.root(), result);
}

} /// namespace gtsam
