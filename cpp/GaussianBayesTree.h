/**
 * @file    GaussianBayesTree
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <map>
#include <list>
#include <vector>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>
#include <stdexcept>

#include "Testable.h"
#include "BayesTree.h"
#include "VectorConfig.h"
#include "GaussianConditional.h"

namespace gtsam {

	typedef BayesTree<GaussianConditional> GaussianBayesTree;
#if 0
	// recursively optimize starting with this conditional and all children
	void optimize(sharedClique clique, VectorConfig& result) {
	  // parents are assumed to already be solved and available in result
		BOOST_REVERSE_FOREACH(GaussianConditional::shared_ptr cg, clique) {
	    Vector x = cg->solve(result); // Solve for that variable
	    result.insert(cg->key(),x);   // store result in partial solution
	  }
		BOOST_FOREACH(sharedClique child, clique->children_) {
			optimize(child, result);
		}
	}

	void optimize(const GaussianBayesTree& bayesTree, VectorConfig& result) {
		// starting from the root, call optimize on each conditional
		optimize(bayesTree.root(), result);
	}
#endif
} /// namespace gtsam
