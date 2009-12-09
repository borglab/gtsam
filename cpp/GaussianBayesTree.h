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
#include "GaussianBayesTree.h"

namespace gtsam {

	typedef BayesTree<GaussianConditional> GaussianBayesTree;

	// recursively optimize this conditional and all subtrees
	void optimize(GaussianBayesTree::sharedClique clique, VectorConfig& result);

	// optimize the BayesTree, starting from the root
	VectorConfig optimize(GaussianBayesTree& bayesTree);

}/// namespace gtsam
