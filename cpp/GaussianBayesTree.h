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
#include "GaussianConditional.h"

namespace gtsam {

	typedef BayesTree<GaussianConditional> GaussianBayesTree;

} /// namespace gtsam
