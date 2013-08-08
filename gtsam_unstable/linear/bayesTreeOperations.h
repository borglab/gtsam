/**
 * @file bayesTreeOperations.h
 *
 * @brief Types and utility functions for operating on linear systems
 * 
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/base/dllexport.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesTree.h>

namespace gtsam {

// Linear Graph Operations

/**
 * Given a graph, splits each factor into factors where the dimension is
 * that of the first variable.
 */
GTSAM_UNSTABLE_EXPORT GaussianFactorGraph splitFactors(const GaussianFactorGraph& fullgraph);

/**
 * Splits a factor into factors where the dimension is
 * that of the first variable.
 */
GTSAM_UNSTABLE_EXPORT GaussianFactorGraph splitFactor(const GaussianFactor::shared_ptr& factor);

/** Removes prior jacobian factors from the graph */
GTSAM_UNSTABLE_EXPORT GaussianFactorGraph removePriors(const GaussianFactorGraph& fullgraph);

// Bayes Tree / Conditional operations

/**
 * Given a Bayes Tree, return conditionals corresponding to cliques that have or
 * are connected to a set of wanted variables.
 *
 * @return the set of conditionals extracted from cliques.
 */
GTSAM_UNSTABLE_EXPORT std::set<GaussianConditional::shared_ptr> findAffectedCliqueConditionals(
    const GaussianBayesTree& bayesTree, const std::set<Key>& savedIndices);

/**
 * Recursively traverses from a given clique in a Bayes Tree and collects all of the conditionals
 * Adds any new cliques from path to root to the result set.
 *
 * Note the use of a set of shared_ptr: this will sort/filter on unique *pointer* locations,
 * which ensures unique cliques, but the order of the cliques is meaningless
 */
GTSAM_UNSTABLE_EXPORT void findCliqueConditionals(const GaussianBayesTree::sharedClique& current_clique,
    std::set<GaussianConditional::shared_ptr>& result);

/**
 * Given a clique, returns a sequence of clique parents to the root, not including the
 * given clique.
 */
GTSAM_UNSTABLE_EXPORT std::deque<GaussianBayesTree::sharedClique>
findPathCliques(const GaussianBayesTree::sharedClique& initial);

/**
 * Liquefies a BayesTree into a GaussianFactorGraph recursively, given a
 * root clique
 *
 * @param splitConditionals flag enables spliting multi-frontal conditionals into separate factors
 */
template <class BAYESTREE>
GaussianFactorGraph liquefy(const typename BAYESTREE::sharedClique& root, bool splitConditionals = false) {
  GaussianFactorGraph result;
  if (root && root->conditional()) {
    GaussianConditional::shared_ptr conditional = root->conditional();
    if (!splitConditionals)
      result.push_back(conditional);
    else
      result.push_back(splitFactor(conditional));
  }
  BOOST_FOREACH(typename BAYESTREE::sharedClique child, root->children)
    result.push_back(liquefy<BAYESTREE>(child, splitConditionals));
  return result;
}

/**
 * Liquefies a BayesTree into a GaussianFactorGraph recursively, from a full bayes tree.
 *
 * @param splitConditionals flag enables spliting multi-frontal conditionals into separate factors
 */
template <class BAYESTREE>
GaussianFactorGraph liquefy(const BAYESTREE& bayesTree, bool splitConditionals = false) {
  GaussianFactorGraph result;
  BOOST_FOREACH(const typename BAYESTREE::sharedClique& root, bayesTree.roots())
    result.push_back(liquefy<BAYESTREE>(root, splitConditionals));
  return result;
}

} // \namespace gtsam


