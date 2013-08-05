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
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

// Managing orderings

/** Converts sets of keys to indices by way of orderings */
GTSAM_UNSTABLE_EXPORT std::set<Index> keysToIndices(const KeySet& keys, const OrderingOrdered& ordering);

// Linear Graph Operations

/**
 * Given a graph, splits each factor into factors where the dimension is
 * that of the first variable.
 */
GTSAM_UNSTABLE_EXPORT GaussianFactorGraphOrdered splitFactors(const GaussianFactorGraphOrdered& fullgraph);

/**
 * Splits a factor into factors where the dimension is
 * that of the first variable.
 */
GTSAM_UNSTABLE_EXPORT GaussianFactorGraphOrdered splitFactor(const GaussianFactorOrdered::shared_ptr& factor);

/** Removes prior jacobian factors from the graph */
GTSAM_UNSTABLE_EXPORT GaussianFactorGraphOrdered removePriors(const GaussianFactorGraphOrdered& fullgraph);

// Bayes Tree / Conditional operations

/**
 * Given a Bayes Tree, return conditionals corresponding to cliques that have or
 * are connected to a set of wanted variables.
 *
 * @return the set of conditionals extracted from cliques.
 */
GTSAM_UNSTABLE_EXPORT std::set<GaussianConditionalOrdered::shared_ptr> findAffectedCliqueConditionals(
    const GaussianBayesTreeOrdered& bayesTree, const std::set<Index>& savedIndices);

/**
 * Recursively traverses from a given clique in a Bayes Tree and collects all of the conditionals
 * Adds any new cliques from path to root to the result set.
 *
 * Note the use of a set of shared_ptr: this will sort/filter on unique *pointer* locations,
 * which ensures unique cliques, but the order of the cliques is meaningless
 */
GTSAM_UNSTABLE_EXPORT void findCliqueConditionals(const GaussianBayesTreeOrdered::sharedClique& current_clique,
    std::set<GaussianConditionalOrdered::shared_ptr>& result);

/**
 * Given a clique, returns a sequence of clique parents to the root, not including the
 * given clique.
 */
GTSAM_UNSTABLE_EXPORT std::deque<GaussianBayesTreeOrdered::sharedClique>
findPathCliques(const GaussianBayesTreeOrdered::sharedClique& initial);

/**
 * Liquefies a BayesTree into a GaussianFactorGraph recursively, given a
 * root clique
 *
 * @param splitConditionals flag enables spliting multi-frontal conditionals into separate factors
 */
template <class BAYESTREE>
GaussianFactorGraphOrdered liquefy(const typename BAYESTREE::sharedClique& root, bool splitConditionals = false) {
  GaussianFactorGraphOrdered result;
  if (root && root->conditional()) {
    GaussianConditionalOrdered::shared_ptr conditional = root->conditional();
    if (!splitConditionals)
      result.push_back(conditional->toFactor());
    else
      result.push_back(splitFactor(conditional->toFactor()));
  }
  BOOST_FOREACH(const typename BAYESTREE::sharedClique& child, root->children())
    result.push_back(liquefy<BAYESTREE>(child, splitConditionals));
  return result;
}

/**
 * Liquefies a BayesTree into a GaussianFactorGraph recursively, from a full bayes tree.
 *
 * @param splitConditionals flag enables spliting multi-frontal conditionals into separate factors
 */
template <class BAYESTREE>
GaussianFactorGraphOrdered liquefy(const BAYESTREE& bayesTree, bool splitConditionals = false) {
  return liquefy<BAYESTREE>(bayesTree.root(), splitConditionals);
}

} // \namespace gtsam


