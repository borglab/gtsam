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
GTSAM_UNSTABLE_EXPORT std::set<Index> keysToIndices(const KeySet& keys, const Ordering& ordering);

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
    const GaussianBayesTree& bayesTree, const std::set<Index>& savedIndices);

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
 * Liquefies a GaussianBayesTree into a GaussianFactorGraph recursively, given either a
 * root clique or a full bayes tree.
 *
 * @param splitConditionals flag enables spliting multi-frontal conditionals into separate factors
 */
GTSAM_UNSTABLE_EXPORT GaussianFactorGraph liquefy(const GaussianBayesTree::sharedClique& root, bool splitConditionals = false);
GTSAM_UNSTABLE_EXPORT GaussianFactorGraph liquefy(const GaussianBayesTree& bayesTree, bool splitConditionals = false);

} // \namespace gtsam


