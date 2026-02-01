/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTreeMarginalizationHelper.h
 * @brief   Helper functions for marginalizing variables from a Bayes Tree.
 *
 * @author  Jeffrey (Zhiwei Wang)
 * @date    Oct 28, 2024
 */

// \callgraph
#pragma once

#include <unordered_map>
#include <unordered_set>
#include <deque>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/base/debug.h>
#include "gtsam/dllexport.h"

namespace gtsam {

/**
 * This class provides helper functions for marginalizing variables from a Bayes Tree.
 */
template <typename BayesTree>
class GTSAM_EXPORT BayesTreeMarginalizationHelper {

public:
  using Clique = typename BayesTree::Clique;
  using sharedClique = typename BayesTree::sharedClique;

  /**
   * This function identifies variables that need to be re-eliminated before
   * performing marginalization.
   * 
   * Re-elimination is necessary for a clique containing marginalizable
   * variables if:
   * 
   * 1. Some non-marginalizable variables appear before marginalizable ones
   *    in that clique;
   * 2. Or it has a child node depending on a marginalizable variable AND the
   *    subtree rooted at that child contains non-marginalizables.
   * 
   * In addition, for any descendant node depending on a marginalizable
   * variable, if the subtree rooted at that descendant contains
   * non-marginalizable variables (i.e., it lies on a path from one of the
   * aforementioned cliques that require re-elimination to a node containing
   * non-marginalizable variables at the leaf side), then it also needs to
   * be re-eliminated.
   * 
   * @param[in] bayesTree The Bayes tree
   * @param[in] marginalizableKeys Keys to be marginalized
   * @return Set of additional keys that need to be re-eliminated
   */
  static std::unordered_set<Key>
  gatherAdditionalKeysToReEliminate(
      const BayesTree& bayesTree,
      const KeyVector& marginalizableKeys) {
    const bool debug = ISDEBUG("BayesTreeMarginalizationHelper");

    std::unordered_set<const Clique*> additionalCliques =
        gatherAdditionalCliquesToReEliminate(bayesTree, marginalizableKeys);    

    std::unordered_set<Key> additionalKeys;
    for (const Clique* clique : additionalCliques) {
      addCliqueToKeySet(clique, &additionalKeys);
    }

    if (debug) {
      std::cout << "BayesTreeMarginalizationHelper: Additional keys to re-eliminate: ";
      for (const Key& key : additionalKeys) {
        std::cout << DefaultKeyFormatter(key) << " ";
      }
      std::cout << std::endl;
    }

    return additionalKeys;
  }

 protected:
  /**
   * This function identifies cliques that need to be re-eliminated before
   * performing marginalization.
   * See the docstring of @ref gatherAdditionalKeysToReEliminate().
   */
  static std::unordered_set<const Clique*>
  gatherAdditionalCliquesToReEliminate(
      const BayesTree& bayesTree,
      const KeyVector& marginalizableKeys) {
    std::unordered_set<const Clique*> additionalCliques;
    std::unordered_set<Key> marginalizableKeySet(
        marginalizableKeys.begin(), marginalizableKeys.end());
    CachedSearch cachedSearch;

    // Check each clique that contains a marginalizable key
    for (const Clique* clique :
         getCliquesContainingKeys(bayesTree, marginalizableKeySet)) {
      if (additionalCliques.count(clique)) {
        // The clique has already been visited. This can happen when an
        // ancestor of the current clique also contain some marginalizable
        // varaibles and it's processed beore the current.
        continue;
      }

      if (needsReelimination(clique, marginalizableKeySet, &cachedSearch)) {
        // Add the current clique
        additionalCliques.insert(clique);

        // Then add the dependent cliques
        gatherDependentCliques(clique, marginalizableKeySet, &additionalCliques,
                               &cachedSearch);
      }
    }
    return additionalCliques;
  }

  /**
   * Gather the cliques containing any of the given keys.
   * 
   * @param[in] bayesTree The Bayes tree
   * @param[in] keysOfInterest Set of keys of interest
   * @return Set of cliques that contain any of the given keys
   */
  static std::unordered_set<const Clique*> getCliquesContainingKeys(
      const BayesTree& bayesTree,
      const std::unordered_set<Key>& keysOfInterest) {
    std::unordered_set<const Clique*> cliques;
    for (const Key& key : keysOfInterest) {
      cliques.insert(bayesTree[key].get());
    }
    return cliques;
  }

  /**
   * A struct to cache the results of the below two functions.
   */
  struct CachedSearch {
    std::unordered_map<const Clique*, bool> wholeMarginalizableCliques;
    std::unordered_map<const Clique*, bool> wholeMarginalizableSubtrees;
  };

  /**
   * Check if all variables in the clique are marginalizable.
   * 
   * Note we use a cache map to avoid repeated searches.
   */
  static bool isWholeCliqueMarginalizable(
      const Clique* clique,
      const std::unordered_set<Key>& marginalizableKeys,
      CachedSearch* cache) {
    auto it = cache->wholeMarginalizableCliques.find(clique);
    if (it != cache->wholeMarginalizableCliques.end()) {
      return it->second;
    } else {
      bool ret = true;
      for (Key key : clique->conditional()->frontals()) {
        if (!marginalizableKeys.count(key)) {
          ret = false;
          break;
        }
      }
      cache->wholeMarginalizableCliques.insert({clique, ret});
      return ret;
    }
  }

  /**
   * Check if all variables in the subtree are marginalizable.
   * 
   * Note we use a cache map to avoid repeated searches.
   */
  static bool isWholeSubtreeMarginalizable(
      const Clique* subtree,
      const std::unordered_set<Key>& marginalizableKeys,
      CachedSearch* cache) {
    auto it = cache->wholeMarginalizableSubtrees.find(subtree);
    if (it != cache->wholeMarginalizableSubtrees.end()) {
      return it->second;
    } else {
      bool ret = true;
      if (isWholeCliqueMarginalizable(subtree, marginalizableKeys, cache)) {
        for (const sharedClique& child : subtree->children) {
          if (!isWholeSubtreeMarginalizable(child.get(), marginalizableKeys, cache)) {
            ret = false;
            break;
          }
        }
      } else {
        ret = false;
      }
      cache->wholeMarginalizableSubtrees.insert({subtree, ret});
      return ret;
    }
  }

  /**
   * Check if a clique contains variables that need reelimination due to
   * elimination ordering conflicts.
   * 
   * @param[in] clique The clique to check
   * @param[in] marginalizableKeys Set of keys to be marginalized
   * @return true if any variables in the clique need re-elimination
   */
  static bool needsReelimination(
      const Clique* clique,
      const std::unordered_set<Key>& marginalizableKeys,
      CachedSearch* cache) {
    bool hasNonMarginalizableAhead = false;

    // Check each frontal variable in order
    for (Key key : clique->conditional()->frontals()) {
      if (marginalizableKeys.count(key)) {
        // If we've seen non-marginalizable variables before this one,
        // we need to reeliminate
        if (hasNonMarginalizableAhead) {
          return true;
        }

        // Check if any child depends on this marginalizable key and the
        // subtree rooted at that child contains non-marginalizables.
        for (const sharedClique& child : clique->children) {
          if (hasDependency(child.get(), key) &&
              !isWholeSubtreeMarginalizable(child.get(), marginalizableKeys, cache)) {
            return true;
          }
        }
      } else {
        hasNonMarginalizableAhead = true;
      }
    }
    return false;
  }

  /**
   * Gather all dependent nodes that lie on a path from the root clique
   * to a clique containing a non-marginalizable variable at the leaf side.
   *
   * @param[in] rootClique The root clique
   * @param[in] marginalizableKeys Set of keys to be marginalized
   */
  static void gatherDependentCliques(
      const Clique* rootClique,
      const std::unordered_set<Key>& marginalizableKeys,
      std::unordered_set<const Clique*>* additionalCliques,
      CachedSearch* cache) {
    std::vector<const Clique*> dependentChildren;
    dependentChildren.reserve(rootClique->children.size());
    for (const sharedClique& child : rootClique->children) {
      if (additionalCliques->count(child.get())) {
        // This child has already been visited. This can happen if the
        // child itself contains a marginalizable variable and it's
        // processed before the current rootClique.
        continue;
      }
      if (hasDependency(child.get(), marginalizableKeys)) {
        dependentChildren.push_back(child.get());
      }
    }
    gatherDependentCliquesFromChildren(
        dependentChildren, marginalizableKeys, additionalCliques, cache);
  }

  /**
   * A helper function for the above gatherDependentCliques().
   */
  static void gatherDependentCliquesFromChildren(
      const std::vector<const Clique*>& dependentChildren,
      const std::unordered_set<Key>& marginalizableKeys,
      std::unordered_set<const Clique*>* additionalCliques,
      CachedSearch* cache) {
    std::deque<const Clique*> descendants(
        dependentChildren.begin(), dependentChildren.end());
    while (!descendants.empty()) {
      const Clique* descendant = descendants.front();
      descendants.pop_front();

      // If the subtree rooted at this descendant contains non-marginalizables,
      // it must lie on a path from the root clique to a clique containing
      // non-marginalizables at the leaf side.
      if (!isWholeSubtreeMarginalizable(descendant, marginalizableKeys, cache)) {
        additionalCliques->insert(descendant);

        // Add children of the current descendant to the set descendants.
        for (const sharedClique& child : descendant->children) {
          if (additionalCliques->count(child.get())) {
            // This child has already been visited.
            continue;
          } else {
            descendants.push_back(child.get());
          }
        }
      }
    }
  }

  /**
   * Add all frontal variables from a clique to a key set.
   * 
   * @param[in] clique Clique to add keys from
   * @param[out] additionalKeys Pointer to the output key set
   */
  static void addCliqueToKeySet(
      const Clique* clique,
      std::unordered_set<Key>* additionalKeys) {
    for (Key key : clique->conditional()->frontals()) {
      additionalKeys->insert(key);
    }
  }

  /**
   * Check if the clique depends on the given key.
   * 
   * @param[in] clique Clique to check
   * @param[in] key Key to check for dependencies
   * @return true if clique depends on the key
   */
  static bool hasDependency(
      const Clique* clique, Key key) {
    auto& conditional = clique->conditional();
    if (std::find(conditional->beginParents(),
        conditional->endParents(), key)
        != conditional->endParents()) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Check if the clique depends on any of the given keys.
   */
  static bool hasDependency(
      const Clique* clique, const std::unordered_set<Key>& keys) {
    auto& conditional = clique->conditional();
    for (auto it = conditional->beginParents();
        it != conditional->endParents(); ++it) {
      if (keys.count(*it)) {
        return true;
      }
    }

    return false;
  }
};
// BayesTreeMarginalizationHelper

}/// namespace gtsam
