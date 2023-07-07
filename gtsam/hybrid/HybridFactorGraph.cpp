/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridFactorGraph.cpp
 * @brief  Factor graph with utilities for hybrid factors.
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   January, 2023
 */

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/hybrid/HybridFactorGraph.h>

namespace gtsam {

/* ************************************************************************* */
std::set<DiscreteKey> HybridFactorGraph::discreteKeys() const {
  std::set<DiscreteKey> keys;
  for (auto& factor : factors_) {
    if (auto p = std::dynamic_pointer_cast<DecisionTreeFactor>(factor)) {
      for (const DiscreteKey& key : p->discreteKeys()) {
        keys.insert(key);
      }
    }
    if (auto p = std::dynamic_pointer_cast<HybridFactor>(factor)) {
      for (const DiscreteKey& key : p->discreteKeys()) {
        keys.insert(key);
      }
    }
  }
  return keys;
}

/* ************************************************************************* */
KeySet HybridFactorGraph::discreteKeySet() const {
  KeySet keys;
  std::set<DiscreteKey> key_set = discreteKeys();
  std::transform(key_set.begin(), key_set.end(),
                 std::inserter(keys, keys.begin()),
                 [](const DiscreteKey& k) { return k.first; });
  return keys;
}

/* ************************************************************************* */
std::unordered_map<Key, DiscreteKey> HybridFactorGraph::discreteKeyMap() const {
  std::unordered_map<Key, DiscreteKey> result;
  for (const DiscreteKey& k : discreteKeys()) {
    result[k.first] = k;
  }
  return result;
}

/* ************************************************************************* */
const KeySet HybridFactorGraph::continuousKeySet() const {
  KeySet keys;
  for (auto& factor : factors_) {
    if (auto p = std::dynamic_pointer_cast<HybridFactor>(factor)) {
      for (const Key& key : p->continuousKeys()) {
        keys.insert(key);
      }
    } else if (auto p = std::dynamic_pointer_cast<GaussianFactor>(factor)) {
      keys.insert(p->keys().begin(), p->keys().end());
    }
  }
  return keys;
}

/* ************************************************************************* */

}  // namespace gtsam
