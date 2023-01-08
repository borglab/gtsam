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

#include <boost/format.hpp>

namespace gtsam {

/* ************************************************************************* */
DiscreteKeys HybridFactorGraph::discreteKeys() const {
  DiscreteKeys keys;
  for (auto& factor : factors_) {
    if (auto p = boost::dynamic_pointer_cast<DecisionTreeFactor>(factor)) {
      for (const DiscreteKey& key : p->discreteKeys()) {
        keys.push_back(key);
      }
    }
    if (auto p = boost::dynamic_pointer_cast<HybridFactor>(factor)) {
      for (const DiscreteKey& key : p->discreteKeys()) {
        keys.push_back(key);
      }
    }
  }
  return keys;
}

/* ************************************************************************* */
KeySet HybridFactorGraph::discreteKeySet() const {
  KeySet keys;
  for (const DiscreteKey& k : discreteKeys()) {
    keys.insert(k.first);
  }
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
    if (auto p = boost::dynamic_pointer_cast<HybridFactor>(factor)) {
      for (const Key& key : p->continuousKeys()) {
        keys.insert(key);
      }
    }
  }
  return keys;
}

/* ************************************************************************* */

}  // namespace gtsam
