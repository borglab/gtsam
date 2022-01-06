/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteKey.h
 * @brief specialized key for discrete variables
 * @author Frank Dellaert
 * @date Feb 28, 2011
 */

#include <gtsam/discrete/DiscreteKey.h>

#include <boost/format.hpp>  // for key names
#include <iostream>

namespace gtsam {

DiscreteKeys operator&(const DiscreteKey& key1, const DiscreteKey& key2) {
  DiscreteKeys keys;
  keys.emplace(key1);
  return keys & key2;
}

DiscreteKeys& operator&(DiscreteKeys& keys, const DiscreteKey& key) {
  keys.emplace(key);
  return keys;
}

DiscreteKeys& operator&(DiscreteKeys& keys, DiscreteKey& key) {
  keys.emplace(key);
  return keys;
}

KeyVector getIndices(const DiscreteKeys& keys) {
  KeyVector indices;
  std::transform(keys.begin(), keys.end(), indices.begin(),
                 [](const std::pair<Key, size_t> discreteKey) {
                   return discreteKey.first;
                 });
  return indices;
}

}  // namespace gtsam
