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

#pragma once

#include <gtsam/global_includes.h>
#include <gtsam/inference/Key.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Key type for discrete variables.
 * Includes Key and cardinality.
 */
using DiscreteKey = std::pair<Key, size_t>;

/// DiscreteKeys is a map of keys that can be assembled using the & operator
using DiscreteKeys = std::map<Key, size_t>;

/// Create a list from two keys
DiscreteKeys operator&(const DiscreteKey& key1, const DiscreteKey& key2);

/// Add a key to a DiscreteKeys object
DiscreteKeys& operator&(DiscreteKeys& keys, const DiscreteKey& key);

/// Add a (non-const) key to a DiscreteKeys object
DiscreteKeys& operator&(DiscreteKeys& keys, DiscreteKey& key);

/**
 * @brief Get the Indices object
 *
 * @param keys
 * @return KeyVector
 */
KeyVector getIndices(const DiscreteKeys& keys);

}  // namespace gtsam
