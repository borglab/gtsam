/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file kruskal-inl.h
 * @date Dec 31, 2009
 * @author Frank Dellaert
 * @author Yong-Dian Jian
 * @author Ankur Roy Chowdhury
 */

#pragma once

#include <gtsam/base/DSFMap.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/types.h>
#include <gtsam/inference/VariableIndex.h>

#include <algorithm>
#include <numeric>
#include <vector>

namespace gtsam::utils {

/*****************************************************************************/
/* Sort the 'weights' in increasing order and return the sorted order of its
 * indices. */
inline std::vector<size_t> sortedIndices(const std::vector<double> &weights) {
  // Get the number of elements in the 'weights' vector.
  const size_t n = weights.size();

  // Create a vector of 'indices'.
  std::vector<size_t> indices(n);
  std::iota(indices.begin(), indices.end(), 0);

  // Sort the 'indices' based on the values in 'weights'.
  std::stable_sort(indices.begin(), indices.end(),
                   [&weights](const size_t i0, const size_t i1) {
                     return weights[i0] < weights[i1];
                   });

  return indices;
}

/****************************************************************/
template <class FACTOR>
std::vector<size_t> kruskal(const FactorGraph<FACTOR> &fg,
                            const std::vector<double> &weights) {
  if (fg.size() != weights.size()) {
    throw std::runtime_error(
        "kruskal() failure: fg.size() != weights.size(), all factors need to "
        "assigned a weight");
  }

  // Create an index from variables to factor indices.
  const VariableIndex variableIndex(fg);

  // Get indices in sort-order of the weights.
  const std::vector<size_t> sortedIndices =
      gtsam::utils::sortedIndices(weights);

  // Create a vector to hold MST indices.
  const size_t n = variableIndex.size();
  std::vector<size_t> treeIndices;
  treeIndices.reserve(n - 1);

  // Initialize disjoint-set forest to keep track of merged Keys.
  DSFMap<Key> dsf;

  // Loop over all edges in order of increasing weight.
  size_t count = 0;
  for (const size_t index : sortedIndices) {
    const auto factor = fg[index];

    // Ignore non-binary edges.
    if (factor->size() != 2) continue;

    // Find the representatives of the sets for both the Keys in the binary
    // factor.
    const auto u = dsf.find(factor->front()), v = dsf.find(factor->back());

    // Check if there is a potential loop.
    const bool loop = (u == v);
    if (!loop) {
      // Merge the sets.
      dsf.merge(u, v);

      // Add the current index to the tree.
      treeIndices.push_back(index);

      // Break if all the Keys have been added to the tree.
      if (++count == n - 1) break;
    }
  }
  return treeIndices;
}

}  // namespace gtsam::utils
