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
 * @author Frank Dellaert, Yong-Dian Jian
 */

#pragma once

#include <gtsam/base/DSFMap.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/types.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/VariableIndex.h>

#include <memory>
#include <vector>

namespace gtsam::utils {

/*****************************************************************************/
/* sort the container and return permutation index with default comparator */
inline std::vector<size_t> sortedIndices(const std::vector<double> &src) {
  const size_t n = src.size();
  std::vector<std::pair<size_t, double>> tmp;
  tmp.reserve(n);
  for (size_t i = 0; i < n; i++) tmp.emplace_back(i, src[i]);

  /* sort */
  std::stable_sort(tmp.begin(), tmp.end());

  /* copy back */
  std::vector<size_t> idx;
  idx.reserve(n);
  for (size_t i = 0; i < n; i++) {
    idx.push_back(tmp[i].first);
  }
  return idx;
}

/****************************************************************/
template <class Graph>
std::vector<size_t> kruskal(const Graph &fg,
                            const std::vector<double> &weights) {
  if (fg.size() != weights.size()) {
    throw std::runtime_error(
        "kruskal() failure: fg.size() != weights.size(), all factors need to "
        "assigned a weight");
  }

  // Create an index from variables to factor indices.
  const VariableIndex variableIndex(fg);

  // Get indices in sort-order of the weights
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

    auto u = dsf.find(factor->front()), v = dsf.find(factor->back());
    auto loop = (u == v);
    if (!loop) {
      dsf.merge(u, v);
      treeIndices.push_back(index);
      if (++count == n - 1) break;
    }
  }
  return treeIndices;
}

}  // namespace gtsam::utils
