/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BlockOrdering.cpp
 * @brief   KeyInfo adapters for CUDA scalar indexing and ordering
 * @author  Ruogu Li
 * @date    Aug 15, 2026
 */

#include <gtsam/linear/cuda/internal/BlockOrdering.h>

#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace gtsam::cuda {
namespace {

int checkedCudaIndex(size_t value) {
  if (value > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("CUDA scalar dimension overflows int");
  }
  return static_cast<int>(value);
}

size_t validateKeyInfo(const KeyInfo& keyInfo) {
  const Ordering& ordering = keyInfo.ordering();
  if (ordering.size() != keyInfo.size()) {
    throw std::invalid_argument(
        "CUDA KeyInfo ordering must contain every key exactly once");
  }

  std::unordered_set<Key> consumed;
  consumed.reserve(ordering.size());
  size_t nextOffset = 0;
  for (size_t index = 0; index < ordering.size(); ++index) {
    const Key key = ordering[index];
    const auto found = keyInfo.find(key);
    if (found == keyInfo.end()) {
      throw std::invalid_argument(
          "CUDA KeyInfo ordering contains an unknown key");
    }
    if (!consumed.insert(key).second) {
      throw std::invalid_argument(
          "CUDA KeyInfo ordering contains a duplicate key");
    }

    const KeyInfoEntry& entry = found->second;
    if (entry.index != index) {
      throw std::invalid_argument("CUDA KeyInfo contains an invalid block index");
    }
    if (entry.start != nextOffset) {
      throw std::invalid_argument(
          "CUDA KeyInfo must cover a contiguous scalar interval");
    }
    if (entry.dim == 0) {
      throw std::invalid_argument("CUDA KeyInfo dimensions must be positive");
    }
    if (entry.dim > std::numeric_limits<size_t>::max() - nextOffset) {
      throw std::invalid_argument(
          "CUDA KeyInfo scalar dimension overflows size_t");
    }
    nextOffset += entry.dim;
    checkedCudaIndex(nextOffset);
  }
  if (nextOffset != keyInfo.numCols()) {
    throw std::invalid_argument(
        "CUDA KeyInfo scalar dimension does not match its entries");
  }
  return nextOffset;
}

}  // namespace

std::vector<int> cudaBlockOffsets(const KeyInfo& keyInfo) {
  validateKeyInfo(keyInfo);
  std::vector<int> offsets;
  offsets.reserve(keyInfo.size() + 1);
  offsets.push_back(0);
  for (const Key key : keyInfo.ordering()) {
    const KeyInfoEntry& entry = keyInfo.at(key);
    offsets.push_back(checkedCudaIndex(entry.start + entry.dim));
  }
  return offsets;
}

std::vector<int> compileScalarPermutation(
    const KeyInfo& keyInfo, const Ordering& ordering) {
  const size_t scalarDimension = validateKeyInfo(keyInfo);
  if (ordering.size() != keyInfo.size()) {
    throw std::invalid_argument(
        "CUDA ordering must contain every block key exactly once");
  }

  std::unordered_set<Key> consumed;
  consumed.reserve(ordering.size());
  std::vector<int> permutation;
  permutation.reserve(scalarDimension);

  for (const Key key : ordering) {
    const auto found = keyInfo.find(key);
    if (found == keyInfo.end()) {
      throw std::invalid_argument("CUDA ordering contains an unknown key");
    }
    if (!consumed.insert(key).second) {
      throw std::invalid_argument("CUDA ordering contains a duplicate key");
    }
    const KeyInfoEntry& entry = found->second;
    for (size_t scalar = 0; scalar < entry.dim; ++scalar) {
      permutation.push_back(checkedCudaIndex(entry.start + scalar));
    }
  }

  if (permutation.size() != scalarDimension) {
    throw std::invalid_argument(
        "CUDA ordering does not cover the scalar dimension");
  }
  return permutation;
}

}  // namespace gtsam::cuda
