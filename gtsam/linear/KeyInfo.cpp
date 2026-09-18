/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   KeyInfo.cpp
 * @brief  Ordered key dimensions and scalar offsets
 * @date   2026
 * @author Frank Dellaert
 */

#include <gtsam/linear/KeyInfo.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <limits>
#include <stdexcept>

namespace gtsam {
namespace {

Ordering naturalOrdering(const std::map<Key, size_t>& dimensions) {
  Ordering ordering;
  ordering.reserve(dimensions.size());
  for (const auto& [key, dimension] : dimensions) {
    (void)dimension;
    ordering.push_back(key);
  }
  return ordering;
}

}  // namespace

KeyInfo::KeyInfo(const GaussianFactorGraph& graph,
                 const Ordering& ordering)
    : ordering_(ordering) {
  initialize(graph.getKeyDimMap());
}

KeyInfo::KeyInfo(const GaussianFactorGraph& graph)
    : ordering_(Ordering::Natural(graph)) {
  initialize(graph.getKeyDimMap());
}

KeyInfo::KeyInfo(const std::map<Key, size_t>& dimensions)
    : ordering_(naturalOrdering(dimensions)) {
  initialize(dimensions);
}

KeyInfo::KeyInfo(const std::map<Key, size_t>& dimensions,
                 const Ordering& ordering)
    : ordering_(ordering) {
  initialize(dimensions);
}

void KeyInfo::initialize(const std::map<Key, size_t>& dimensions) {
  if (ordering_.size() != dimensions.size()) {
    throw std::invalid_argument(
        "KeyInfo ordering must contain every dimension key exactly once");
  }

  size_t start = 0;
  for (size_t index = 0; index < ordering_.size(); ++index) {
    const Key key = ordering_[index];
    const auto found = dimensions.find(key);
    if (found == dimensions.end()) {
      throw std::invalid_argument("KeyInfo ordering contains an unknown key");
    }
    if (found->second > std::numeric_limits<size_t>::max() - start) {
      throw std::overflow_error("KeyInfo scalar dimension overflows size_t");
    }
    if (!emplace(key, KeyInfoEntry(index, found->second, start)).second) {
      throw std::invalid_argument("KeyInfo ordering contains a duplicate key");
    }
    start += found->second;
  }
  numCols_ = start;
}

std::vector<size_t> KeyInfo::colSpec() const {
  std::vector<size_t> result(size(), 0);
  for (const auto& [key, entry] : *this) {
    (void)key;
    result[entry.index] = entry.dim;
  }
  return result;
}

VectorValues KeyInfo::x0() const {
  VectorValues result;
  for (const auto& [key, entry] : *this) {
    result.emplace(key, Vector::Zero(entry.dim));
  }
  return result;
}

Vector KeyInfo::x0vector() const { return Vector::Zero(numCols_); }

VectorValues buildVectorValues(const Vector& vector,
                               const KeyInfo& keyInfo) {
  if (static_cast<size_t>(vector.size()) != keyInfo.numCols()) {
    throw std::invalid_argument(
        "buildVectorValues: flat vector dimension does not match KeyInfo");
  }

  VectorValues result;
  for (const auto& [key, entry] : keyInfo) {
    result.emplace(key, vector.segment(static_cast<Eigen::Index>(entry.start),
                                       static_cast<Eigen::Index>(entry.dim)));
  }
  return result;
}

}  // namespace gtsam
