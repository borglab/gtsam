/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   KeyInfo.h
 * @brief  Ordered key dimensions and scalar offsets
 * @date   2026
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <gtsam/inference/Ordering.h>

#include <map>
#include <vector>

namespace gtsam {

class GaussianFactorGraph;
class VectorValues;

/// Position, dimension, and scalar offset of one key in an ordered vector.
struct GTSAM_EXPORT KeyInfoEntry {
  size_t index = 0;
  size_t dim = 0;
  size_t start = 0;

  KeyInfoEntry() = default;
  KeyInfoEntry(size_t index, size_t dimension, size_t start)
      : index(index), dim(dimension), start(start) {}
};

/**
 * Ordered key dimensions and offsets in one flattened variable vector.
 *
 * The map provides lookup by key. `ordering()` defines block order, each entry
 * stores the matching order index and scalar prefix, and `numCols()` is the
 * sum of all block dimensions.
 */
class GTSAM_EXPORT KeyInfo : public std::map<Key, KeyInfoEntry> {
 public:
  using Base = std::map<Key, KeyInfoEntry>;

  /// Construct an empty layout.
  KeyInfo() = default;

  /// Construct from a Gaussian factor graph in natural key order.
  explicit KeyInfo(const GaussianFactorGraph& graph);

  /// Construct from a graph in the supplied complete ordering.
  KeyInfo(const GaussianFactorGraph& graph, const Ordering& ordering);

  /// Construct from dimensions in ascending key order.
  explicit KeyInfo(const std::map<Key, size_t>& dimensions);

  /// Construct from dimensions in the supplied complete ordering.
  KeyInfo(const std::map<Key, size_t>& dimensions, const Ordering& ordering);

  /// Return the total scalar dimension.
  size_t numCols() const { return numCols_; }

  /// Return the key block ordering.
  const Ordering& ordering() const { return ordering_; }

  /// Return block dimensions in `ordering()` order.
  std::vector<size_t> colSpec() const;

  /// Return zero-valued keyed blocks with these dimensions.
  VectorValues x0() const;

  /// Return one zero-valued flat vector with total dimension `numCols()`.
  Vector x0vector() const;

 private:
  Ordering ordering_;
  size_t numCols_ = 0;

  void initialize(const std::map<Key, size_t>& dimensions);
};

/**
 * Split a flat vector into keyed blocks using KeyInfo offsets and dimensions.
 *
 * @throws std::invalid_argument if the flat dimension differs from
 * `keyInfo.numCols()`.
 */
GTSAM_EXPORT VectorValues buildVectorValues(const Vector& vector,
                                            const KeyInfo& keyInfo);

}  // namespace gtsam
