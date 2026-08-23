/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BatchHessianMapping.h
 * @brief Cached destinations for compact batch-factor elimination.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/types.h>

#include <vector>

namespace gtsam::internal {

/**
 * Flattened block and scalar destinations for every compact row group.
 *
 * `blockSlots` is always populated. `scalarOffsets` is either empty or has the
 * same size, so callers cannot accidentally pair slots with offsets from a
 * different load plan or destination matrix.
 */
struct BatchHessianMapping {
  std::vector<DenseIndex> blockSlots;
  std::vector<DenseIndex> scalarOffsets;

  /// Return whether this mapping contains no row-group destinations.
  bool empty() const { return blockSlots.empty(); }

  /// Return whether scalar destinations have been cached.
  bool hasScalarOffsets() const {
    return !scalarOffsets.empty() && scalarOffsets.size() == blockSlots.size();
  }
};

}  // namespace gtsam::internal
