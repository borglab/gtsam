/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmReducedCsrPlan.h
 * @brief   Symbolic plan for the upper-triangular camera Schur complement
 * @author  Ruogu Li
 * @date    Aug 16, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/KeyInfo.h>
#include <gtsam/sfm/SfmData.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace gtsam::cuda {

/**
 * Stable symbolic plan for the upper-triangular camera Schur complement.
 *
 * The plan depends only on track co-visibility. Every camera contributes a
 * contiguous 9-scalar block, diagonal blocks store their upper triangle, and
 * off-diagonal blocks store all 9x9 entries. Its CSR structure can therefore
 * be analyzed once and reused across LM iterations and damping retries.
 */
class GTSAM_EXPORT SfmReducedCsrPlan {
 public:
  SfmReducedCsrPlan(const SfmData& data,
                        const std::vector<Key>& cameraKeys);

  int dimension() const { return dimension_; }
  const std::vector<int>& rowPointers() const { return rowPointers_; }
  const std::vector<int>& columnIndices() const { return columnIndices_; }
  const KeyInfo& cameraKeyInfo() const { return cameraKeyInfo_; }

  bool hasCameraPair(int cameraI, int cameraJ) const;

  /// Return the CSR value offset for an entry in a symmetric camera block.
  int valueOffset(int cameraI, int cameraJ, int localRow,
                  int localColumn) const;

  /// Fill-reducing camera-key ordering derived from the co-visibility graph.
  Ordering colamdOrdering() const;

 private:
  static std::uint64_t pairKey(int first, int second);
  static std::uint64_t scalarKey(int row, int column);

  int cameraCount_ = 0;
  int dimension_ = 0;
  std::vector<Key> cameraKeys_;
  KeyInfo cameraKeyInfo_;
  std::vector<int> rowPointers_;
  std::vector<int> columnIndices_;
  std::unordered_map<std::uint64_t, int> scalarOffsets_;
  std::unordered_map<std::uint64_t, bool> cameraPairs_;
};

}  // namespace gtsam::cuda
