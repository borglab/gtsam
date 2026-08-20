#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/cuda/internal/BlockOrdering.h>
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
  const BlockLayout& cameraBlocks() const { return cameraBlocks_; }

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
  BlockLayout cameraBlocks_;
  std::vector<int> rowPointers_;
  std::vector<int> columnIndices_;
  std::unordered_map<std::uint64_t, int> scalarOffsets_;
  std::unordered_map<std::uint64_t, bool> cameraPairs_;
};

}  // namespace gtsam::cuda
