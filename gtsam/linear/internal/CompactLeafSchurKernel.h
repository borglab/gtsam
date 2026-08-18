/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CompactLeafSchurKernel.h
 * @brief Internal allocation-free kernels for compact leaf elimination.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/dllexport.h>

#include <cstddef>
#include <vector>

namespace gtsam {

class SymmetricBlockMatrix;
class VerticalBlockMatrix;

namespace internal {

/** Small numerical kernels used by fused one-frontal-block leaf elimination. */
class GTSAM_EXPORT CompactLeafSchurKernel {
 public:
  /**
   * Factor the frontal block in place and solve its separator rows.
   *
   * On success, `frontalRows` changes from `[Hff Hfs gf]` to `[R S d]`,
   * where `R'R=Hff` and `R'[S d]=[Hfs gf]`.
   */
  static bool factorFrontalRows(VerticalBlockMatrix* frontalRows,
                                DenseIndex frontalDimension);

  /** Expand block scalar offsets into one destination per scalar column. */
  static std::vector<DenseIndex> expandScalarOffsets(
      const std::vector<size_t>& blockDimensions,
      const std::vector<DenseIndex>& blockScalarOffsets);

  /**
   * Subtract the upper triangle of `[S d]'[S d]` directly into `target`.
   *
   * `targetScalarOffsets` maps every scalar column after the frontal columns
   * in `frontalRows` to its scalar destination in `target`.
   */
  static void subtractMappedOuterProduct(
      const VerticalBlockMatrix& frontalRows, DenseIndex frontalDimension,
      const std::vector<DenseIndex>& targetScalarOffsets,
      SymmetricBlockMatrix* target);
};

}  // namespace internal
}  // namespace gtsam
