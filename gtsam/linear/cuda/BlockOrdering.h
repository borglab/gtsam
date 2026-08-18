#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/inference/Ordering.h>

#include <vector>

namespace gtsam::cuda {

struct VariableBlock {
  Key key = 0;
  int scalarOffset = 0;
  int dimension = 0;
};

using BlockLayout = std::vector<VariableBlock>;

/** validate that blocks uniquely cover the contiguous scalar interval [0,n). */
GTSAM_EXPORT int validateBlockLayout(const BlockLayout& blocks);

/** Expand a key ordering by appending every scalar in each ordered block. */
GTSAM_EXPORT std::vector<int> compileScalarPermutation(
    const BlockLayout& blocks, const Ordering& ordering);

}  // namespace gtsam::cuda
