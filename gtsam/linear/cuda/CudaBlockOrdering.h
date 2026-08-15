#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/inference/Ordering.h>

#include <vector>

namespace gtsam::cuda {

struct CudaVariableBlock {
  Key key = 0;
  int scalarOffset = 0;
  int dimension = 0;
};

using CudaBlockLayout = std::vector<CudaVariableBlock>;

/** Validate that blocks uniquely cover the contiguous scalar interval [0,n). */
GTSAM_EXPORT int ValidateCudaBlockLayout(const CudaBlockLayout& blocks);

/** Expand a key ordering by appending every scalar in each ordered block. */
GTSAM_EXPORT std::vector<int> CompileCudaScalarPermutation(
    const CudaBlockLayout& blocks, const Ordering& ordering);

}  // namespace gtsam::cuda
