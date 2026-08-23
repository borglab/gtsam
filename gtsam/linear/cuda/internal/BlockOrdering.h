/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BlockOrdering.h
 * @brief   KeyInfo adapters for CUDA scalar indexing and ordering
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/KeyInfo.h>

#include <vector>

namespace gtsam::cuda {

/// Convert KeyInfo prefix sums to the int32 offsets consumed by CUDA APIs.
GTSAM_EXPORT std::vector<int> cudaBlockOffsets(const KeyInfo& keyInfo);

/// Expand a key ordering by appending every scalar in each ordered block.
GTSAM_EXPORT std::vector<int> compileScalarPermutation(
    const KeyInfo& keyInfo, const Ordering& ordering);

}  // namespace gtsam::cuda
