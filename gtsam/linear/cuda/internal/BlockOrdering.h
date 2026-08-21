/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BlockOrdering.h
 * @brief   Variable-block layouts and key-to-scalar ordering expansion
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/inference/Ordering.h>

#include <vector>

namespace gtsam::cuda {

/**
 * One variable's contiguous range of scalar columns in a flattened system.
 *
 * This is the same idea as gtsam::KeyInfoEntry and gtsam::Scatter's SlotEntry,
 * and is deliberately a third spelling of it, for two reasons. KeyInfo can only
 * be built from a GaussianFactorGraph — its ordering and column count are
 * protected with no other way to set them — and the CUDA path never builds one,
 * since it linearizes straight from the nonlinear graph into CSR. Scatter can be
 * built key by key, but leaves offsets implicit, whereas device kernels index
 * CSR with a stored int32 base per block rather than recomputing a prefix sum.
 * The int widths here are the ones cuDSS, cuSPARSE, and the kernels take, so
 * KeyInfoEntry's size_t fields would need a checked narrowing at every use.
 *
 * Making KeyInfo constructible from key/dimension pairs would remove the first
 * of those reasons and let this be a thin adapter over it, but that is a change
 * to core GTSAM rather than to this backend.
 */
struct VariableBlock {
  /// Variable this block belongs to.
  Key key = 0;
  /// First scalar column of the block in the flattened system.
  int scalarOffset = 0;
  /// Tangent-space width of the variable.
  int dimension = 0;
};

/// Blocks of a flattened system, which together tile [0, n) exactly once.
using BlockLayout = std::vector<VariableBlock>;

/// validate that blocks uniquely cover the contiguous scalar interval [0,n).
GTSAM_EXPORT int validateBlockLayout(const BlockLayout& blocks);

/// Expand a key ordering by appending every scalar in each ordered block.
GTSAM_EXPORT std::vector<int> compileScalarPermutation(
    const BlockLayout& blocks, const Ordering& ordering);

}  // namespace gtsam::cuda
