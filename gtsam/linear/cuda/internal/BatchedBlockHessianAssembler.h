/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedBlockHessianAssembler.h
 * @brief   Batched GPU formation of block-tridiagonal normal-equation
 *          pieces from B dense Jacobians, via cuBLAS batched GEMM
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/linear/cuda/internal/BatchedDenseCholeskySolver.h>

#include <memory>

namespace gtsam::cuda {

/**
 * Forms diag[i][t] = At^T At, offDiag[i][t] = At^T At+1, and
 * rhs[i] = A_i^T b_i for B independent dense Jacobians of identical
 * row/column shape, via cuBLAS batched GEMM -- replaces a per-item,
 * per-block CPU Eigen matmul loop with a handful of GPU kernel launches.
 */
class BatchedBlockHessianAssembler {
 public:
  /// Calls cudaSetDevice(options.deviceId) first when deviceId >= 0, then
  /// creates the cuBLAS handle on that device.
  explicit BatchedBlockHessianAssembler(
      const BatchedDenseSolverOptions& options = {});
  ~BatchedBlockHessianAssembler();

  BatchedBlockHessianAssembler(const BatchedBlockHessianAssembler&) = delete;
  BatchedBlockHessianAssembler& operator=(
      const BatchedBlockHessianAssembler&) = delete;
  BatchedBlockHessianAssembler(BatchedBlockHessianAssembler&&) noexcept;
  BatchedBlockHessianAssembler& operator=(
      BatchedBlockHessianAssembler&&) noexcept;

  /**
   * deviceA holds batchSize dense Jacobians (jacobianRows x numBlocks*
   * blockDim each, column-major, batch-item-major contiguous); deviceB
   * holds batchSize RHS vectors (jacobianRows each, same layout). Writes
   * diagOut (batchSize*numBlocks*blockDim*blockDim), offDiagOut
   * (batchSize*(numBlocks-1)*blockDim*blockDim), and rhsOut (batchSize*
   * numBlocks*blockDim), all batch-item-major -- the exact layout
   * BatchedBlockTridiagonalCholeskySolver::factorize() expects. Throws on
   * a malformed batch.
   */
  void assemble(int batchSize, int numBlocks, int blockDim, int jacobianRows,
                DeviceArray<double>* deviceA, DeviceArray<double>* deviceB,
                DeviceArray<double>* diagOut,
                DeviceArray<double>* offDiagOut,
                DeviceArray<double>* rhsOut, cudaStream_t stream = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
