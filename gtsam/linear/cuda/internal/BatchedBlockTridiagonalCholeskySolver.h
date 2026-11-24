/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedBlockTridiagonalCholeskySolver.h
 * @brief   Persistent cuSOLVER/cuBLAS batched block-Cholesky backend for B
 *          independent block-tridiagonal SPD systems of the same shape
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/linear/cuda/internal/BatchedSolverOptions.h>

#include <memory>

namespace gtsam::cuda {

/**
 * Batched block-tridiagonal Cholesky backend (block Thomas algorithm) for
 * B independent systems whose scalar dimension partitions into numBlocks
 * blockDim-wide blocks, each factor/RHS touching only one or two adjacent
 * blocks -- e.g. DFG-TAMP's window graphs. Exploits the chain sparsity
 * directly instead of factorizing a dense n x n Hessian, reducing
 * factorize cost to O(numBlocks * blockDim^3).
 */
class BatchedBlockTridiagonalCholeskySolver {
 public:
  /// Calls cudaSetDevice(options.deviceId) first when deviceId >= 0, then
  /// creates the cuSOLVER and cuBLAS handles on that device.
  explicit BatchedBlockTridiagonalCholeskySolver(
      const BatchedSolverOptions& options = {});
  ~BatchedBlockTridiagonalCholeskySolver();

  BatchedBlockTridiagonalCholeskySolver(
      const BatchedBlockTridiagonalCholeskySolver&) = delete;
  BatchedBlockTridiagonalCholeskySolver& operator=(
      const BatchedBlockTridiagonalCholeskySolver&) = delete;
  BatchedBlockTridiagonalCholeskySolver(
      BatchedBlockTridiagonalCholeskySolver&&) noexcept;
  BatchedBlockTridiagonalCholeskySolver& operator=(
      BatchedBlockTridiagonalCholeskySolver&&) noexcept;

  /**
   * Factorizes batchSize block-tridiagonal SPD systems (numBlocks diagonal
   * blocks, blockDim x blockDim each) in place: diagonalBlocks/
   * offDiagonalBlocks (upper off-diagonal H_{t,t+1} only) hold batch-item-
   * major contiguous blocks, overwritten with L_tt and Z_t for solve().
   * Throws on a malformed batch or a non-positive-definite block.
   */
  void factorize(int batchSize, int numBlocks, int blockDim,
                 DeviceArray<double>* diagonalBlocks,
                 DeviceArray<double>* offDiagonalBlocks,
                 cudaStream_t stream = nullptr,
                 LinearSolveStats* stats = nullptr);

  /// Solves k RHS columns per item against the cached factorization; rhs is
  /// batch-item-major blockDim*k blocks, overwritten with the solution.
  /// Must follow a matching factorize() call.
  void solve(int k, DeviceArray<double>* rhs, cudaStream_t stream = nullptr,
             LinearSolveStats* stats = nullptr);

  /// Returns the batch size recorded by the most recent factorize() call.
  int batchSize() const;
  /// Returns the block count recorded by the most recent factorize() call.
  int numBlocks() const;
  /// Returns the block dimension recorded by the most recent factorize() call.
  int blockDim() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
