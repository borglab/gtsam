/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedDenseCholeskySolver.h
 * @brief   Persistent cuSOLVER DN batched Cholesky backend for B independent
 *          dense SPD systems of the same dimension
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/linear/cuda/LinearSolver.h>

#include <memory>

namespace gtsam::cuda {

/**
 * Options for a batched dense SPD backend. deviceId of -1 keeps whatever
 * device is already current for the calling thread (one worker thread per
 * GPU); >= 0 pins this instance to that device at construction.
 */
struct BatchedDenseSolverOptions {
  int deviceId = -1;
};

/**
 * Batched dense Cholesky (cuSOLVER potrfBatched/potrsBatched): factorizes
 * and solves B independent n x n SPD systems in one call each, amortizing
 * dispatch overhead across the batch. solve() issues k calls for k RHS
 * columns (potrsBatched only accepts nrhs=1).
 */
class BatchedDenseCholeskySolver {
 public:
  /// Calls cudaSetDevice(options.deviceId) first when deviceId >= 0, then
  /// creates the cuSOLVER handle on that device.
  explicit BatchedDenseCholeskySolver(
      const BatchedDenseSolverOptions& options = {});
  ~BatchedDenseCholeskySolver();

  BatchedDenseCholeskySolver(const BatchedDenseCholeskySolver&) = delete;
  BatchedDenseCholeskySolver& operator=(
      const BatchedDenseCholeskySolver&) = delete;
  BatchedDenseCholeskySolver(BatchedDenseCholeskySolver&&) noexcept;
  BatchedDenseCholeskySolver& operator=(
      BatchedDenseCholeskySolver&&) noexcept;

  /// Factorizes batchSize dimension x dimension SPD matrices in place
  /// (values holds batchSize contiguous column-major blocks). Throws on a
  /// malformed batch or a non-positive-definite item.
  void factorize(int batchSize, int dimension, DeviceArray<double>* values,
                 cudaStream_t stream = nullptr,
                 LinearSolveStats* stats = nullptr);

  /// Solves k RHS columns per item against the cached factorization; rhs
  /// is batchSize contiguous dimension*k blocks, overwritten in place.
  void solve(int k, DeviceArray<double>* rhs, cudaStream_t stream = nullptr,
             LinearSolveStats* stats = nullptr);

  /// Returns the batch size recorded by the most recent factorize() call.
  int batchSize() const;
  /// Returns the dimension recorded by the most recent factorize() call.
  int dimension() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
