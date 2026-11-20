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
 * @date    Nov 20, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/linear/cuda/LinearSolver.h>

#include <memory>

namespace gtsam::cuda {

/**
 * Options for a batched dense SPD backend.
 *
 * gtsam::cuda has no global or singleton device state anywhere (every handle
 * is owned by the instance that created it), so multi-GPU use is simply one
 * solver instance per GPU, each pinned to its device at construction. deviceId
 * of -1 keeps whatever device is already current for the calling thread,
 * which is the right choice when the caller has already called
 * cudaSetDevice() itself (for example, one worker thread per GPU).
 */
struct BatchedDenseSolverOptions {
  int deviceId = -1;
};

/**
 * Batched dense Cholesky backend: cuSOLVER DN potrfBatched then potrsBatched,
 * factorizing and solving B independent n x n SPD systems in one call each,
 * rather than one system per call.
 *
 * This is the batched counterpart to DenseCholeskySolver. That class solves
 * one dense system per call, which pays full cuSOLVER dispatch/stream
 * overhead every time -- exactly the setup cost CLAUDE.md's GPU-accelerated
 * GTSAM investigation found dominating a single ~256-column solve, making
 * the existing sparse CUDA LM path ~50% slower than CPU for DFG-TAMP's
 * per-transition problem size. DFG-TAMP's JointWindowTrainer does not solve
 * one graph at a time, though -- it solves B=8 structurally identical,
 * independent window graphs per training step. Batching those into one
 * cuSOLVER call amortizes the per-call overhead across the whole batch
 * instead of paying it B times, which is what actually wins on GPU here (see
 * the scratchpad prototype this is built from, recorded in
 * i-want-you-to-dapper-torvalds.md: 2.1x forward / ~parity backward / 1.84x
 * combined over CPU at this exact B=8, n=256 shape).
 *
 * Dense, not sparse: at n ~= 256 columns, DFG-TAMP's window graphs are small
 * enough that dense batched Cholesky is a legitimate substitute for a batched
 * sparse direct solve, and cuSOLVER is already linked whenever
 * GTSAM_ENABLE_CUDA is on -- no new library dependency (cuDSS itself is not
 * installed on this machine; it has no batched multi-system API either way,
 * so a true sparse-cuDSS batched backend would need its own per-system
 * analyze/factorize loop, not a single call, and is future work, not done
 * here).
 *
 * cusolverDnXpotrsBatched accepts only nrhs=1 per call -- a real cuSOLVER API
 * restriction confirmed empirically (CUSOLVER_STATUS_INVALID_VALUE for
 * nrhs>1), not a limitation introduced here. solve() with k>1 right-hand-side
 * columns per batch item therefore issues k such calls, every one reusing the
 * same cached factorization with no refactorization -- functionally the
 * intended "factorize once, solve many RHS" backward pattern, just k cheap
 * calls instead of one fused one.
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

  /**
   * Factorizes batchSize independent dimension x dimension SPD matrices in
   * place (lower triangle, column-major). values holds batchSize contiguous
   * column-major blocks of size dimension*dimension, in batch order; it
   * becomes each item's Cholesky factor. The caller must keep values alive
   * and unmodified until every intended solve() call completes.
   *
   * Throws std::invalid_argument on an empty or malformed batch and
   * std::runtime_error if any batch item is not positive definite.
   */
  void factorize(int batchSize, int dimension, DeviceArray<double>* values,
                 cudaStream_t stream = nullptr,
                 LinearSolveStats* stats = nullptr);

  /**
   * Solves for k right-hand-side columns per batch item against the cached
   * factorization, with no refactorization. rhs holds batchSize contiguous
   * blocks of size dimension*k (k column-major columns of length dimension
   * each); overwritten in place with the solution. Must follow a matching
   * factorize() call.
   */
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
