/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DenseCholeskySolver.h
 * @brief   Persistent cuSOLVER DN Cholesky backend for dense SPD systems
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/linear/cuda/LinearSolver.h>

#include <memory>

namespace gtsam::cuda {

/**
 * Dense Cholesky backend: cuSOLVER DN potrf then potrs, on the lower triangle
 * of a column-major matrix with one right-hand side.
 *
 * The reason this is a class rather than two free calls is state that is
 * expensive to build and cheap to keep. A cusolverDnHandle_t is expensive to
 * create, and the factorization workspace is a device allocation whose size
 * depends only on the dimension. Both are created once and reused across the
 * many solves a Levenberg-Marquardt run performs, so a repeated solve at fixed
 * dimension does no allocation at all.
 *
 * There is no symbolic phase to speak of: dense Cholesky has no fill-in to
 * predict and no ordering to choose. analyze() therefore only binds the stream
 * and records a dimension, and exists so that this backend presents the same
 * analyze/solve lifecycle as the sparse ones and LinearSolverSession can select
 * between them uniformly. The workspace is sized in solveInPlace() from what
 * cuSOLVER asks for, so analyze() may be skipped.
 *
 * Dense storage costs O(n²) memory and O(n³) work, so this is for small systems
 * and as the straightforward reference the sparse backends are checked against;
 * anything of graph size belongs on CudssSpdSolver or PcgSolver.
 *
 * Both solve entry points detect a matrix that is not positive definite through
 * cuSOLVER's device info value and throw. That check reads the value back to the
 * host, so each factorization synchronizes the stream once.
 */
class DenseCholeskySolver {
 public:
  /// Creates the cuSOLVER handle; allocates no workspace yet.
  DenseCholeskySolver();
  ~DenseCholeskySolver();

  DenseCholeskySolver(const DenseCholeskySolver&) = delete;
  DenseCholeskySolver& operator=(const DenseCholeskySolver&) = delete;
  DenseCholeskySolver(DenseCholeskySolver&&) noexcept;
  DenseCholeskySolver& operator=(DenseCholeskySolver&&) noexcept;

  /// Binds the stream and records the dimension to expect; allocates nothing.
  void analyze(int maximumDimension, cudaStream_t stream = nullptr);

  /**
   * Factors and solves, overwriting the system: values becomes the Cholesky
   * factor and rhs becomes the solution.
   *
   * Throws std::invalid_argument on an empty or malformed view and
   * std::runtime_error if the matrix is not positive definite. stats, when
   * given, is accumulated into rather than overwritten.
   */
  void solveInPlace(DenseSpdSystemView system,
                    cudaStream_t stream = nullptr,
                    LinearSolveStats* stats = nullptr);

  /// As solveInPlace(), but also copies the solution into solution, resizing it.
  void solve(DenseSpdSystemView system,
             DeviceArray<double>* solution,
             cudaStream_t stream = nullptr,
             LinearSolveStats* stats = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
