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

/// Persistent cuSOLVER DN Cholesky backend for column-major SPD systems.
class DenseCholeskySolver {
 public:
  DenseCholeskySolver();
  ~DenseCholeskySolver();

  DenseCholeskySolver(const DenseCholeskySolver&) = delete;
  DenseCholeskySolver& operator=(const DenseCholeskySolver&) = delete;
  DenseCholeskySolver(DenseCholeskySolver&&) noexcept;
  DenseCholeskySolver& operator=(DenseCholeskySolver&&) noexcept;

  void analyze(int maximumDimension, cudaStream_t stream = nullptr);
  void solveInPlace(DenseSpdSystemView system,
                    cudaStream_t stream = nullptr,
                    LinearSolveStats* stats = nullptr);
  void solve(DenseSpdSystemView system,
             DeviceArray<double>* solution,
             cudaStream_t stream = nullptr,
             LinearSolveStats* stats = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
