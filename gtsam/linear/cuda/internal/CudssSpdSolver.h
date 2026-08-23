/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CudssSpdSolver.h
 * @brief   Reusable cuDSS analysis, factorization, and solve for one pattern
 * @author  Ruogu Li
 * @date    Jun 17, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>

#include <memory>
#include <vector>

namespace gtsam::cuda {

/// Reusable cuDSS analysis, factorization, and solve state for one CSR pattern.
class CudssSpdSolver {
 public:
  CudssSpdSolver();
  ~CudssSpdSolver();

  CudssSpdSolver(const CudssSpdSolver&) = delete;
  CudssSpdSolver& operator=(const CudssSpdSolver&) = delete;
  CudssSpdSolver(CudssSpdSolver&&) noexcept;
  CudssSpdSolver& operator=(CudssSpdSolver&&) noexcept;

  /// Analyze an SPD CSR pattern using cuDSS automatic ordering.
  void analyze(const DeviceSparseSpdSystem& system,
               DeviceArray<double>* solution,
               cudaStream_t stream = nullptr);
  /// Analyze an SPD CSR pattern using a caller-supplied scalar permutation.
  void analyze(const DeviceSparseSpdSystem& system,
               DeviceArray<double>* solution,
               const std::vector<int>& scalarPermutation,
               cudaStream_t stream = nullptr);
  /**
   * Numerically factor and solve the analyzed SPD system.
   *
   * Throws std::runtime_error when cuDSS reports a non-positive minor during
   * numerical factorization.
   */
  void solve(const DeviceSparseSpdSystem& system,
             DeviceArray<double>* solution, cudaStream_t stream = nullptr);

  /// Returns cumulative analysis, factorization, solve, and boundary timings.
  const LinearSolveStats& stats() const;
  /// Returns the scalar permutation retained by cuDSS analysis.
  const std::vector<int>& appliedPermutation() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
