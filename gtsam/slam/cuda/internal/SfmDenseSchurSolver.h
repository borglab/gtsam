/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmDenseSchurSolver.h
 * @brief   Dense Cholesky solve of the reduced camera Schur system
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/nonlinear/cuda/internal/DeviceValues.h>
#include <gtsam/slam/cuda/internal/SfmProjectionBatch.h>
#include <gtsam/slam/cuda/internal/SfmProjectionLinearization.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <memory>

namespace gtsam::cuda {

class GTSAM_EXPORT SfmDenseSchurSolver {
 public:
  SfmDenseSchurSolver();
  ~SfmDenseSchurSolver();

  SfmDenseSchurSolver(const SfmDenseSchurSolver&) = delete;
  SfmDenseSchurSolver& operator=(const SfmDenseSchurSolver&) = delete;
  SfmDenseSchurSolver(SfmDenseSchurSolver&&) noexcept;
  SfmDenseSchurSolver& operator=(SfmDenseSchurSolver&&) noexcept;

  /// Explicit outer-iteration lifecycle used by the SFM optimizer.
  void linearize(const DeviceValues& values,
                 const SfmProjectionBatch& batch, int numCameras,
                 cudaStream_t stream = nullptr);
  void solveLinearized(double lambda, DeviceArray<double>* delta,
                       cudaStream_t stream = nullptr);
  void solveLinearized(double lambda,
                       const DeviceArray<double>& dampingDiagonal,
                       DeviceArray<double>* delta,
                       cudaStream_t stream = nullptr);

  size_t linearizationCount() const;
  size_t denseAssemblyCount() const;
  const SfmProjectionLinearization& linearization() const;
  const LinearSolveStats& linearSolveStats() const;

  void solve(const DeviceValues& values, const SfmProjectionBatch& batch,
             int numCameras, double lambda, DeviceArray<double>* delta,
             cudaStream_t stream = nullptr);
  void solve(const DeviceValues& values, const SfmProjectionBatch& batch,
             int numCameras, double lambda,
             const DeviceArray<double>& dampingDiagonal,
             DeviceArray<double>* delta, cudaStream_t stream = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

GTSAM_EXPORT void solveSfmDenseSchur(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, double lambda, DeviceArray<double>* delta,
    cudaStream_t stream = nullptr);

GTSAM_EXPORT void solveSfmDenseSchur(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, double lambda,
    const DeviceArray<double>& dampingDiagonal,
    DeviceArray<double>* delta, cudaStream_t stream = nullptr);

}  // namespace gtsam::cuda
