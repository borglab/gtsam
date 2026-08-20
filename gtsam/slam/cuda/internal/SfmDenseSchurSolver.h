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
