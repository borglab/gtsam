#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/LinearSystem.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/slam/cuda/SfmProjectionBatch.h>
#include <gtsam/slam/cuda/SfmProjectionLinearization.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace gtsam::cuda {

/** Borrowed matrix-free full-normal system prepared for one LM damping value. */
struct SfmFullNormalView {
  const LinearOperator* linearOperator = nullptr;
  const Preconditioner* preconditioner = nullptr;
  const double* rhs = nullptr;
};

/**
 * Produces the SFM full normal operator J'J + lambda D without materializing
 * CSR storage. Projection Jacobians are borrowed from the current outer
 * linearization; RHS and camera/point block-Jacobi storage persist across
 * lambda attempts.
 */
class GTSAM_EXPORT SfmFullNormalProblem {
 public:
  SfmFullNormalProblem();
  ~SfmFullNormalProblem();

  SfmFullNormalProblem(const SfmFullNormalProblem&) = delete;
  SfmFullNormalProblem& operator=(const SfmFullNormalProblem&) = delete;
  SfmFullNormalProblem(SfmFullNormalProblem&&) noexcept;
  SfmFullNormalProblem& operator=(SfmFullNormalProblem&&) noexcept;

  void initialize(const SfmProjectionBatch& batch, int numCameras,
                  cudaStream_t stream = nullptr);
  void initializeSparse(
      const SfmProjectionBatch& batch, int numCameras,
      const std::vector<int>& rowPointers,
      const std::vector<int>& columnIndices, cudaStream_t stream = nullptr,
      DeviceTransferSummary* transferProfile = nullptr);
  void linearize(const SfmProjectionLinearization& linearization,
                 cudaStream_t stream = nullptr);
  SfmFullNormalView prepare(double lambda,
                                cudaStream_t stream = nullptr);
  SfmFullNormalView prepare(
      double lambda, const DeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);
  DeviceSparseSpdSystem& prepareSparse(
      double lambda, cudaStream_t stream = nullptr);
  DeviceSparseSpdSystem& prepareSparse(
      double lambda, const DeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);

  int dimension() const;
  size_t linearizationCount() const;
  size_t preparationCount() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
