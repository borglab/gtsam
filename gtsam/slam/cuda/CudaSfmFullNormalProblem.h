#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/CudaLinearSystem.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace gtsam::cuda {

/** Borrowed matrix-free full-normal system prepared for one LM damping value. */
struct CudaSfmFullNormalView {
  const CudaLinearOperator* linearOperator = nullptr;
  const CudaPreconditioner* preconditioner = nullptr;
  const double* rhs = nullptr;
};

/**
 * Produces the SFM full normal operator J'J + lambda D without materializing
 * CSR storage. Projection Jacobians are borrowed from the current outer
 * linearization; RHS and camera/point block-Jacobi storage persist across
 * lambda attempts.
 */
class GTSAM_EXPORT CudaSfmFullNormalProblem {
 public:
  CudaSfmFullNormalProblem();
  ~CudaSfmFullNormalProblem();

  CudaSfmFullNormalProblem(const CudaSfmFullNormalProblem&) = delete;
  CudaSfmFullNormalProblem& operator=(const CudaSfmFullNormalProblem&) = delete;
  CudaSfmFullNormalProblem(CudaSfmFullNormalProblem&&) noexcept;
  CudaSfmFullNormalProblem& operator=(CudaSfmFullNormalProblem&&) noexcept;

  void initialize(const CudaSfmProjectionBatch& batch, int numCameras,
                  cudaStream_t stream = nullptr);
  void initializeSparse(
      const CudaSfmProjectionBatch& batch, int numCameras,
      const std::vector<int>& rowPointers,
      const std::vector<int>& columnIndices, cudaStream_t stream = nullptr,
      CudaDeviceTransferSummary* transferProfile = nullptr);
  void linearize(const CudaSfmProjectionLinearization& linearization,
                 cudaStream_t stream = nullptr);
  CudaSfmFullNormalView prepare(double lambda,
                                cudaStream_t stream = nullptr);
  CudaSfmFullNormalView prepare(
      double lambda, const CudaDeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);
  DeviceSparseSpdSystem& prepareSparse(
      double lambda, cudaStream_t stream = nullptr);
  DeviceSparseSpdSystem& prepareSparse(
      double lambda, const CudaDeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);

  int dimension() const;
  size_t linearizationCount() const;
  size_t preparationCount() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
