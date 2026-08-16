#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/CudaLinearSystem.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <memory>

namespace gtsam::cuda {

class CudaSfmReducedCsrPlan;

/**
 * Persistent SFM-specific producer for reduced camera Schur systems.
 *
 * Projection residuals and Jacobians are linearized once per LM outer
 * iteration. A backend may then prepare a fresh lambda-dependent system for
 * every damping attempt and use the same state to recover point increments.
 */
class GTSAM_EXPORT CudaSfmSchurProblem {
 public:
  CudaSfmSchurProblem();
  ~CudaSfmSchurProblem();

  CudaSfmSchurProblem(const CudaSfmSchurProblem&) = delete;
  CudaSfmSchurProblem& operator=(const CudaSfmSchurProblem&) = delete;
  CudaSfmSchurProblem(CudaSfmSchurProblem&&) noexcept;
  CudaSfmSchurProblem& operator=(CudaSfmSchurProblem&&) noexcept;

  void initialize(const CudaSfmProjectionBatch& batch, int numCameras);
  void linearize(const DeviceValues& values, cudaStream_t stream = nullptr);

  CudaDenseSpdSystemView prepareDense(double lambda,
                                      cudaStream_t stream = nullptr);
  CudaDenseSpdSystemView prepareDense(
      double lambda, const CudaDeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);

  DeviceSparseSpdSystem& prepareSparse(
      double lambda, const CudaSfmReducedCsrPlan& plan,
      cudaStream_t stream = nullptr);
  DeviceSparseSpdSystem& prepareSparse(
      double lambda, const CudaDeviceArray<double>& dampingDiagonal,
      const CudaSfmReducedCsrPlan& plan, cudaStream_t stream = nullptr);

  void recoverPoints(double lambda,
                     const CudaDeviceArray<double>& cameraDelta,
                     CudaDeviceArray<double>* fullDelta,
                     cudaStream_t stream = nullptr);
  void recoverPoints(double lambda,
                     const CudaDeviceArray<double>& dampingDiagonal,
                     const CudaDeviceArray<double>& cameraDelta,
                     CudaDeviceArray<double>* fullDelta,
                     cudaStream_t stream = nullptr);

  int cameraDimension() const;
  int totalDimension() const;
  size_t linearizationCount() const;
  size_t denseAssemblyCount() const;

  /** Borrow the current condensed RHS (overwritten in-place by dense solve). */
  const CudaDeviceArray<double>& cameraRhs() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
