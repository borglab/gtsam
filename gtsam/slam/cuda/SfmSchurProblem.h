#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/LinearSystem.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/slam/cuda/SfmProjectionBatch.h>
#include <gtsam/slam/cuda/SfmProjectionLinearization.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <memory>

namespace gtsam::cuda {

class SfmReducedCsrPlan;

struct SfmImplicitSchurView {
  const LinearOperator* linearOperator = nullptr;
  const Preconditioner* preconditioner = nullptr;
  const double* rhs = nullptr;
  int dimension = 0;
};

/** Undamped normal-equation blocks retained for one SFM linearization. */
struct SfmSchurBlocks {
  /** Per-camera U = Jc'Jc blocks, row-major 9-by-9. */
  DeviceArray<double> cameraNormalBlocks;
  /** Per-camera gc = -Jc'r vectors. */
  DeviceArray<double> cameraGradient;
  /** Per-point V = Jp'Jp blocks, row-major 3-by-3. */
  DeviceArray<double> pointNormalBlocks;
  /** Per-point gp = -Jp'r vectors. */
  DeviceArray<double> pointGradient;
  /** Per-observation W = Jc'Jp blocks, row-major 9-by-3. */
  DeviceArray<double> cameraPointBlocks;
};

/**
 * Persistent SFM-specific producer for reduced camera Schur systems.
 *
 * Projection residuals and Jacobians are linearized once per LM outer
 * iteration. A backend may then prepare a fresh lambda-dependent system for
 * every damping attempt and use the same state to recover point increments.
 */
class GTSAM_EXPORT SfmSchurProblem {
 public:
  SfmSchurProblem();
  ~SfmSchurProblem();

  SfmSchurProblem(const SfmSchurProblem&) = delete;
  SfmSchurProblem& operator=(const SfmSchurProblem&) = delete;
  SfmSchurProblem(SfmSchurProblem&&) noexcept;
  SfmSchurProblem& operator=(SfmSchurProblem&&) noexcept;

  void initialize(const SfmProjectionBatch& batch, int numCameras);
  void linearize(const DeviceValues& values, cudaStream_t stream = nullptr);

  DenseSpdSystemView prepareDense(double lambda,
                                      cudaStream_t stream = nullptr);
  DenseSpdSystemView prepareDense(
      double lambda, const DeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);

  DeviceSparseSpdSystem& prepareSparse(
      double lambda, const SfmReducedCsrPlan& plan,
      cudaStream_t stream = nullptr);
  DeviceSparseSpdSystem& prepareSparse(
      double lambda, const DeviceArray<double>& dampingDiagonal,
      const SfmReducedCsrPlan& plan, cudaStream_t stream = nullptr);

  SfmImplicitSchurView prepareImplicit(
      double lambda, cudaStream_t stream = nullptr);
  SfmImplicitSchurView prepareImplicit(
      double lambda, const DeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);

  void recoverPoints(double lambda,
                     const DeviceArray<double>& cameraDelta,
                     DeviceArray<double>* fullDelta,
                     cudaStream_t stream = nullptr);
  void recoverPoints(double lambda,
                     const DeviceArray<double>& dampingDiagonal,
                     const DeviceArray<double>& cameraDelta,
                     DeviceArray<double>* fullDelta,
                     cudaStream_t stream = nullptr);

  int cameraDimension() const;
  int totalDimension() const;
  size_t linearizationCount() const;
  size_t blockBuildCount() const;
  size_t denseAssemblyCount() const;
  const SfmProjectionLinearization& linearization() const;
  const SfmSchurBlocks& blocks() const;

  /** Borrow the current condensed RHS (overwritten in-place by dense solve). */
  const DeviceArray<double>& cameraRhs() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
