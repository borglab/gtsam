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

/**
 * Borrowed matrix-free full-normal system prepared for one LM damping value.
 *
 * All pointers refer to storage owned by the producing SfmFullNormalProblem and
 * remain valid only until that producer is prepared, linearized, initialized,
 * moved, or destroyed again. Consumers must enqueue work on the same stream or
 * establish an explicit CUDA dependency before using the view elsewhere.
 */
struct SfmFullNormalView {
  /** Matrix-free damped normal-equation operator. */
  const LinearOperator* linearOperator = nullptr;
  /** Block-Jacobi preconditioner associated with linearOperator. */
  const Preconditioner* preconditioner = nullptr;
  /** Device pointer to the right-hand side. */
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
  /** Constructs an empty producer; call initialize before linearize. */
  SfmFullNormalProblem();
  ~SfmFullNormalProblem();

  SfmFullNormalProblem(const SfmFullNormalProblem&) = delete;
  SfmFullNormalProblem& operator=(const SfmFullNormalProblem&) = delete;
  SfmFullNormalProblem(SfmFullNormalProblem&&) noexcept;
  SfmFullNormalProblem& operator=(SfmFullNormalProblem&&) noexcept;

  /**
   * Initializes matrix-free storage for batch.
   *
   * batch is borrowed by subsequent linearizations and must outlive this
   * object. Initialization and later operations must be ordered on stream.
   */
  void initialize(const SfmProjectionBatch& batch, int numCameras,
                  cudaStream_t stream = nullptr);
  /**
   * Initializes matrix-free and explicit upper-CSR storage.
   *
   * The host pattern is copied before return; batch remains borrowed. Optional
   * transfer accounting is updated for the queued pattern uploads.
   */
  void initializeSparse(
      const SfmProjectionBatch& batch, int numCameras,
      const std::vector<int>& rowPointers,
      const std::vector<int>& columnIndices, cudaStream_t stream = nullptr,
      DeviceTransferSummary* transferProfile = nullptr);
  /**
   * Builds the undamped right-hand side and block-Jacobi terms.
   *
   * linearization is borrowed until the next call to linearize or initialize
   * and must remain alive while prepared views are consumed.
   */
  void linearize(const SfmProjectionLinearization& linearization,
                 cudaStream_t stream = nullptr);
  /** Prepares and returns a borrowed matrix-free system with scalar damping. */
  SfmFullNormalView prepare(double lambda,
                            cudaStream_t stream = nullptr);
  /**
   * Prepares a borrowed matrix-free system with element-wise damping.
   * dampingDiagonal must contain dimension() entries on the device.
   */
  SfmFullNormalView prepare(
      double lambda, const DeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);
  /**
   * Assembles scalar-damped upper CSR and returns producer-owned storage.
   * The reference is invalidated by reinitialization or object destruction.
   */
  DeviceSparseSpdSystem& prepareSparse(
      double lambda, cudaStream_t stream = nullptr);
  /** Assembles diagonally damped upper CSR in producer-owned storage. */
  DeviceSparseSpdSystem& prepareSparse(
      double lambda, const DeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);

  /** Returns the scalar dimension of the full camera-and-point system. */
  int dimension() const;
  /** Returns the number of successful linearize calls. */
  size_t linearizationCount() const;
  /** Returns the number of matrix-free or sparse prepare calls. */
  size_t preparationCount() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
