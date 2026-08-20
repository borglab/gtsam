#pragma once

#include <cuda_runtime_api.h>
#include <cusparse.h>
#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/cuda/internal/JacobianNormalOperator.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace gtsam::cuda {

struct DevicePcgOptions {
  int maxIterations = 250;
  double relativeTolerance = 1e-6;
  bool warmStart = true;
  int convergenceCheckInterval = 10;
  DevicePcgPreconditioner preconditioner = DevicePcgPreconditioner::BlockJacobi;
};

/**
 * Produces the matrix-free normal operator and block preconditioner consumed
 * by LinearSolverSession.
 *
 * The operator is applied as two cuSPARSE SpMVs (J·p, then Jᵀ·(J·p)) plus an
 * elementwise damping term; the normal matrix H is never formed. The
 * preconditioner is block-Jacobi over the variable blocks of the column
 * layout: the undamped Gram blocks diag_k(JᵀJ) are rebuilt once per
 * linearization from the Jᵀ CSR arrays. prepare() applies attempt-specific
 * damping before the common session runs the PCG recurrence.
 */
class GTSAM_EXPORT DevicePcgSolver {
 public:
  DevicePcgSolver();
  ~DevicePcgSolver();

  DevicePcgSolver(const DevicePcgSolver&) = delete;
  DevicePcgSolver& operator=(const DevicePcgSolver&) = delete;
  DevicePcgSolver(DevicePcgSolver&&) noexcept;
  DevicePcgSolver& operator=(DevicePcgSolver&&) noexcept;

  /**
   * One-time setup: allocates CG vectors and preconditioner block storage,
   * creates dense-vector descriptors and the SpMV workspace, and validates
   * that each variable block's columns share an identical Jᵀ row pattern
   * (which the sparse-plan construction guarantees and the Gram kernel
   * exploits). blockOffsets holds numBlocks+1 ascending scalar-column
   * offsets with blockOffsets.back() == columns. Synchronizes the stream
   * once for the validation result.
   */
  void initialize(cusparseHandle_t handle, int rows, int columns,
                  cusparseSpMatDescr_t j, cusparseSpMatDescr_t jt,
                  const DeviceArray<int>& jtRowPointers,
                  const std::vector<int>& blockOffsets,
                  const DevicePcgOptions& options, cudaStream_t stream,
                  bool collectProfile);

  /**
   * Per outer linearization: rebuild the undamped Gram blocks from the
   * numerically refreshed Jᵀ values and invalidate the warm start.
   * Asynchronous except when profiling is enabled.
   */
  void buildPreconditioner(const DeviceArray<double>& jtValues,
                           cudaStream_t stream);

  /// Applies attempt-specific damping to the borrowed operator/preconditioner.
  void prepare(double lambda,
               const DeviceArray<double>& dampingDiagonal,
               cudaStream_t stream);
  /// Returns the prepared matrix-free operator.
  const LinearOperator& linearOperator() const;
  /// Returns the prepared preconditioner.
  const Preconditioner& preconditioner() const;
  /// Returns cumulative device time spent rebuilding the preconditioner.
  double preconditionerBuildSeconds() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
