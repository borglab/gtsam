#pragma once

#include <cuda_runtime_api.h>
#include <cusparse.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/dllexport.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace gtsam::cuda {

enum class DevicePcgPreconditioner {
  BlockJacobi,  // per-variable Gram-block inverses (default)
  Jacobi,       // scalar 1/diag(JtJ + lambda*D), PyPose/BAE-style
  None,         // unpreconditioned CG (ablation baseline)
};

struct DevicePcgOptions {
  int maxIterations = 250;
  double relativeTolerance = 1e-6;
  bool warmStart = true;
  int convergenceCheckInterval = 10;
  DevicePcgPreconditioner preconditioner = DevicePcgPreconditioner::BlockJacobi;
};

struct DevicePcgSolveStats {
  int iterations = 0;
  double residualNormSquared = 0.0;
  double gradientNormSquared = 0.0;
  bool converged = false;
  bool breakdown = false;
};

struct DevicePcgProfile {
  double preconditionerBuild = 0.0;
  double solve = 0.0;
  size_t iterationsTotal = 0;
  size_t solveCount = 0;
  size_t maxIterationHits = 0;
};

/**
 * Matrix-free preconditioned conjugate gradient for the damped normal
 * equations (JᵀJ + lambda·D) x = g on a fixed borrowed stream.
 *
 * The operator is applied as two cuSPARSE SpMVs (J·p, then Jᵀ·(J·p)) plus an
 * elementwise damping term; the normal matrix H is never formed. The
 * preconditioner is block-Jacobi over the variable blocks of the column
 * layout: the undamped Gram blocks diag_k(JᵀJ) are rebuilt once per
 * linearization from the Jᵀ CSR arrays, and each solve() damps, factors, and
 * explicitly inverts them per lambda.
 *
 * solve() drives CG from the host with device-resident scalar recurrences and
 * synchronizes the stream on the periodic convergence checks and once on
 * exit, so lastSolveStats() is valid immediately after solve() returns.
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
                  const CudaDeviceArray<int>& jtRowPointers,
                  const std::vector<int>& blockOffsets,
                  const DevicePcgOptions& options, cudaStream_t stream,
                  bool collectProfile);

  /**
   * Per outer linearization: rebuild the undamped Gram blocks from the
   * numerically refreshed Jᵀ values and invalidate the warm start.
   * Asynchronous except when profiling is enabled.
   */
  void buildPreconditioner(const CudaDeviceArray<double>& jtValues,
                           cudaStream_t stream);

  /**
   * Per lambda attempt: factor the damped preconditioner and run PCG,
   * writing the solution into *delta (warm-started from the previous
   * attempt's delta when enabled). gradient is g = Jᵀb and dampingDiagonal
   * is the prepared damping vector D. The returned delta is always finite:
   * numerical breakdown freezes the last finite iterate instead of
   * propagating non-finite values. Synchronizes the stream before returning.
   */
  void solve(double lambda, const CudaDeviceArray<double>& gradient,
             const CudaDeviceArray<double>& dampingDiagonal,
             CudaDeviceArray<double>* delta, cudaStream_t stream);

  const DevicePcgSolveStats& lastSolveStats() const;
  const DevicePcgProfile& profile() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
