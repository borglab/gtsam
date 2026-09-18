/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    PcgOperatorBuilder.h
 * @brief   Builds the matrix-free normal operator and preconditioner for PCG
 * @author  Ruogu Li
 * @date    Jul 25, 2026
 */

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

/// Settings for the PCG run that consumes these operators.
struct DevicePcgOptions {
  /// Maximum conjugate gradient iterations per linear solve.
  int maxIterations = 250;
  /// Convergence threshold on the residual, relative to the initial residual.
  double relativeTolerance = 1e-6;
  /// Whether to start from the previous attempt's solution.
  bool warmStart = true;
  /// Iterations between residual norm reductions, which each cost a sync.
  int convergenceCheckInterval = 10;
  /// Which preconditioner buildPreconditioner() should construct.
  DevicePcgPreconditioner preconditioner = DevicePcgPreconditioner::BlockJacobi;
};

/**
 * Produces the matrix-free normal operator and block preconditioner that
 * LinearSolverSession hands to PcgSolver for general factor graphs.
 *
 * This class runs no conjugate gradient iterations and has no solve(): the
 * recurrence, convergence tests, and warm-start state all live in
 * gtsam::cuda::PcgSolver, which owns the CG kernels. What this builds is
 * PcgSolver's two inputs, from a Jacobian that is already resident on the
 * device:
 *
 *  - the operator applying JᵀJ + λD as two cuSPARSE SpMVs (J·p, then
 *    Jᵀ·(J·p)) plus an elementwise damping term, so the normal matrix H is
 *    never formed; and
 *  - a block-Jacobi preconditioner over the variable blocks of the column
 *    layout, whose undamped Gram blocks diag_k(JᵀJ) are rebuilt once per
 *    linearization from the Jᵀ CSR arrays.
 *
 * The SFM optimizer does not use this: it reaches PcgSolver through its own
 * Schur-complement operator instead. The call order is initialize() once, then
 * buildPreconditioner() per linearization, then prepare() per damping attempt.
 */
class GTSAM_EXPORT PcgOperatorBuilder {
 public:
  PcgOperatorBuilder();
  ~PcgOperatorBuilder();

  PcgOperatorBuilder(const PcgOperatorBuilder&) = delete;
  PcgOperatorBuilder& operator=(const PcgOperatorBuilder&) = delete;
  PcgOperatorBuilder(PcgOperatorBuilder&&) noexcept;
  PcgOperatorBuilder& operator=(PcgOperatorBuilder&&) noexcept;

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
