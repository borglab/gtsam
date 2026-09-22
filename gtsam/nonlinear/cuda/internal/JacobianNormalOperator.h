/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    JacobianNormalOperator.h
 * @brief   Matrix-free J'J + lambda D operator and its Jacobi preconditioner
 * @author  Ruogu Li
 * @date    Aug 16, 2026
 */

#pragma once

#include <cuda_runtime_api.h>
#include <cusparse.h>
#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/LinearSystem.h>

#include <memory>
#include <vector>

namespace gtsam::cuda {

/// Which approximate inverse of J'J + lambda D the PCG solver applies.
enum class DevicePcgPreconditioner {
  /// Invert each variable's diagonal block of the Gram matrix exactly. More
  /// setup per linearization, and usually the fewest iterations.
  BlockJacobi,
  /// Invert the scalar diagonal only, which is nearly free to build.
  Jacobi,
  /// No preconditioning; PCG reduces to plain CG.
  None,
};

/**
 * The normal-equations operator J'J + lambda D of a general sparse Jacobian,
 * applied without ever forming it.
 *
 * J'J is far denser than J, so building it costs more memory and time than the
 * whole PCG solve; a matrix-vector product with it is instead two cuSPARSE
 * products, by J and then by J'. That is all a Krylov method needs, so this
 * satisfies LinearOperator on top of descriptors it does not own: the caller
 * keeps the Jacobian and its transpose alive, and only the damping is state here.
 *
 * The damping is separate from initialization so an LM attempt loop can change
 * lambda without touching the Jacobian.
 */
class GTSAM_EXPORT JacobianNormalOperator final
    : public LinearOperator {
 public:
  /// Creates an operator with no Jacobian bound; call initialize() first.
  JacobianNormalOperator();
  ~JacobianNormalOperator() override;
  JacobianNormalOperator(JacobianNormalOperator&&) noexcept;
  JacobianNormalOperator& operator=(
      JacobianNormalOperator&&) noexcept;

  JacobianNormalOperator(const JacobianNormalOperator&) = delete;
  JacobianNormalOperator& operator=(
      const JacobianNormalOperator&) = delete;

  /// Binds the Jacobian and its explicitly stored transpose, sizes the
  /// intermediate vector, and reserves the cuSPARSE workspaces. The transpose is
  /// passed in as its own matrix so both products can run untransposed; the
  /// caller builds it once per structure. Both descriptors and the handle must
  /// outlive this operator, and every later call must use the same stream, since
  /// the workspaces are bound to it.
  void initialize(cusparseHandle_t handle, int rows, int columns,
                  cusparseSpMatDescr_t jacobian,
                  cusparseSpMatDescr_t jacobianTranspose,
                  cudaStream_t stream = nullptr);
  /// Sets the damping term, so lambda can change between LM attempts without
  /// rebinding the Jacobian. The diagonal is borrowed, not copied.
  void setDamping(double lambda,
                  const DeviceArray<double>& dampingDiagonal,
                  cudaStream_t stream = nullptr);

  /// Side length of the operator, which is the Jacobian's column count.
  int dimension() const override;
  /// Computes output = (J'J + lambda D) input as two sparse products and a
  /// damping kernel, all queued on the stream given to initialize().
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;

 private:
  struct Impl;
  /// Holds the borrowed descriptors, the intermediate vector, and the
  /// workspaces, so this header needs no nvcc.
  std::unique_ptr<Impl> impl_;
};

/** General-Jacobian block/scalar Jacobi producer exposed as a common
 * Preconditioner. Numerical Gram blocks are rebuilt per linearization;
 * lambda-dependent inverses are prepared per LM attempt. */
class GTSAM_EXPORT JacobianNormalPreconditioner final
    : public Preconditioner {
 public:
  /// Creates a preconditioner with no structure bound; call initialize() first.
  JacobianNormalPreconditioner();
  ~JacobianNormalPreconditioner() override;
  JacobianNormalPreconditioner(
      JacobianNormalPreconditioner&&) noexcept;
  JacobianNormalPreconditioner& operator=(
      JacobianNormalPreconditioner&&) noexcept;

  JacobianNormalPreconditioner(
      const JacobianNormalPreconditioner&) = delete;
  JacobianNormalPreconditioner& operator=(
      const JacobianNormalPreconditioner&) = delete;

  /// Records the structure: the transposed Jacobian's row pointers, the variable
  /// block boundaries, and which preconditioner to build. Called once per
  /// symbolic structure. Set collectProfile to accumulate the timings below,
  /// which costs a synchronization per call.
  void initialize(int columns,
                  const DeviceArray<int>& transposeRowPointers,
                  const std::vector<int>& blockOffsets,
                  DevicePcgPreconditioner type,
                  cudaStream_t stream = nullptr,
                  bool collectProfile = false);
  /// Recomputes the undamped Gram diagonal or blocks from fresh numbers. Once
  /// per linearization, since these depend on the Jacobian but not on lambda.
  void build(const DeviceArray<double>& transposeValues,
             cudaStream_t stream = nullptr);
  /// Applies the damping to what build() produced: BlockJacobi inverts its
  /// damped blocks here, while Jacobi only records lambda and divides during
  /// apply(). Once per LM attempt, since this is the lambda-dependent part.
  void prepare(double lambda,
               const DeviceArray<double>& dampingDiagonal,
               cudaStream_t stream = nullptr);

  /// Side length of the preconditioner, which is the Jacobian's column count.
  int dimension() const override;
  /// Applies the stored approximate inverse, which for None is a copy.
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;

  /// Total time in build(), zero unless profiling was requested.
  double buildSeconds() const;
  /// Total time in prepare(), zero unless profiling was requested.
  double prepareSeconds() const;

 private:
  struct Impl;
  /// Holds the block structure, the Gram blocks, and their inverses, so this
  /// header needs no nvcc.
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
