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

enum class DevicePcgPreconditioner {
  BlockJacobi,
  Jacobi,
  None,
};

/// Non-owning general-Jacobian operator J'J + lambda D.
class GTSAM_EXPORT JacobianNormalOperator final
    : public LinearOperator {
 public:
  JacobianNormalOperator();
  ~JacobianNormalOperator() override;
  JacobianNormalOperator(JacobianNormalOperator&&) noexcept;
  JacobianNormalOperator& operator=(
      JacobianNormalOperator&&) noexcept;

  JacobianNormalOperator(const JacobianNormalOperator&) = delete;
  JacobianNormalOperator& operator=(
      const JacobianNormalOperator&) = delete;

  void initialize(cusparseHandle_t handle, int rows, int columns,
                  cusparseSpMatDescr_t jacobian,
                  cusparseSpMatDescr_t jacobianTranspose,
                  cudaStream_t stream = nullptr);
  void setDamping(double lambda,
                  const DeviceArray<double>& dampingDiagonal,
                  cudaStream_t stream = nullptr);

  int dimension() const override;
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/** General-Jacobian block/scalar Jacobi producer exposed as a common
 * Preconditioner. Numerical Gram blocks are rebuilt per linearization;
 * lambda-dependent inverses are prepared per LM attempt. */
class GTSAM_EXPORT JacobianNormalPreconditioner final
    : public Preconditioner {
 public:
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

  void initialize(int columns,
                  const DeviceArray<int>& transposeRowPointers,
                  const std::vector<int>& blockOffsets,
                  DevicePcgPreconditioner type,
                  cudaStream_t stream = nullptr,
                  bool collectProfile = false);
  void build(const DeviceArray<double>& transposeValues,
             cudaStream_t stream = nullptr);
  void prepare(double lambda,
               const DeviceArray<double>& dampingDiagonal,
               cudaStream_t stream = nullptr);

  int dimension() const override;
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;

  double buildSeconds() const;
  double prepareSeconds() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
