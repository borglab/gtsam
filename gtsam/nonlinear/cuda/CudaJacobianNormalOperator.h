#pragma once

#include <cuda_runtime_api.h>
#include <cusparse.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/CudaLinearSystem.h>

#include <memory>
#include <vector>

namespace gtsam::cuda {

enum class DevicePcgPreconditioner {
  BlockJacobi,
  Jacobi,
  None,
};

/** Borrowed general-Jacobian operator J'J + lambda D. */
class GTSAM_EXPORT CudaJacobianNormalOperator final
    : public CudaLinearOperator {
 public:
  CudaJacobianNormalOperator();
  ~CudaJacobianNormalOperator() override;
  CudaJacobianNormalOperator(CudaJacobianNormalOperator&&) noexcept;
  CudaJacobianNormalOperator& operator=(
      CudaJacobianNormalOperator&&) noexcept;

  CudaJacobianNormalOperator(const CudaJacobianNormalOperator&) = delete;
  CudaJacobianNormalOperator& operator=(
      const CudaJacobianNormalOperator&) = delete;

  void initialize(cusparseHandle_t handle, int rows, int columns,
                  cusparseSpMatDescr_t jacobian,
                  cusparseSpMatDescr_t jacobianTranspose,
                  cudaStream_t stream = nullptr);
  void setDamping(double lambda,
                  const CudaDeviceArray<double>& dampingDiagonal,
                  cudaStream_t stream = nullptr);

  int dimension() const override;
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/** General-Jacobian block/scalar Jacobi producer exposed as a common
 * CudaPreconditioner. Numerical Gram blocks are rebuilt per linearization;
 * lambda-dependent inverses are prepared per LM attempt. */
class GTSAM_EXPORT CudaJacobianNormalPreconditioner final
    : public CudaPreconditioner {
 public:
  CudaJacobianNormalPreconditioner();
  ~CudaJacobianNormalPreconditioner() override;
  CudaJacobianNormalPreconditioner(
      CudaJacobianNormalPreconditioner&&) noexcept;
  CudaJacobianNormalPreconditioner& operator=(
      CudaJacobianNormalPreconditioner&&) noexcept;

  CudaJacobianNormalPreconditioner(
      const CudaJacobianNormalPreconditioner&) = delete;
  CudaJacobianNormalPreconditioner& operator=(
      const CudaJacobianNormalPreconditioner&) = delete;

  void initialize(int columns,
                  const CudaDeviceArray<int>& transposeRowPointers,
                  const std::vector<int>& blockOffsets,
                  DevicePcgPreconditioner type,
                  cudaStream_t stream = nullptr,
                  bool collectProfile = false);
  void build(const CudaDeviceArray<double>& transposeValues,
             cudaStream_t stream = nullptr);
  void prepare(double lambda,
               const CudaDeviceArray<double>& dampingDiagonal,
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
