#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/linear/cuda/CudaLinearSystem.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>

namespace gtsam::cuda {

class CudaSfmSchurOperator final : public CudaLinearOperator {
 public:
  CudaSfmSchurOperator(const CudaSfmProjectionBatch& batch,
                       const CudaSfmProjectionLinearization& linearization,
                       int numCameras);

  void configure(double lambda,
                 const CudaDeviceArray<double>* dampingDiagonal);
  int dimension() const override { return 9 * numCameras_; }
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;

 private:
  const CudaSfmProjectionBatch* batch_;
  const CudaSfmProjectionLinearization* linearization_;
  int numCameras_;
  double lambda_ = 0.0;
  const CudaDeviceArray<double>* dampingDiagonal_ = nullptr;
};

class CudaSfmCameraBlockPreconditioner final : public CudaPreconditioner {
 public:
  CudaSfmCameraBlockPreconditioner(
      const CudaSfmProjectionBatch& batch,
      const CudaSfmProjectionLinearization& linearization, int numCameras);

  void build(double lambda, const CudaDeviceArray<double>* dampingDiagonal,
             CudaDeviceArray<double>* condensedRhs,
             cudaStream_t stream = nullptr);
  int dimension() const override { return 9 * numCameras_; }
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;
  const CudaDeviceArray<double>& inverseBlocks() const {
    return inverseBlocks_;
  }

 private:
  const CudaSfmProjectionBatch* batch_;
  const CudaSfmProjectionLinearization* linearization_;
  int numCameras_;
  CudaDeviceArray<double> cameraBlocks_;
  CudaDeviceArray<double> inverseBlocks_;
  CudaDeviceArray<int> singularBlocks_;
};

}  // namespace gtsam::cuda
