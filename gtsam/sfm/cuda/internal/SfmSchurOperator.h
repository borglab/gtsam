/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmSchurOperator.h
 * @brief   Matrix-free reduced-camera Schur operator and block preconditioner
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/LinearSystem.h>
#include <gtsam/sfm/cuda/internal/SfmProjectionBatch.h>
#include <gtsam/sfm/cuda/internal/SfmProjectionLinearization.h>

namespace gtsam::cuda {

struct SfmSchurBlocks;

class GTSAM_EXPORT SfmSchurOperator final : public LinearOperator {
 public:
  SfmSchurOperator(const SfmProjectionBatch& batch,
                   const SfmSchurBlocks& blocks, int numCameras);

  void configure(double lambda,
                 const DeviceArray<double>* dampingDiagonal);
  int dimension() const override { return 9 * numCameras_; }
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;

 private:
  const SfmProjectionBatch* batch_;
  const SfmSchurBlocks* blocks_;
  int numCameras_;
  double lambda_ = 0.0;
  const DeviceArray<double>* dampingDiagonal_ = nullptr;
};

class GTSAM_EXPORT SfmCameraBlockPreconditioner final
    : public Preconditioner {
 public:
  SfmCameraBlockPreconditioner(
      const SfmProjectionBatch& batch,
      const SfmSchurBlocks& blocks, int numCameras);

  void build(double lambda, const DeviceArray<double>* dampingDiagonal,
             DeviceArray<double>* condensedRhs,
             cudaStream_t stream = nullptr);
  int dimension() const override { return 9 * numCameras_; }
  void apply(const double* input, double* output,
             cudaStream_t stream) const override;
  const DeviceArray<double>& inverseBlocks() const {
    return inverseBlocks_;
  }

 private:
  const SfmProjectionBatch* batch_;
  const SfmSchurBlocks* blocks_;
  int numCameras_;
  DeviceArray<double> cameraBlocks_;
  DeviceArray<double> inverseBlocks_;
  DeviceArray<int> singularBlocks_;
};

}  // namespace gtsam::cuda
