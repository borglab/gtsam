/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmSchurOperator.cu
 * @brief   Matrix-free reduced-camera Schur operator and block preconditioner
 * @author  Ruogu Li
 * @date    Aug 16, 2026
 */

#include <gtsam/base/cuda/Errors.h>
#include <gtsam/sfm/cuda/internal/SfmSchurOperator.h>
#include <gtsam/sfm/cuda/internal/SfmSchurProblem.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace gtsam::cuda {
namespace {
constexpr int kBlockSize = 128;

__device__ bool invertPointBlock(const double* A, double* inverse) {
  const double c00 = A[4] * A[8] - A[5] * A[7];
  const double c01 = A[2] * A[7] - A[1] * A[8];
  const double c02 = A[1] * A[5] - A[2] * A[4];
  const double c10 = A[5] * A[6] - A[3] * A[8];
  const double c11 = A[0] * A[8] - A[2] * A[6];
  const double c12 = A[2] * A[3] - A[0] * A[5];
  const double c20 = A[3] * A[7] - A[4] * A[6];
  const double c21 = A[1] * A[6] - A[0] * A[7];
  const double c22 = A[0] * A[4] - A[1] * A[3];
  const double determinant = A[0] * c00 + A[1] * c10 + A[2] * c20;
  if (fabs(determinant) < 1e-30) return false;
  const double scale = 1.0 / determinant;
  inverse[0] = c00 * scale;
  inverse[1] = c01 * scale;
  inverse[2] = c02 * scale;
  inverse[3] = c10 * scale;
  inverse[4] = c11 * scale;
  inverse[5] = c12 * scale;
  inverse[6] = c20 * scale;
  inverse[7] = c21 * scale;
  inverse[8] = c22 * scale;
  return true;
}

__device__ void formPointBlock(const double* pointNormalBlocks,
                               const double* pointGradient, int point,
                               int pointBase, double lambda,
                               const double* damping, double* block,
                               double* rhs) {
  const double* undamped = pointNormalBlocks + 9 * point;
  const double* gradient = pointGradient ? pointGradient + 3 * point : nullptr;
  for (int i = 0; i < 9; ++i) block[i] = undamped[i];
  block[0] += lambda * (damping ? damping[pointBase] : 1.0);
  block[4] += lambda * (damping ? damping[pointBase + 1] : 1.0);
  block[8] += lambda * (damping ? damping[pointBase + 2] : 1.0);
  rhs[0] = gradient ? gradient[0] : 0.0;
  rhs[1] = gradient ? gradient[1] : 0.0;
  rhs[2] = gradient ? gradient[2] : 0.0;
}

__device__ void rowTimesInverse(const double* row, const double* inverse,
                                double* result) {
  for (int c = 0; c < 3; ++c) {
    result[c] = row[0] * inverse[c] + row[1] * inverse[3 + c] +
                row[2] * inverse[6 + c];
  }
}

__global__ void applyImplicitSchurKernel(
    const SfmObservation* observations, const int* pointOffsets,
    const double* pointNormalBlocks, const double* cameraPointBlocks,
    int numPoints, int cameraDimension, double lambda, const double* damping,
    const double* input, double* output) {
  const int point = blockIdx.x * blockDim.x + threadIdx.x;
  if (point >= numPoints) return;
  const int begin = pointOffsets[point];
  const int end = pointOffsets[point + 1];
  double pointBlock[9], unusedRhs[3], inverse[9];
  formPointBlock(pointNormalBlocks, nullptr, point,
                 cameraDimension + 3 * point, lambda, damping, pointBlock,
                 unusedRhs);
  if (!invertPointBlock(pointBlock, inverse)) return;

  double pointProduct[3] = {0.0, 0.0, 0.0};
  for (int observation = begin; observation < end; ++observation) {
    const int cameraBase = 9 * observations[observation].cameraSlot;
    const double* W = cameraPointBlocks + 27 * observation;
    for (int c = 0; c < 9; ++c) {
      for (int p = 0; p < 3; ++p) {
        pointProduct[p] += W[3 * c + p] * input[cameraBase + c];
      }
    }
  }
  double solvedPoint[3];
  for (int r = 0; r < 3; ++r) {
    solvedPoint[r] = inverse[3 * r] * pointProduct[0] +
                     inverse[3 * r + 1] * pointProduct[1] +
                     inverse[3 * r + 2] * pointProduct[2];
  }
  for (int observation = begin; observation < end; ++observation) {
    const int cameraBase = 9 * observations[observation].cameraSlot;
    const double* W = cameraPointBlocks + 27 * observation;
    for (int c = 0; c < 9; ++c) {
      atomicAdd(&output[cameraBase + c],
                -(W[3 * c] * solvedPoint[0] +
                  W[3 * c + 1] * solvedPoint[1] +
                  W[3 * c + 2] * solvedPoint[2]));
    }
  }
}

__global__ void initializeImplicitCameraProductKernel(
    int numCameras, const double* cameraNormalBlocks, double lambda,
    const double* damping, const double* input, double* output) {
  const int camera = blockIdx.x * blockDim.x + threadIdx.x;
  if (camera >= numCameras) return;
  const int base = 9 * camera;
  const double* U = cameraNormalBlocks + 81 * camera;
  for (int row = 0; row < 9; ++row) {
    double value = 0.0;
    for (int column = 0; column < 9; ++column) {
      value += U[9 * row + column] * input[base + column];
    }
    value += lambda * (damping ? damping[base + row] : 1.0) *
             input[base + row];
    output[base + row] = value;
  }
}

__global__ void buildRhsAndCameraBlocksKernel(
    const SfmObservation* observations, const int* pointOffsets,
    const double* pointNormalBlocks, const double* pointGradient,
    const double* cameraPointBlocks, int numPoints, int cameraDimension,
    double lambda, const double* damping, double* rhs, double* blocks) {
  const int point = blockIdx.x * blockDim.x + threadIdx.x;
  if (point >= numPoints) return;
  const int begin = pointOffsets[point];
  const int end = pointOffsets[point + 1];
  double pointBlock[9], pointRhs[3], inverse[9];
  formPointBlock(pointNormalBlocks, pointGradient, point,
                 cameraDimension + 3 * point, lambda, damping, pointBlock,
                 pointRhs);
  if (!invertPointBlock(pointBlock, inverse)) return;

  for (int obsI = begin; obsI < end; ++obsI) {
    const int cameraI = observations[obsI].cameraSlot;
    const int cameraBase = 9 * cameraI;
    const double* W_i = cameraPointBlocks + 27 * obsI;
    for (int a = 0; a < 9; ++a) {
      const double* Ei = W_i + 3 * a;
      double EiInv[3];
      rowTimesInverse(Ei, inverse, EiInv);
      atomicAdd(&rhs[cameraBase + a],
                -(EiInv[0] * pointRhs[0] + EiInv[1] * pointRhs[1] +
                  EiInv[2] * pointRhs[2]));
      for (int obsJ = begin; obsJ < end; ++obsJ) {
        if (observations[obsJ].cameraSlot != cameraI) continue;
        const double* W_j = cameraPointBlocks + 27 * obsJ;
        for (int b = 0; b < 9; ++b) {
          const double* Ej = W_j + 3 * b;
          atomicAdd(&blocks[81 * cameraI + 9 * a + b],
                    -(EiInv[0] * Ej[0] + EiInv[1] * Ej[1] +
                      EiInv[2] * Ej[2]));
        }
      }
    }
  }
}

__global__ void initializeCameraPreconditionerKernel(
    int numCameras, const double* cameraNormalBlocks,
    const double* cameraGradient, double* rhs, double* blocks) {
  const int camera = blockIdx.x * blockDim.x + threadIdx.x;
  if (camera >= numCameras) return;
  const int base = 9 * camera;
  for (int i = 0; i < 9; ++i) rhs[base + i] = cameraGradient[base + i];
  for (int i = 0; i < 81; ++i) {
    blocks[81 * camera + i] = cameraNormalBlocks[81 * camera + i];
  }
}

__global__ void addCameraBlockDampingKernel(int numCameras, double lambda,
                                            const double* damping,
                                            double* blocks) {
  const int camera = blockIdx.x * blockDim.x + threadIdx.x;
  if (camera >= numCameras) return;
  for (int scalar = 0; scalar < 9; ++scalar) {
    blocks[81 * camera + 9 * scalar + scalar] +=
        lambda * (damping ? damping[9 * camera + scalar] : 1.0);
  }
}

__global__ void invertCameraBlocksKernel(int numCameras, const double* blocks,
                                         double* inverseBlocks,
                                         int* singularBlocks) {
  const int camera = blockIdx.x * blockDim.x + threadIdx.x;
  if (camera >= numCameras) return;
  double augmented[9][18];
  for (int row = 0; row < 9; ++row) {
    for (int column = 0; column < 9; ++column) {
      augmented[row][column] = blocks[81 * camera + 9 * row + column];
      augmented[row][9 + column] = row == column ? 1.0 : 0.0;
    }
  }
  for (int pivot = 0; pivot < 9; ++pivot) {
    int best = pivot;
    for (int row = pivot + 1; row < 9; ++row) {
      if (fabs(augmented[row][pivot]) > fabs(augmented[best][pivot])) best = row;
    }
    if (fabs(augmented[best][pivot]) < 1e-30) {
      atomicAdd(singularBlocks, 1);
      for (int row = 0; row < 9; ++row)
        for (int column = 0; column < 9; ++column)
          inverseBlocks[81 * camera + 9 * row + column] =
              row == column ? 1.0 : 0.0;
      return;
    }
    if (best != pivot) {
      for (int column = 0; column < 18; ++column) {
        const double temporary = augmented[pivot][column];
        augmented[pivot][column] = augmented[best][column];
        augmented[best][column] = temporary;
      }
    }
    const double scale = 1.0 / augmented[pivot][pivot];
    for (int column = 0; column < 18; ++column)
      augmented[pivot][column] *= scale;
    for (int row = 0; row < 9; ++row) {
      if (row == pivot) continue;
      const double factor = augmented[row][pivot];
      for (int column = 0; column < 18; ++column)
        augmented[row][column] -= factor * augmented[pivot][column];
    }
  }
  for (int row = 0; row < 9; ++row)
    for (int column = 0; column < 9; ++column)
      inverseBlocks[81 * camera + 9 * row + column] = augmented[row][9 + column];
}

__global__ void applyCameraBlocksKernel(int numCameras,
                                        const double* inverseBlocks,
                                        const double* input, double* output) {
  const int camera = blockIdx.x * blockDim.x + threadIdx.x;
  if (camera >= numCameras) return;
  for (int row = 0; row < 9; ++row) {
    double value = 0.0;
    for (int column = 0; column < 9; ++column) {
      value += inverseBlocks[81 * camera + 9 * row + column] *
               input[9 * camera + column];
    }
    output[9 * camera + row] = value;
  }
}

int grid(int count) {
  const size_t blocks =
      (static_cast<size_t>(count) + kBlockSize - 1) / kBlockSize;
  if (blocks > static_cast<size_t>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("implicit Schur launch grid exceeds int range");
  return static_cast<int>(blocks);
}

void validateDamping(const SfmProjectionBatch& batch, int numCameras,
                     double lambda,
                     const DeviceArray<double>* dampingDiagonal) {
  if (!(lambda > 0.0) || !std::isfinite(lambda))
    throw std::invalid_argument("implicit Schur requires finite lambda > 0");
  if (dampingDiagonal &&
      dampingDiagonal->size() != 9 * static_cast<size_t>(numCameras) +
                                     3 * batch.numPoints())
    throw std::invalid_argument("implicit Schur damping size mismatch");
}
}  // namespace

SfmSchurOperator::SfmSchurOperator(
    const SfmProjectionBatch& batch,
    const SfmSchurBlocks& blocks, int numCameras)
    : batch_(&batch), blocks_(&blocks), numCameras_(numCameras) {}

void SfmSchurOperator::configure(
    double lambda, const DeviceArray<double>* dampingDiagonal) {
  validateDamping(*batch_, numCameras_, lambda, dampingDiagonal);
  lambda_ = lambda;
  dampingDiagonal_ = dampingDiagonal;
}

void SfmSchurOperator::apply(const double* input, double* output,
                                 cudaStream_t stream) const {
  if (!(lambda_ > 0.0))
    throw std::logic_error("implicit Schur operator is not configured");
  const int cameraDimension = dimension();
  if (numCameras_ > 0) {
    initializeImplicitCameraProductKernel<<<grid(numCameras_), kBlockSize, 0,
                                            stream>>>(
        numCameras_, blocks_->cameraNormalBlocks.data(), lambda_,
        dampingDiagonal_ ? dampingDiagonal_->data() : nullptr, input, output);
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
  const int numPoints = static_cast<int>(batch_->numPoints());
  if (numPoints > 0) {
    applyImplicitSchurKernel<<<grid(numPoints), kBlockSize, 0, stream>>>(
        batch_->observations().data(),
        batch_->pointObservationOffsets().data(),
        blocks_->pointNormalBlocks.data(),
        blocks_->cameraPointBlocks.data(), numPoints, cameraDimension, lambda_,
        dampingDiagonal_ ? dampingDiagonal_->data() : nullptr, input, output);
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

SfmCameraBlockPreconditioner::SfmCameraBlockPreconditioner(
    const SfmProjectionBatch& batch,
    const SfmSchurBlocks& blocks, int numCameras)
    : batch_(&batch), blocks_(&blocks), numCameras_(numCameras) {}

void SfmCameraBlockPreconditioner::build(
    double lambda, const DeviceArray<double>* dampingDiagonal,
    DeviceArray<double>* condensedRhs, cudaStream_t stream) {
  if (!condensedRhs)
    throw std::invalid_argument("implicit Schur RHS output is null");
  validateDamping(*batch_, numCameras_, lambda, dampingDiagonal);
  const int cameraDimension = dimension();
  condensedRhs->resize(cameraDimension);
  condensedRhs->zero(stream);
  cameraBlocks_.resize(81 * static_cast<size_t>(numCameras_));
  cameraBlocks_.zero(stream);
  inverseBlocks_.resize(81 * static_cast<size_t>(numCameras_));
  singularBlocks_.resize(1);
  singularBlocks_.zero(stream);
  if (numCameras_ > 0) {
    initializeCameraPreconditionerKernel<<<grid(numCameras_), kBlockSize, 0,
                                           stream>>>(
        numCameras_, blocks_->cameraNormalBlocks.data(),
        blocks_->cameraGradient.data(), condensedRhs->data(),
        cameraBlocks_.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
  const int numPoints = static_cast<int>(batch_->numPoints());
  if (numPoints > 0) {
    buildRhsAndCameraBlocksKernel<<<grid(numPoints), kBlockSize, 0, stream>>>(
        batch_->observations().data(),
        batch_->pointObservationOffsets().data(),
        blocks_->pointNormalBlocks.data(), blocks_->pointGradient.data(),
        blocks_->cameraPointBlocks.data(), numPoints, cameraDimension,
        lambda, dampingDiagonal ? dampingDiagonal->data() : nullptr,
        condensedRhs->data(), cameraBlocks_.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
  if (numCameras_ > 0) {
    addCameraBlockDampingKernel<<<grid(numCameras_), kBlockSize, 0, stream>>>(
        numCameras_, lambda,
        dampingDiagonal ? dampingDiagonal->data() : nullptr,
        cameraBlocks_.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
    invertCameraBlocksKernel<<<grid(numCameras_), kBlockSize, 0, stream>>>(
        numCameras_, cameraBlocks_.data(), inverseBlocks_.data(),
        singularBlocks_.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

void SfmCameraBlockPreconditioner::apply(const double* input,
                                             double* output,
                                             cudaStream_t stream) const {
  if (inverseBlocks_.size() != 81 * static_cast<size_t>(numCameras_))
    throw std::logic_error("camera block preconditioner is not built");
  if (numCameras_ > 0) {
    applyCameraBlocksKernel<<<grid(numCameras_), kBlockSize, 0, stream>>>(
        numCameras_, inverseBlocks_.data(), input, output);
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

}  // namespace gtsam::cuda
