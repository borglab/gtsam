#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/slam/cuda/CudaSfmSchurOperator.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace gtsam::cuda {
namespace {
constexpr int kBlockSize = 128;

__device__ bool InvertPointBlock(const double* A, double* inverse) {
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

__device__ void FormPointBlock(const double* residuals,
                               const double* pointJacobians, int begin,
                               int end, int pointBase, double lambda,
                               const double* damping, double* block,
                               double* rhs) {
  for (int i = 0; i < 9; ++i) block[i] = 0.0;
  block[0] = lambda * (damping ? damping[pointBase] : 1.0);
  block[4] = lambda * (damping ? damping[pointBase + 1] : 1.0);
  block[8] = lambda * (damping ? damping[pointBase + 2] : 1.0);
  rhs[0] = rhs[1] = rhs[2] = 0.0;
  for (int observation = begin; observation < end; ++observation) {
    const double* J = pointJacobians + 6 * observation;
    const double r0 = residuals ? residuals[2 * observation] : 0.0;
    const double r1 = residuals ? residuals[2 * observation + 1] : 0.0;
    for (int a = 0; a < 3; ++a) {
      rhs[a] += -J[a] * r0 - J[3 + a] * r1;
      for (int b = 0; b < 3; ++b) {
        block[3 * a + b] += J[a] * J[b] + J[3 + a] * J[3 + b];
      }
    }
  }
}

__device__ void CameraPointRow(const double* cameraJacobians,
                               const double* pointJacobians, int observation,
                               int cameraRow, double* row) {
  const double* Jc = cameraJacobians + 18 * observation;
  const double* Jp = pointJacobians + 6 * observation;
  for (int p = 0; p < 3; ++p) {
    row[p] = Jc[cameraRow] * Jp[p] +
             Jc[9 + cameraRow] * Jp[3 + p];
  }
}

__device__ void RowTimesInverse(const double* row, const double* inverse,
                                double* result) {
  for (int c = 0; c < 3; ++c) {
    result[c] = row[0] * inverse[c] + row[1] * inverse[3 + c] +
                row[2] * inverse[6 + c];
  }
}

__global__ void ApplyImplicitSchurKernel(
    const CudaSfmObservation* observations, const int* pointOffsets,
    const double* cameraJacobians, const double* pointJacobians, int numPoints,
    int cameraDimension, double lambda, const double* damping,
    const double* input, double* output) {
  const int point = blockIdx.x * blockDim.x + threadIdx.x;
  if (point >= numPoints) return;
  const int begin = pointOffsets[point];
  const int end = pointOffsets[point + 1];
  double pointBlock[9], unusedRhs[3], inverse[9];
  FormPointBlock(nullptr, pointJacobians, begin, end,
                 cameraDimension + 3 * point, lambda, damping, pointBlock,
                 unusedRhs);
  if (!InvertPointBlock(pointBlock, inverse)) return;

  double pointProduct[3] = {0.0, 0.0, 0.0};
  for (int observation = begin; observation < end; ++observation) {
    const int cameraBase = 9 * observations[observation].cameraSlot;
    const double* Jc = cameraJacobians + 18 * observation;
    double projection0 = 0.0, projection1 = 0.0;
    for (int c = 0; c < 9; ++c) {
      projection0 += Jc[c] * input[cameraBase + c];
      projection1 += Jc[9 + c] * input[cameraBase + c];
    }
    for (int c = 0; c < 9; ++c) {
      atomicAdd(&output[cameraBase + c],
                Jc[c] * projection0 + Jc[9 + c] * projection1);
      double E[3];
      CameraPointRow(cameraJacobians, pointJacobians, observation, c, E);
      for (int p = 0; p < 3; ++p) {
        pointProduct[p] += E[p] * input[cameraBase + c];
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
    for (int c = 0; c < 9; ++c) {
      double E[3];
      CameraPointRow(cameraJacobians, pointJacobians, observation, c, E);
      atomicAdd(&output[cameraBase + c],
                -(E[0] * solvedPoint[0] + E[1] * solvedPoint[1] +
                  E[2] * solvedPoint[2]));
    }
  }
}

__global__ void AddImplicitDampingKernel(int dimension, double lambda,
                                         const double* damping,
                                         const double* input,
                                         double* output) {
  const int scalar = blockIdx.x * blockDim.x + threadIdx.x;
  if (scalar < dimension) {
    output[scalar] +=
        lambda * (damping ? damping[scalar] : 1.0) * input[scalar];
  }
}

__global__ void BuildRhsAndCameraBlocksKernel(
    const CudaSfmObservation* observations, const int* pointOffsets,
    const double* residuals, const double* cameraJacobians,
    const double* pointJacobians, int numPoints, int cameraDimension,
    double lambda, const double* damping, double* rhs, double* blocks) {
  const int point = blockIdx.x * blockDim.x + threadIdx.x;
  if (point >= numPoints) return;
  const int begin = pointOffsets[point];
  const int end = pointOffsets[point + 1];
  double pointBlock[9], pointRhs[3], inverse[9];
  FormPointBlock(residuals, pointJacobians, begin, end,
                 cameraDimension + 3 * point, lambda, damping, pointBlock,
                 pointRhs);
  if (!InvertPointBlock(pointBlock, inverse)) return;

  for (int observation = begin; observation < end; ++observation) {
    const int camera = observations[observation].cameraSlot;
    const int cameraBase = 9 * camera;
    const double* J = cameraJacobians + 18 * observation;
    const double r0 = residuals[2 * observation];
    const double r1 = residuals[2 * observation + 1];
    for (int a = 0; a < 9; ++a) {
      atomicAdd(&rhs[cameraBase + a], -J[a] * r0 - J[9 + a] * r1);
      for (int b = 0; b < 9; ++b) {
        atomicAdd(&blocks[81 * camera + 9 * a + b],
                  J[a] * J[b] + J[9 + a] * J[9 + b]);
      }
    }
  }

  for (int obsI = begin; obsI < end; ++obsI) {
    const int cameraI = observations[obsI].cameraSlot;
    const int cameraBase = 9 * cameraI;
    for (int a = 0; a < 9; ++a) {
      double Ei[3], EiInv[3];
      CameraPointRow(cameraJacobians, pointJacobians, obsI, a, Ei);
      RowTimesInverse(Ei, inverse, EiInv);
      atomicAdd(&rhs[cameraBase + a],
                -(EiInv[0] * pointRhs[0] + EiInv[1] * pointRhs[1] +
                  EiInv[2] * pointRhs[2]));
      for (int obsJ = begin; obsJ < end; ++obsJ) {
        if (observations[obsJ].cameraSlot != cameraI) continue;
        for (int b = 0; b < 9; ++b) {
          double Ej[3];
          CameraPointRow(cameraJacobians, pointJacobians, obsJ, b, Ej);
          atomicAdd(&blocks[81 * cameraI + 9 * a + b],
                    -(EiInv[0] * Ej[0] + EiInv[1] * Ej[1] +
                      EiInv[2] * Ej[2]));
        }
      }
    }
  }
}

__global__ void AddCameraBlockDampingKernel(int numCameras, double lambda,
                                            const double* damping,
                                            double* blocks) {
  const int camera = blockIdx.x * blockDim.x + threadIdx.x;
  if (camera >= numCameras) return;
  for (int scalar = 0; scalar < 9; ++scalar) {
    blocks[81 * camera + 9 * scalar + scalar] +=
        lambda * (damping ? damping[9 * camera + scalar] : 1.0);
  }
}

__global__ void InvertCameraBlocksKernel(int numCameras, const double* blocks,
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

__global__ void ApplyCameraBlocksKernel(int numCameras,
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

int Grid(int count) {
  const size_t blocks =
      (static_cast<size_t>(count) + kBlockSize - 1) / kBlockSize;
  if (blocks > static_cast<size_t>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("implicit Schur launch grid exceeds int range");
  return static_cast<int>(blocks);
}

void ValidateDamping(const CudaSfmProjectionBatch& batch, int numCameras,
                     double lambda,
                     const CudaDeviceArray<double>* dampingDiagonal) {
  if (!(lambda > 0.0) || !std::isfinite(lambda))
    throw std::invalid_argument("implicit Schur requires finite lambda > 0");
  if (dampingDiagonal &&
      dampingDiagonal->size() != 9 * static_cast<size_t>(numCameras) +
                                     3 * batch.numPoints())
    throw std::invalid_argument("implicit Schur damping size mismatch");
}
}  // namespace

CudaSfmSchurOperator::CudaSfmSchurOperator(
    const CudaSfmProjectionBatch& batch,
    const CudaSfmProjectionLinearization& linearization, int numCameras)
    : batch_(&batch), linearization_(&linearization), numCameras_(numCameras) {}

void CudaSfmSchurOperator::configure(
    double lambda, const CudaDeviceArray<double>* dampingDiagonal) {
  ValidateDamping(*batch_, numCameras_, lambda, dampingDiagonal);
  lambda_ = lambda;
  dampingDiagonal_ = dampingDiagonal;
}

void CudaSfmSchurOperator::apply(const double* input, double* output,
                                 cudaStream_t stream) const {
  if (!(lambda_ > 0.0))
    throw std::logic_error("implicit Schur operator is not configured");
  const int cameraDimension = dimension();
  GTSAM_CUDA_CHECK(cudaMemsetAsync(output, 0,
                                   sizeof(double) * cameraDimension, stream));
  const int numPoints = static_cast<int>(batch_->numPoints());
  if (numPoints > 0) {
    ApplyImplicitSchurKernel<<<Grid(numPoints), kBlockSize, 0, stream>>>(
        batch_->observations().data(),
        batch_->pointObservationOffsets().data(),
        linearization_->cameraJacobians.data(),
        linearization_->pointJacobians.data(), numPoints, cameraDimension,
        lambda_, dampingDiagonal_ ? dampingDiagonal_->data() : nullptr, input,
        output);
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
  if (cameraDimension > 0) {
    AddImplicitDampingKernel<<<Grid(cameraDimension), kBlockSize, 0, stream>>>(
        cameraDimension, lambda_,
        dampingDiagonal_ ? dampingDiagonal_->data() : nullptr, input, output);
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

CudaSfmCameraBlockPreconditioner::CudaSfmCameraBlockPreconditioner(
    const CudaSfmProjectionBatch& batch,
    const CudaSfmProjectionLinearization& linearization, int numCameras)
    : batch_(&batch), linearization_(&linearization), numCameras_(numCameras) {}

void CudaSfmCameraBlockPreconditioner::build(
    double lambda, const CudaDeviceArray<double>* dampingDiagonal,
    CudaDeviceArray<double>* condensedRhs, cudaStream_t stream) {
  if (!condensedRhs)
    throw std::invalid_argument("implicit Schur RHS output is null");
  ValidateDamping(*batch_, numCameras_, lambda, dampingDiagonal);
  const int cameraDimension = dimension();
  condensedRhs->resize(cameraDimension);
  condensedRhs->zero(stream);
  cameraBlocks_.resize(81 * static_cast<size_t>(numCameras_));
  cameraBlocks_.zero(stream);
  inverseBlocks_.resize(81 * static_cast<size_t>(numCameras_));
  singularBlocks_.resize(1);
  singularBlocks_.zero(stream);
  const int numPoints = static_cast<int>(batch_->numPoints());
  if (numPoints > 0) {
    BuildRhsAndCameraBlocksKernel<<<Grid(numPoints), kBlockSize, 0, stream>>>(
        batch_->observations().data(),
        batch_->pointObservationOffsets().data(),
        linearization_->residuals.data(),
        linearization_->cameraJacobians.data(),
        linearization_->pointJacobians.data(), numPoints, cameraDimension,
        lambda, dampingDiagonal ? dampingDiagonal->data() : nullptr,
        condensedRhs->data(), cameraBlocks_.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
  if (numCameras_ > 0) {
    AddCameraBlockDampingKernel<<<Grid(numCameras_), kBlockSize, 0, stream>>>(
        numCameras_, lambda,
        dampingDiagonal ? dampingDiagonal->data() : nullptr,
        cameraBlocks_.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
    InvertCameraBlocksKernel<<<Grid(numCameras_), kBlockSize, 0, stream>>>(
        numCameras_, cameraBlocks_.data(), inverseBlocks_.data(),
        singularBlocks_.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

void CudaSfmCameraBlockPreconditioner::apply(const double* input,
                                             double* output,
                                             cudaStream_t stream) const {
  if (inverseBlocks_.size() != 81 * static_cast<size_t>(numCameras_))
    throw std::logic_error("camera block preconditioner is not built");
  if (numCameras_ > 0) {
    ApplyCameraBlocksKernel<<<Grid(numCameras_), kBlockSize, 0, stream>>>(
        numCameras_, inverseBlocks_.data(), input, output);
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

}  // namespace gtsam::cuda
