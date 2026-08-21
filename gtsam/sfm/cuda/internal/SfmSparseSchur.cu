/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmSparseSchur.cu
 * @brief   Numerical assembly of the camera-only upper-CSR Schur complement
 * @author  Ruogu Li
 * @date    Aug 16, 2026
 */

#include <gtsam/base/cuda/Errors.h>
#include <gtsam/sfm/cuda/internal/SfmSchurProblem.h>
#include <gtsam/sfm/cuda/internal/SfmSparseSchur.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace gtsam::cuda {
namespace {
constexpr int kBlockSize = 128;

__device__ int findEntry(const int* rowPointers, const int* columns, int row,
                         int column) {
  int first = rowPointers[row];
  int last = rowPointers[row + 1];
  while (first < last) {
    const int middle = first + (last - first) / 2;
    if (columns[middle] < column) {
      first = middle + 1;
    } else {
      last = middle;
    }
  }
  return first < rowPointers[row + 1] && columns[first] == column ? first : -1;
}

__device__ bool invert3x3Sparse(const double* A, double* inverse) {
  const double c00 = A[4] * A[8] - A[5] * A[7];
  const double c01 = A[2] * A[7] - A[1] * A[8];
  const double c02 = A[1] * A[5] - A[2] * A[4];
  const double c10 = A[5] * A[6] - A[3] * A[8];
  const double c11 = A[0] * A[8] - A[2] * A[6];
  const double c12 = A[2] * A[3] - A[0] * A[5];
  const double c20 = A[3] * A[7] - A[4] * A[6];
  const double c21 = A[1] * A[6] - A[0] * A[7];
  const double c22 = A[0] * A[4] - A[1] * A[3];
  const double det = A[0] * c00 + A[1] * c10 + A[2] * c20;
  if (fabs(det) < 1e-30) return false;
  const double scale = 1.0 / det;
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

__device__ void timesInverse(const double* row, const double* inverse,
                             double* result) {
  for (int c = 0; c < 3; ++c) {
    result[c] = row[0] * inverse[c] + row[1] * inverse[3 + c] +
                row[2] * inverse[6 + c];
  }
}

__global__ void initializeSparseCameraBlocksKernel(
    int numCameras, const double* cameraNormalBlocks,
    const double* cameraGradient, const int* rowPointers,
    const int* columns, double* values, double* rhs) {
  const int camera = blockIdx.x * blockDim.x + threadIdx.x;
  if (camera >= numCameras) return;
  const int base = 9 * camera;
  const double* U = cameraNormalBlocks + 81 * camera;
  for (int row = 0; row < 9; ++row) {
    rhs[base + row] = cameraGradient[base + row];
    for (int column = row; column < 9; ++column) {
      const int entry =
          findEntry(rowPointers, columns, base + row, base + column);
      if (entry >= 0) values[entry] = U[9 * row + column];
    }
  }
}

__global__ void assembleSparseSchurKernel(
    const SfmObservation* observations, const int* pointOffsets,
    const double* pointNormalBlocks, const double* pointGradient,
    const double* cameraPointBlocks, int numPoints, int cameraDimension,
    double lambda, const double* dampingDiagonal, const int* rowPointers,
    const int* columns, double* values, double* cameraRhs,
    int* singularPointBlocks) {
  const int point = blockIdx.x * blockDim.x + threadIdx.x;
  if (point >= numPoints) return;
  const int begin = pointOffsets[point];
  const int end = pointOffsets[point + 1];

  double pointBlock[9];
  double pointRhs[3];
  double inverse[9];
  const double* undampedPoint = pointNormalBlocks + 9 * point;
  const double* undampedRhs = pointGradient + 3 * point;
  for (int i = 0; i < 9; ++i) pointBlock[i] = undampedPoint[i];
  const int pointBase = cameraDimension + 3 * point;
  pointBlock[0] +=
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase] : 1.0);
  pointBlock[4] +=
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase + 1] : 1.0);
  pointBlock[8] +=
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase + 2] : 1.0);
  pointRhs[0] = undampedRhs[0];
  pointRhs[1] = undampedRhs[1];
  pointRhs[2] = undampedRhs[2];
  if (!invert3x3Sparse(pointBlock, inverse)) {
    atomicAdd(singularPointBlocks, 1);
    return;
  }

  for (int obsI = begin; obsI < end; ++obsI) {
    const int cameraIBase = 9 * observations[obsI].cameraSlot;
    for (int a = 0; a < 9; ++a) {
      const double* Ei = cameraPointBlocks + 27 * obsI + 3 * a;
      double EiInv[3];
      timesInverse(Ei, inverse, EiInv);
      atomicAdd(&cameraRhs[cameraIBase + a],
                -(EiInv[0] * pointRhs[0] + EiInv[1] * pointRhs[1] +
                  EiInv[2] * pointRhs[2]));

      for (int obsJ = begin; obsJ < end; ++obsJ) {
        const int cameraJBase = 9 * observations[obsJ].cameraSlot;
        for (int b = 0; b < 9; ++b) {
          int row = cameraIBase + a;
          int column = cameraJBase + b;
          if (row > column) continue;
          const double* Ej = cameraPointBlocks + 27 * obsJ + 3 * b;
          const int entry = findEntry(rowPointers, columns, row, column);
          if (entry >= 0) {
            atomicAdd(&values[entry],
                      -(EiInv[0] * Ej[0] + EiInv[1] * Ej[1] +
                        EiInv[2] * Ej[2]));
          }
        }
      }
    }
  }
}

__global__ void addSparseCameraDampingKernel(
    int cameraDimension, double lambda, const double* dampingDiagonal,
    const int* rowPointers, const int* columns, double* values) {
  const int scalar = blockIdx.x * blockDim.x + threadIdx.x;
  if (scalar >= cameraDimension) return;
  const int entry = findEntry(rowPointers, columns, scalar, scalar);
  if (entry >= 0) {
    values[entry] +=
        lambda * (dampingDiagonal ? dampingDiagonal[scalar] : 1.0);
  }
}

int grid(int count) {
  const size_t blocks =
      (static_cast<size_t>(count) + kBlockSize - 1) / kBlockSize;
  if (blocks > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("sparse Schur launch grid exceeds int range");
  }
  return static_cast<int>(blocks);
}
}  // namespace

void assembleSfmSparseSchur(
    const SfmProjectionBatch& batch,
    const SfmSchurBlocks& blocks, int numCameras,
    double lambda, const DeviceArray<double>* dampingDiagonal,
    DeviceSparseSpdSystem* system, DeviceArray<int>* singularPointBlocks,
    cudaStream_t stream) {
  if (!system || !singularPointBlocks) {
    throw std::invalid_argument("sparse Schur output storage is null");
  }
  const int cameraDimension = 9 * numCameras;
  if (system->rows() != cameraDimension) {
    throw std::invalid_argument("sparse Schur CSR dimension mismatch");
  }
  if (dampingDiagonal &&
      dampingDiagonal->size() !=
          static_cast<size_t>(cameraDimension + 3 * batch.numPoints())) {
    throw std::invalid_argument("sparse Schur damping dimension mismatch");
  }
  system->zero(stream);
  singularPointBlocks->resize(1);
  singularPointBlocks->zero(stream);

  if (numCameras > 0) {
    initializeSparseCameraBlocksKernel<<<grid(numCameras), kBlockSize, 0,
                                         stream>>>(
        numCameras, blocks.cameraNormalBlocks.data(),
        blocks.cameraGradient.data(), system->rowPointers().data(),
        system->colIndices().data(), system->values().data(),
        system->rhs().data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }

  const int numPoints = static_cast<int>(batch.numPoints());
  if (numPoints > 0) {
    assembleSparseSchurKernel<<<grid(numPoints), kBlockSize, 0, stream>>>(
        batch.observations().data(), batch.pointObservationOffsets().data(),
        blocks.pointNormalBlocks.data(), blocks.pointGradient.data(),
        blocks.cameraPointBlocks.data(), numPoints, cameraDimension, lambda,
        dampingDiagonal ? dampingDiagonal->data() : nullptr,
        system->rowPointers().data(), system->colIndices().data(),
        system->values().data(), system->rhs().data(),
        singularPointBlocks->data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
  if (cameraDimension > 0) {
    addSparseCameraDampingKernel<<<grid(cameraDimension), kBlockSize, 0,
                                     stream>>>(
        cameraDimension, lambda,
        dampingDiagonal ? dampingDiagonal->data() : nullptr,
        system->rowPointers().data(), system->colIndices().data(),
        system->values().data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

}  // namespace gtsam::cuda
