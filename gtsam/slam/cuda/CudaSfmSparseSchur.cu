#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/slam/cuda/CudaSfmSparseSchur.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace gtsam::cuda {
namespace {
constexpr int kBlockSize = 128;

__device__ int FindEntry(const int* rowPointers, const int* columns, int row,
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

__device__ bool Invert3x3Sparse(const double* A, double* inverse) {
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

__device__ void PointNormal(const double* residuals,
                            const double* pointJacobians, int begin, int end,
                            int pointBase, double lambda,
                            const double* dampingDiagonal, double* block,
                            double* rhs) {
  for (int i = 0; i < 9; ++i) block[i] = 0.0;
  block[0] = lambda * (dampingDiagonal ? dampingDiagonal[pointBase] : 1.0);
  block[4] =
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase + 1] : 1.0);
  block[8] =
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase + 2] : 1.0);
  rhs[0] = rhs[1] = rhs[2] = 0.0;
  for (int observation = begin; observation < end; ++observation) {
    const double r0 = residuals[2 * observation];
    const double r1 = residuals[2 * observation + 1];
    const double* J = pointJacobians + 6 * observation;
    for (int a = 0; a < 3; ++a) {
      rhs[a] += -J[a] * r0 - J[3 + a] * r1;
      for (int b = 0; b < 3; ++b) {
        block[3 * a + b] += J[a] * J[b] + J[3 + a] * J[3 + b];
      }
    }
  }
}

__device__ void CameraPointRowSparse(const double* cameraJacobians,
                                     const double* pointJacobians,
                                     int observation, int row,
                                     double* result) {
  const double* Jc = cameraJacobians + 18 * observation;
  const double* Jp = pointJacobians + 6 * observation;
  for (int p = 0; p < 3; ++p) {
    result[p] = Jc[row] * Jp[p] + Jc[9 + row] * Jp[3 + p];
  }
}

__device__ void TimesInverse(const double* row, const double* inverse,
                             double* result) {
  for (int c = 0; c < 3; ++c) {
    result[c] = row[0] * inverse[c] + row[1] * inverse[3 + c] +
                row[2] * inverse[6 + c];
  }
}

__global__ void AssembleSparseSchurKernel(
    const CudaSfmObservation* observations, const int* pointOffsets,
    const double* residuals, const double* cameraJacobians,
    const double* pointJacobians, int numPoints, int cameraDimension,
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
  PointNormal(residuals, pointJacobians, begin, end,
              cameraDimension + 3 * point, lambda, dampingDiagonal,
              pointBlock, pointRhs);
  if (!Invert3x3Sparse(pointBlock, inverse)) {
    atomicAdd(singularPointBlocks, 1);
    return;
  }

  for (int obs = begin; obs < end; ++obs) {
    const int cameraBase = 9 * observations[obs].cameraSlot;
    const double r0 = residuals[2 * obs];
    const double r1 = residuals[2 * obs + 1];
    const double* J = cameraJacobians + 18 * obs;
    for (int a = 0; a < 9; ++a) {
      atomicAdd(&cameraRhs[cameraBase + a],
                -J[a] * r0 - J[9 + a] * r1);
      for (int b = a; b < 9; ++b) {
        const int entry = FindEntry(rowPointers, columns, cameraBase + a,
                                    cameraBase + b);
        if (entry >= 0) {
          atomicAdd(&values[entry], J[a] * J[b] + J[9 + a] * J[9 + b]);
        }
      }
    }
  }

  for (int obsI = begin; obsI < end; ++obsI) {
    const int cameraIBase = 9 * observations[obsI].cameraSlot;
    for (int a = 0; a < 9; ++a) {
      double Ei[3];
      double EiInv[3];
      CameraPointRowSparse(cameraJacobians, pointJacobians, obsI, a, Ei);
      TimesInverse(Ei, inverse, EiInv);
      atomicAdd(&cameraRhs[cameraIBase + a],
                -(EiInv[0] * pointRhs[0] + EiInv[1] * pointRhs[1] +
                  EiInv[2] * pointRhs[2]));

      for (int obsJ = begin; obsJ < end; ++obsJ) {
        const int cameraJBase = 9 * observations[obsJ].cameraSlot;
        for (int b = 0; b < 9; ++b) {
          int row = cameraIBase + a;
          int column = cameraJBase + b;
          if (row > column) continue;
          double Ej[3];
          CameraPointRowSparse(cameraJacobians, pointJacobians, obsJ, b, Ej);
          const int entry = FindEntry(rowPointers, columns, row, column);
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

__global__ void AddSparseCameraDampingKernel(
    int cameraDimension, double lambda, const double* dampingDiagonal,
    const int* rowPointers, const int* columns, double* values) {
  const int scalar = blockIdx.x * blockDim.x + threadIdx.x;
  if (scalar >= cameraDimension) return;
  const int entry = FindEntry(rowPointers, columns, scalar, scalar);
  if (entry >= 0) {
    values[entry] +=
        lambda * (dampingDiagonal ? dampingDiagonal[scalar] : 1.0);
  }
}

int Grid(int count) {
  const size_t blocks =
      (static_cast<size_t>(count) + kBlockSize - 1) / kBlockSize;
  if (blocks > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("sparse Schur launch grid exceeds int range");
  }
  return static_cast<int>(blocks);
}
}  // namespace

void AssembleCudaSfmSparseSchur(
    const CudaSfmProjectionBatch& batch,
    const CudaSfmProjectionLinearization& linearization, int numCameras,
    double lambda, const CudaDeviceArray<double>* dampingDiagonal,
    DeviceSparseSpdSystem* system, CudaDeviceArray<int>* singularPointBlocks,
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

  const int numPoints = static_cast<int>(batch.numPoints());
  if (numPoints > 0) {
    AssembleSparseSchurKernel<<<Grid(numPoints), kBlockSize, 0, stream>>>(
        batch.observations().data(), batch.pointObservationOffsets().data(),
        linearization.residuals.data(), linearization.cameraJacobians.data(),
        linearization.pointJacobians.data(), numPoints, cameraDimension, lambda,
        dampingDiagonal ? dampingDiagonal->data() : nullptr,
        system->rowPointers().data(), system->colIndices().data(),
        system->values().data(), system->rhs().data(),
        singularPointBlocks->data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
  if (cameraDimension > 0) {
    AddSparseCameraDampingKernel<<<Grid(cameraDimension), kBlockSize, 0,
                                     stream>>>(
        cameraDimension, lambda,
        dampingDiagonal ? dampingDiagonal->data() : nullptr,
        system->rowPointers().data(), system->colIndices().data(),
        system->values().data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
}

}  // namespace gtsam::cuda
