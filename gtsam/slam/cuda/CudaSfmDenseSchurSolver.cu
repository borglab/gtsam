#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/slam/cuda/CudaSfmDenseSchurSolver.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>

#include <algorithm>
#include <cmath>
#include <cusolverDn.h>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {
namespace {

constexpr int kDenseSchurBlockSize = 128;
constexpr int kLongTrackSchurBlockSize = 256;

const char* CusolverStatusName(cusolverStatus_t status) {
  switch (status) {
    case CUSOLVER_STATUS_SUCCESS:
      return "CUSOLVER_STATUS_SUCCESS";
    case CUSOLVER_STATUS_NOT_INITIALIZED:
      return "CUSOLVER_STATUS_NOT_INITIALIZED";
    case CUSOLVER_STATUS_ALLOC_FAILED:
      return "CUSOLVER_STATUS_ALLOC_FAILED";
    case CUSOLVER_STATUS_INVALID_VALUE:
      return "CUSOLVER_STATUS_INVALID_VALUE";
    case CUSOLVER_STATUS_ARCH_MISMATCH:
      return "CUSOLVER_STATUS_ARCH_MISMATCH";
    case CUSOLVER_STATUS_MAPPING_ERROR:
      return "CUSOLVER_STATUS_MAPPING_ERROR";
    case CUSOLVER_STATUS_EXECUTION_FAILED:
      return "CUSOLVER_STATUS_EXECUTION_FAILED";
    case CUSOLVER_STATUS_INTERNAL_ERROR:
      return "CUSOLVER_STATUS_INTERNAL_ERROR";
    case CUSOLVER_STATUS_MATRIX_TYPE_NOT_SUPPORTED:
      return "CUSOLVER_STATUS_MATRIX_TYPE_NOT_SUPPORTED";
    case CUSOLVER_STATUS_NOT_SUPPORTED:
      return "CUSOLVER_STATUS_NOT_SUPPORTED";
    case CUSOLVER_STATUS_ZERO_PIVOT:
      return "CUSOLVER_STATUS_ZERO_PIVOT";
    case CUSOLVER_STATUS_INVALID_LICENSE:
      return "CUSOLVER_STATUS_INVALID_LICENSE";
    case CUSOLVER_STATUS_IRS_PARAMS_NOT_INITIALIZED:
      return "CUSOLVER_STATUS_IRS_PARAMS_NOT_INITIALIZED";
    case CUSOLVER_STATUS_IRS_PARAMS_INVALID:
      return "CUSOLVER_STATUS_IRS_PARAMS_INVALID";
    case CUSOLVER_STATUS_IRS_PARAMS_INVALID_PREC:
      return "CUSOLVER_STATUS_IRS_PARAMS_INVALID_PREC";
    case CUSOLVER_STATUS_IRS_PARAMS_INVALID_REFINE:
      return "CUSOLVER_STATUS_IRS_PARAMS_INVALID_REFINE";
    case CUSOLVER_STATUS_IRS_PARAMS_INVALID_MAXITER:
      return "CUSOLVER_STATUS_IRS_PARAMS_INVALID_MAXITER";
    case CUSOLVER_STATUS_IRS_INTERNAL_ERROR:
      return "CUSOLVER_STATUS_IRS_INTERNAL_ERROR";
    case CUSOLVER_STATUS_IRS_NOT_SUPPORTED:
      return "CUSOLVER_STATUS_IRS_NOT_SUPPORTED";
    case CUSOLVER_STATUS_IRS_OUT_OF_RANGE:
      return "CUSOLVER_STATUS_IRS_OUT_OF_RANGE";
    case CUSOLVER_STATUS_IRS_NRHS_NOT_SUPPORTED_FOR_REFINE_GMRES:
      return "CUSOLVER_STATUS_IRS_NRHS_NOT_SUPPORTED_FOR_REFINE_GMRES";
    case CUSOLVER_STATUS_IRS_INFOS_NOT_INITIALIZED:
      return "CUSOLVER_STATUS_IRS_INFOS_NOT_INITIALIZED";
    case CUSOLVER_STATUS_IRS_INFOS_NOT_DESTROYED:
      return "CUSOLVER_STATUS_IRS_INFOS_NOT_DESTROYED";
    case CUSOLVER_STATUS_IRS_MATRIX_SINGULAR:
      return "CUSOLVER_STATUS_IRS_MATRIX_SINGULAR";
    case CUSOLVER_STATUS_INVALID_WORKSPACE:
      return "CUSOLVER_STATUS_INVALID_WORKSPACE";
  }
  return "CUSOLVER_STATUS_UNKNOWN";
}

void CheckCusolver(cusolverStatus_t status, const char* expression) {
  if (status == CUSOLVER_STATUS_SUCCESS) return;
  std::ostringstream os;
  os << "cuSOLVER call failed: " << expression << " returned "
     << CusolverStatusName(status) << " (" << static_cast<int>(status) << ")";
  throw std::runtime_error(os.str());
}

#define GTSAM_CUSOLVER_CHECK(expr) CheckCusolver((expr), #expr)

struct CusolverDnHandle {
  cusolverDnHandle_t value = nullptr;

  CusolverDnHandle() { GTSAM_CUSOLVER_CHECK(cusolverDnCreate(&value)); }

  CusolverDnHandle(const CusolverDnHandle&) = delete;
  CusolverDnHandle& operator=(const CusolverDnHandle&) = delete;

  ~CusolverDnHandle() {
    if (value) {
      cusolverDnDestroy(value);
    }
  }
};

__device__ int DenseColumnMajorIndex(int row, int col, int dimension) {
  return col * dimension + row;
}

void CheckDeviceInfo(const CudaDeviceArray<int>& info, cudaStream_t stream,
                     const char* operation) {
  std::vector<int> hostInfo;
  info.download(&hostInfo, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  if (!hostInfo.empty() && hostInfo[0] != 0) {
    std::ostringstream os;
    os << operation << " failed with device info " << hostInfo[0];
    throw std::runtime_error(os.str());
  }
}

__device__ void AddCameraNormalBlock(const double* residuals,
                                     const double* cameraJacobians,
                                     int observationIndex,
                                     double* denseCameraSystem, double* rhs,
                                     int cameraDim, int cameraSlot) {
  const int cameraBase = 9 * cameraSlot;
  const double residual0 = residuals[2 * observationIndex];
  const double residual1 = residuals[2 * observationIndex + 1];
  const double* cameraJacobian = cameraJacobians + 18 * observationIndex;
  for (int a = 0; a < 9; ++a) {
    const double Ja0 = cameraJacobian[a];
    const double Ja1 = cameraJacobian[9 + a];
    atomicAdd(&rhs[cameraBase + a], -Ja0 * residual0 - Ja1 * residual1);
    for (int b = 0; b <= a; ++b) {
      const double Jb0 = cameraJacobian[b];
      const double Jb1 = cameraJacobian[9 + b];
      atomicAdd(&denseCameraSystem[DenseColumnMajorIndex(
                    cameraBase + a, cameraBase + b, cameraDim)],
                Ja0 * Jb0 + Ja1 * Jb1);
    }
  }
}

__device__ void AddPointNormalBlock(const double* residuals,
                                    const double* pointJacobians,
                                    int observationIndex,
                                    double* pointBlock, double* pointRhs) {
  const double residual0 = residuals[2 * observationIndex];
  const double residual1 = residuals[2 * observationIndex + 1];
  const double* pointJacobian = pointJacobians + 6 * observationIndex;
  for (int a = 0; a < 3; ++a) {
    const double Ja0 = pointJacobian[a];
    const double Ja1 = pointJacobian[3 + a];
    pointRhs[a] += -Ja0 * residual0 - Ja1 * residual1;
    for (int b = 0; b < 3; ++b) {
      const double Jb0 = pointJacobian[b];
      const double Jb1 = pointJacobian[3 + b];
      pointBlock[3 * a + b] += Ja0 * Jb0 + Ja1 * Jb1;
    }
  }
}

__device__ void CameraPointBlock(const double* cameraJacobians,
                                 const double* pointJacobians,
                                 int observationIndex,
                                 double* block) {
  const double* cameraJacobian = cameraJacobians + 18 * observationIndex;
  const double* pointJacobian = pointJacobians + 6 * observationIndex;
  for (int a = 0; a < 9; ++a) {
    const double Ja0 = cameraJacobian[a];
    const double Ja1 = cameraJacobian[9 + a];
    for (int b = 0; b < 3; ++b) {
      block[3 * a + b] =
          Ja0 * pointJacobian[b] + Ja1 * pointJacobian[3 + b];
    }
  }
}

__device__ void CameraPointRow(const double* cameraJacobians,
                               const double* pointJacobians,
                               int observationIndex, int cameraRow,
                               double* row) {
  const double* cameraJacobian = cameraJacobians + 18 * observationIndex;
  const double* pointJacobian = pointJacobians + 6 * observationIndex;
  const double Ja0 = cameraJacobian[cameraRow];
  const double Ja1 = cameraJacobian[9 + cameraRow];
  for (int c = 0; c < 3; ++c) {
    row[c] = Ja0 * pointJacobian[c] + Ja1 * pointJacobian[3 + c];
  }
}

__device__ void CameraPointRowTimesInvPoint(const double* cameraJacobians,
                                            const double* pointJacobians,
                                            int observationIndex,
                                            int cameraRow,
                                            const double* invPoint,
                                            double* row) {
  double cameraPointRow[3];
  CameraPointRow(cameraJacobians, pointJacobians, observationIndex, cameraRow,
                 cameraPointRow);
  for (int c = 0; c < 3; ++c) {
    row[c] = cameraPointRow[0] * invPoint[c] +
             cameraPointRow[1] * invPoint[3 + c] +
             cameraPointRow[2] * invPoint[6 + c];
  }
}

__device__ void DecodeLowerTriangular9(int index, int* row, int* col) {
  int r = 0;
  while (index > r) {
    index -= r + 1;
    ++r;
  }
  *row = r;
  *col = index;
}

__device__ bool Invert3x3(const double* A, double* inverse) {
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
  if (fabs(det) < 1e-30) {
    return false;
  }
  const double invDet = 1.0 / det;
  inverse[0] = c00 * invDet;
  inverse[1] = c01 * invDet;
  inverse[2] = c02 * invDet;
  inverse[3] = c10 * invDet;
  inverse[4] = c11 * invDet;
  inverse[5] = c12 * invDet;
  inverse[6] = c20 * invDet;
  inverse[7] = c21 * invDet;
  inverse[8] = c22 * invDet;
  return true;
}

__device__ void ComputePointNormal(
    const double* residuals, const double* pointJacobians, int begin, int end,
    double lambda, const double* dampingDiagonal, int pointBase,
    double* pointBlock, double* pointRhs) {
  for (int i = 0; i < 9; ++i) {
    pointBlock[i] = 0.0;
  }
  pointBlock[0] =
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase] : 1.0);
  pointBlock[4] =
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase + 1] : 1.0);
  pointBlock[8] =
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase + 2] : 1.0);
  pointRhs[0] = 0.0;
  pointRhs[1] = 0.0;
  pointRhs[2] = 0.0;

  for (int obsIndex = begin; obsIndex < end; ++obsIndex) {
    AddPointNormalBlock(residuals, pointJacobians, obsIndex, pointBlock,
                        pointRhs);
  }
}

__device__ void MultiplyCameraPointByInvPoint(const double* cameraPoint,
                                              const double* invPoint,
                                              double* result) {
  for (int r = 0; r < 9; ++r) {
    for (int c = 0; c < 3; ++c) {
      result[3 * r + c] = cameraPoint[3 * r] * invPoint[c] +
                          cameraPoint[3 * r + 1] * invPoint[3 + c] +
                          cameraPoint[3 * r + 2] * invPoint[6 + c];
    }
  }
}

__global__ void AccumulateDenseSchurKernel(
    const CudaSfmObservation* observations, const int* pointOffsets,
    const double* residuals, const double* cameraJacobians,
    const double* pointJacobians, int numPoints, int cameraDim, double lambda,
    const double* dampingDiagonal, double* denseCameraSystem,
    double* cameraRhs, int* singularPointBlocks) {
  const int pointSlot = blockIdx.x * blockDim.x + threadIdx.x;
  if (pointSlot >= numPoints) return;

  const int begin = pointOffsets[pointSlot];
  const int end = pointOffsets[pointSlot + 1];
  if (end - begin > CudaSfmProjectionBatch::kLongTrackMeasurementThreshold) {
    return;
  }
  double pointBlock[9];
  double pointRhs[3];
  double invPoint[9];
  const int pointBase = cameraDim + 3 * pointSlot;
  ComputePointNormal(residuals, pointJacobians, begin, end, lambda,
                     dampingDiagonal, pointBase, pointBlock, pointRhs);
  if (!Invert3x3(pointBlock, invPoint)) {
    atomicAdd(singularPointBlocks, 1);
    return;
  }

  for (int obsIndex = begin; obsIndex < end; ++obsIndex) {
    const CudaSfmObservation observation = observations[obsIndex];
    AddCameraNormalBlock(residuals, cameraJacobians, obsIndex,
                         denseCameraSystem, cameraRhs, cameraDim,
                         observation.cameraSlot);
  }

  for (int obsI = begin; obsI < end; ++obsI) {
    const CudaSfmObservation observationI = observations[obsI];
    double E_i[27];
    double E_i_Cinv[27];
    CameraPointBlock(cameraJacobians, pointJacobians, obsI, E_i);
    MultiplyCameraPointByInvPoint(E_i, invPoint, E_i_Cinv);

    const int cameraIBase = 9 * observationI.cameraSlot;
    for (int a = 0; a < 9; ++a) {
      const double rhsCorrection = E_i_Cinv[3 * a] * pointRhs[0] +
                                   E_i_Cinv[3 * a + 1] * pointRhs[1] +
                                   E_i_Cinv[3 * a + 2] * pointRhs[2];
      atomicAdd(&cameraRhs[cameraIBase + a], -rhsCorrection);
    }

    for (int obsJ = begin; obsJ < end; ++obsJ) {
      const CudaSfmObservation observationJ = observations[obsJ];
      double E_j[27];
      CameraPointBlock(cameraJacobians, pointJacobians, obsJ, E_j);

      const int cameraJBase = 9 * observationJ.cameraSlot;
      for (int a = 0; a < 9; ++a) {
        const int row = cameraIBase + a;
        for (int b = 0; b < 9; ++b) {
          const int col = cameraJBase + b;
          if (row < col) {
            continue;
          }
          const double value = E_i_Cinv[3 * a] * E_j[3 * b] +
                               E_i_Cinv[3 * a + 1] * E_j[3 * b + 1] +
                               E_i_Cinv[3 * a + 2] * E_j[3 * b + 2];
          atomicAdd(&denseCameraSystem[DenseColumnMajorIndex(row, col,
                                                             cameraDim)],
                    -value);
        }
      }
    }
  }
}

__global__ void AccumulateLongTrackDenseSchurKernel(
    const int* longTrackPointSlots, int numLongTrackPoints,
    const CudaSfmObservation* observations, const int* pointOffsets,
    const double* residuals, const double* cameraJacobians,
    const double* pointJacobians, int cameraDim, double lambda,
    const double* dampingDiagonal, double* denseCameraSystem,
    double* cameraRhs, int* singularPointBlocks) {
  const int listIndex = blockIdx.x;
  if (listIndex >= numLongTrackPoints) return;

  const int pointSlot = longTrackPointSlots[listIndex];
  const int begin = pointOffsets[pointSlot];
  const int end = pointOffsets[pointSlot + 1];
  const int degree = end - begin;
  if (degree <= CudaSfmProjectionBatch::kLongTrackMeasurementThreshold) return;

  __shared__ double pointBlock[9];
  __shared__ double pointRhs[3];
  __shared__ double invPoint[9];
  __shared__ int invertible;

  if (threadIdx.x == 0) {
    const int pointBase = cameraDim + 3 * pointSlot;
    ComputePointNormal(residuals, pointJacobians, begin, end, lambda,
                       dampingDiagonal, pointBase, pointBlock, pointRhs);
    invertible = Invert3x3(pointBlock, invPoint) ? 1 : 0;
    if (!invertible) {
      atomicAdd(singularPointBlocks, 1);
    }
  }
  __syncthreads();
  if (!invertible) return;

  constexpr int kCameraDirectEntries = 9 + 45;
  for (int work = threadIdx.x; work < degree * kCameraDirectEntries;
       work += blockDim.x) {
    const int obsIndex = begin + work / kCameraDirectEntries;
    const int local = work % kCameraDirectEntries;
    const CudaSfmObservation observation = observations[obsIndex];
    const int cameraBase = 9 * observation.cameraSlot;
    const double residual0 = residuals[2 * obsIndex];
    const double residual1 = residuals[2 * obsIndex + 1];
    const double* cameraJacobian = cameraJacobians + 18 * obsIndex;

    if (local < 9) {
      const int a = local;
      atomicAdd(&cameraRhs[cameraBase + a],
                -cameraJacobian[a] * residual0 -
                    cameraJacobian[9 + a] * residual1);
    } else {
      int a;
      int b;
      DecodeLowerTriangular9(local - 9, &a, &b);
      atomicAdd(&denseCameraSystem[DenseColumnMajorIndex(
                    cameraBase + a, cameraBase + b, cameraDim)],
                cameraJacobian[a] * cameraJacobian[b] +
                    cameraJacobian[9 + a] * cameraJacobian[9 + b]);
    }
  }

  for (int work = threadIdx.x; work < degree * 9; work += blockDim.x) {
    const int obsIndex = begin + work / 9;
    const int a = work % 9;
    const CudaSfmObservation observation = observations[obsIndex];
    const int cameraBase = 9 * observation.cameraSlot;
    double E_i_Cinv_row[3];
    CameraPointRowTimesInvPoint(cameraJacobians, pointJacobians, obsIndex, a,
                                invPoint, E_i_Cinv_row);
    const double rhsCorrection = E_i_Cinv_row[0] * pointRhs[0] +
                                 E_i_Cinv_row[1] * pointRhs[1] +
                                 E_i_Cinv_row[2] * pointRhs[2];
    atomicAdd(&cameraRhs[cameraBase + a], -rhsCorrection);
  }

  const int pairScalarCount = degree * degree * 81;
  for (int work = threadIdx.x; work < pairScalarCount; work += blockDim.x) {
    int remaining = work;
    const int b = remaining % 9;
    remaining /= 9;
    const int a = remaining % 9;
    remaining /= 9;
    const int obsJ = begin + remaining % degree;
    const int obsI = begin + remaining / degree;

    const CudaSfmObservation observationI = observations[obsI];
    const CudaSfmObservation observationJ = observations[obsJ];
    const int row = 9 * observationI.cameraSlot + a;
    const int col = 9 * observationJ.cameraSlot + b;
    if (row < col) {
      continue;
    }

    double E_i_Cinv_row[3];
    double E_j_row[3];
    CameraPointRowTimesInvPoint(cameraJacobians, pointJacobians, obsI, a,
                                invPoint, E_i_Cinv_row);
    CameraPointRow(cameraJacobians, pointJacobians, obsJ, b, E_j_row);
    const double value = E_i_Cinv_row[0] * E_j_row[0] +
                         E_i_Cinv_row[1] * E_j_row[1] +
                         E_i_Cinv_row[2] * E_j_row[2];
    atomicAdd(&denseCameraSystem[DenseColumnMajorIndex(row, col, cameraDim)],
              -value);
  }
}

__global__ void AddCameraDampingKernel(int cameraDim, double lambda,
                                       const double* dampingDiagonal,
                                       double* denseCameraSystem) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < cameraDim) {
    denseCameraSystem[i * cameraDim + i] +=
        lambda * (dampingDiagonal ? dampingDiagonal[i] : 1.0);
  }
}

__global__ void RecoverPointDeltaKernel(
    const CudaSfmObservation* observations, const int* pointOffsets,
    const double* residuals, const double* cameraJacobians,
    const double* pointJacobians, int numPoints, int cameraDim, double lambda,
    const double* dampingDiagonal, double* delta, int* singularPointBlocks) {
  const int pointSlot = blockIdx.x * blockDim.x + threadIdx.x;
  if (pointSlot >= numPoints) return;

  const int begin = pointOffsets[pointSlot];
  const int end = pointOffsets[pointSlot + 1];
  double pointBlock[9];
  double pointRhs[3];
  double invPoint[9];
  const int pointBase = cameraDim + 3 * pointSlot;
  ComputePointNormal(residuals, pointJacobians, begin, end, lambda,
                     dampingDiagonal, pointBase, pointBlock, pointRhs);
  if (!Invert3x3(pointBlock, invPoint)) {
    atomicAdd(singularPointBlocks, 1);
    return;
  }

  double reducedRhs[3] = {pointRhs[0], pointRhs[1], pointRhs[2]};
  for (int obsIndex = begin; obsIndex < end; ++obsIndex) {
    const CudaSfmObservation observation = observations[obsIndex];
    const double* cameraJacobian = cameraJacobians + 18 * obsIndex;
    const double* pointJacobian = pointJacobians + 6 * obsIndex;
    const int cameraBase = 9 * observation.cameraSlot;
    for (int p = 0; p < 3; ++p) {
      double correction = 0.0;
      for (int c = 0; c < 9; ++c) {
        const double E_cp =
            cameraJacobian[c] * pointJacobian[p] +
            cameraJacobian[9 + c] * pointJacobian[3 + p];
        correction += E_cp * delta[cameraBase + c];
      }
      reducedRhs[p] -= correction;
    }
  }

  double* pointDelta = delta + cameraDim + 3 * pointSlot;
  for (int r = 0; r < 3; ++r) {
    pointDelta[r] = invPoint[3 * r] * reducedRhs[0] +
                    invPoint[3 * r + 1] * reducedRhs[1] +
                    invPoint[3 * r + 2] * reducedRhs[2];
  }
}

void SolveDenseCameraSystemOnDevice(CusolverDnHandle* handle,
                                    CudaDeviceArray<double>* workspace,
                                    CudaDeviceArray<int>* info,
                                    CudaDeviceArray<double>* denseCameraSystem,
                                    CudaDeviceArray<double>* cameraRhs,
                                    int cameraDim, cudaStream_t stream) {
  GTSAM_CUSOLVER_CHECK(cusolverDnSetStream(handle->value, stream));
  int workspaceSize = 0;
  GTSAM_CUSOLVER_CHECK(cusolverDnDpotrf_bufferSize(
      handle->value, CUBLAS_FILL_MODE_LOWER, cameraDim,
      denseCameraSystem->data(), cameraDim, &workspaceSize));
  workspace->resize(static_cast<size_t>(workspaceSize));
  info->resize(1);

  GTSAM_CUSOLVER_CHECK(cusolverDnDpotrf(
      handle->value, CUBLAS_FILL_MODE_LOWER, cameraDim,
      denseCameraSystem->data(), cameraDim, workspace->data(), workspaceSize,
      info->data()));
  CheckDeviceInfo(*info, stream, "cusolverDnDpotrf");

  GTSAM_CUSOLVER_CHECK(cusolverDnDpotrs(
      handle->value, CUBLAS_FILL_MODE_LOWER, cameraDim, 1,
      denseCameraSystem->data(), cameraDim, cameraRhs->data(), cameraDim,
      info->data()));
  CheckDeviceInfo(*info, stream, "cusolverDnDpotrs");
}

void CheckSchurInputs(const DeviceValues& values,
                      const CudaSfmProjectionBatch& batch, int numCameras,
                      double lambda,
                      const CudaDeviceArray<double>* dampingDiagonal,
                      const CudaDeviceArray<double>* delta) {
  if (!delta) {
    throw std::invalid_argument("SolveCudaSfmDenseSchur requires delta output");
  }
  if (numCameras < 0) {
    throw std::invalid_argument("SolveCudaSfmDenseSchur numCameras < 0");
  }
  if (lambda <= 0.0) {
    throw std::invalid_argument("SolveCudaSfmDenseSchur requires lambda > 0");
  }
  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);
  if (batch.numCameras() > cameraBlock.values.size() ||
      static_cast<size_t>(numCameras) < batch.numCameras()) {
    throw std::invalid_argument(
        "SolveCudaSfmDenseSchur camera batch/value size mismatch");
  }
  if (batch.numPoints() > pointBlock.values.size()) {
    throw std::invalid_argument(
        "SolveCudaSfmDenseSchur point batch/value size mismatch");
  }
  if (batch.pointObservationOffsets().size() != batch.numPoints() + 1) {
    throw std::invalid_argument(
        "SolveCudaSfmDenseSchur point offset size mismatch");
  }
  const size_t totalDim =
      9 * static_cast<size_t>(numCameras) + 3 * batch.numPoints();
  if (dampingDiagonal && dampingDiagonal->size() != totalDim) {
    throw std::invalid_argument(
        "SolveCudaSfmDenseSchur damping diagonal size mismatch");
  }
}

}  // namespace

struct CudaSfmDenseSchurSolver::Impl {
  CusolverDnHandle cusolver;
  CudaDeviceArray<double> denseCameraSystem;
  CudaDeviceArray<double> cameraRhs;
  CudaSfmProjectionLinearization linearization;
  CudaDeviceArray<double> workspace;
  CudaDeviceArray<int> info;
  CudaDeviceArray<int> singularPointBlocks;

  void solve(const DeviceValues& values, const CudaSfmProjectionBatch& batch,
             int numCameras, double lambda, CudaDeviceArray<double>* delta,
             cudaStream_t stream) {
    solve(values, batch, numCameras, lambda, nullptr, delta, stream);
  }

  void solve(const DeviceValues& values, const CudaSfmProjectionBatch& batch,
             int numCameras, double lambda,
             const CudaDeviceArray<double>* dampingDiagonal,
             CudaDeviceArray<double>* delta, cudaStream_t stream) {
    CheckSchurInputs(values, batch, numCameras, lambda, dampingDiagonal, delta);
    const int cameraDim = 9 * numCameras;
    const int numPoints = static_cast<int>(batch.numPoints());
    const int totalDim = cameraDim + 3 * numPoints;
    delta->resize(static_cast<size_t>(totalDim));
    if (totalDim == 0) {
      return;
    }

    if (cameraDim <= 0) {
      delta->zero(stream);
      return;
    }
    if (static_cast<size_t>(cameraDim) >
        std::numeric_limits<size_t>::max() / static_cast<size_t>(cameraDim)) {
      throw std::invalid_argument(
          "SolveCudaSfmDenseSchur camera system too big");
    }

    denseCameraSystem.resize(static_cast<size_t>(cameraDim) *
                             static_cast<size_t>(cameraDim));
    cameraRhs.resize(static_cast<size_t>(cameraDim));
    singularPointBlocks.resize(1);
    denseCameraSystem.zero(stream);
    cameraRhs.zero(stream);
    singularPointBlocks.zero(stream);
    LinearizeCudaSfmProjectionBatch(values, batch, &linearization, stream);

    const int pointGrid =
        (numPoints + kDenseSchurBlockSize - 1) / kDenseSchurBlockSize;
    if (pointGrid > 0) {
      AccumulateDenseSchurKernel<<<pointGrid, kDenseSchurBlockSize, 0,
                                   stream>>>(
          batch.observations().data(), batch.pointObservationOffsets().data(),
          linearization.residuals.data(), linearization.cameraJacobians.data(),
          linearization.pointJacobians.data(), numPoints, cameraDim, lambda,
          dampingDiagonal ? dampingDiagonal->data() : nullptr,
          denseCameraSystem.data(), cameraRhs.data(),
          singularPointBlocks.data());
      GTSAM_CUDA_CHECK(cudaGetLastError());
    }

    const int longTrackGrid =
        static_cast<int>(batch.longTrackPointSlots().size());
    if (longTrackGrid > 0) {
      AccumulateLongTrackDenseSchurKernel<<<longTrackGrid,
                                            kLongTrackSchurBlockSize, 0,
                                            stream>>>(
          batch.longTrackPointSlots().data(), longTrackGrid,
          batch.observations().data(), batch.pointObservationOffsets().data(),
          linearization.residuals.data(), linearization.cameraJacobians.data(),
          linearization.pointJacobians.data(), cameraDim, lambda,
          dampingDiagonal ? dampingDiagonal->data() : nullptr,
          denseCameraSystem.data(), cameraRhs.data(),
          singularPointBlocks.data());
      GTSAM_CUDA_CHECK(cudaGetLastError());
    }

    const int cameraGrid =
        (cameraDim + kDenseSchurBlockSize - 1) / kDenseSchurBlockSize;
    AddCameraDampingKernel<<<cameraGrid, kDenseSchurBlockSize, 0, stream>>>(
        cameraDim, lambda, dampingDiagonal ? dampingDiagonal->data() : nullptr,
        denseCameraSystem.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());

    SolveDenseCameraSystemOnDevice(&cusolver, &workspace, &info,
                                   &denseCameraSystem, &cameraRhs, cameraDim,
                                   stream);
    delta->zero(stream);
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(delta->data(), cameraRhs.data(),
                                     sizeof(double) * cameraRhs.size(),
                                     cudaMemcpyDeviceToDevice, stream));

    singularPointBlocks.zero(stream);
    if (pointGrid > 0) {
      RecoverPointDeltaKernel<<<pointGrid, kDenseSchurBlockSize, 0, stream>>>(
          batch.observations().data(), batch.pointObservationOffsets().data(),
          linearization.residuals.data(), linearization.cameraJacobians.data(),
          linearization.pointJacobians.data(), numPoints, cameraDim, lambda,
          dampingDiagonal ? dampingDiagonal->data() : nullptr, delta->data(),
          singularPointBlocks.data());
      GTSAM_CUDA_CHECK(cudaGetLastError());
    }
  }
};

CudaSfmDenseSchurSolver::CudaSfmDenseSchurSolver()
    : impl_(std::make_unique<Impl>()) {}

CudaSfmDenseSchurSolver::~CudaSfmDenseSchurSolver() = default;

CudaSfmDenseSchurSolver::CudaSfmDenseSchurSolver(
    CudaSfmDenseSchurSolver&&) noexcept = default;

CudaSfmDenseSchurSolver& CudaSfmDenseSchurSolver::operator=(
    CudaSfmDenseSchurSolver&&) noexcept = default;

void CudaSfmDenseSchurSolver::solve(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, double lambda, CudaDeviceArray<double>* delta,
    cudaStream_t stream) {
  impl_->solve(values, batch, numCameras, lambda, delta, stream);
}

void CudaSfmDenseSchurSolver::solve(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, double lambda,
    const CudaDeviceArray<double>& dampingDiagonal,
    CudaDeviceArray<double>* delta, cudaStream_t stream) {
  impl_->solve(values, batch, numCameras, lambda, &dampingDiagonal, delta,
              stream);
}

void SolveCudaSfmDenseSchur(const DeviceValues& values,
                            const CudaSfmProjectionBatch& batch,
                            int numCameras, double lambda,
                            CudaDeviceArray<double>* delta,
                            cudaStream_t stream) {
  CudaSfmDenseSchurSolver solver;
  solver.solve(values, batch, numCameras, lambda, delta, stream);
}

void SolveCudaSfmDenseSchur(const DeviceValues& values,
                            const CudaSfmProjectionBatch& batch,
                            int numCameras, double lambda,
                            const CudaDeviceArray<double>& dampingDiagonal,
                            CudaDeviceArray<double>* delta,
                            cudaStream_t stream) {
  CudaSfmDenseSchurSolver solver;
  solver.solve(values, batch, numCameras, lambda, dampingDiagonal, delta,
               stream);
}

}  // namespace gtsam::cuda
