#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryTypes.h>
#include <gtsam/slam/cuda/CudaSfmDenseSchurSolver.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>
#include <gtsam/slam/cuda/CudaSfmReducedCsrPlan.h>
#include <gtsam/slam/cuda/CudaSfmSchurProblem.h>
#include <gtsam/slam/cuda/CudaSfmSchurOperator.h>
#include <gtsam/slam/cuda/CudaSfmSparseSchur.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {
namespace {

constexpr int kDenseSchurBlockSize = 128;
constexpr int kLongTrackSchurBlockSize = 256;

__global__ void BuildPersistentSchurBlocksKernel(
    const CudaSfmObservation* observations, const double* residuals,
    const double* cameraJacobians, const double* pointJacobians,
    int numObservations, double* cameraNormalBlocks, double* cameraGradient,
    double* pointNormalBlocks, double* pointGradient,
    double* cameraPointBlocks) {
  const int observation = blockIdx.x * blockDim.x + threadIdx.x;
  if (observation >= numObservations) return;

  const int camera = observations[observation].cameraSlot;
  const int point = observations[observation].pointSlot;
  const double r0 = residuals[2 * observation];
  const double r1 = residuals[2 * observation + 1];
  const double* Jc = cameraJacobians + 18 * observation;
  const double* Jp = pointJacobians + 6 * observation;
  double* U = cameraNormalBlocks + 81 * camera;
  double* gc = cameraGradient + 9 * camera;
  double* V = pointNormalBlocks + 9 * point;
  double* gp = pointGradient + 3 * point;
  double* W = cameraPointBlocks + 27 * observation;

  for (int row = 0; row < 9; ++row) {
    atomicAdd(&gc[row], -Jc[row] * r0 - Jc[9 + row] * r1);
    for (int column = 0; column < 9; ++column) {
      atomicAdd(&U[9 * row + column],
                Jc[row] * Jc[column] +
                    Jc[9 + row] * Jc[9 + column]);
    }
    for (int column = 0; column < 3; ++column) {
      W[3 * row + column] =
          Jc[row] * Jp[column] + Jc[9 + row] * Jp[3 + column];
    }
  }
  for (int row = 0; row < 3; ++row) {
    atomicAdd(&gp[row], -Jp[row] * r0 - Jp[3 + row] * r1);
    for (int column = 0; column < 3; ++column) {
      atomicAdd(&V[3 * row + column],
                Jp[row] * Jp[column] +
                    Jp[3 + row] * Jp[3 + column]);
    }
  }
}

__device__ int DenseColumnMajorIndex(int row, int col, int dimension) {
  return col * dimension + row;
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

__device__ void LoadDampedPointNormal(
    const double* pointNormalBlocks, const double* pointGradient,
    int pointSlot, int pointBase, double lambda,
    const double* dampingDiagonal, double* pointBlock, double* pointRhs) {
  const double* undamped = pointNormalBlocks + 9 * pointSlot;
  const double* gradient = pointGradient + 3 * pointSlot;
  for (int i = 0; i < 9; ++i) pointBlock[i] = undamped[i];
  pointBlock[0] +=
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase] : 1.0);
  pointBlock[4] +=
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase + 1] : 1.0);
  pointBlock[8] +=
      lambda * (dampingDiagonal ? dampingDiagonal[pointBase + 2] : 1.0);
  pointRhs[0] = gradient[0];
  pointRhs[1] = gradient[1];
  pointRhs[2] = gradient[2];
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
    const double* pointNormalBlocks, const double* pointGradient,
    const double* cameraPointBlocks, int numPoints, int cameraDim,
    double lambda, const double* dampingDiagonal,
    double* denseCameraSystem, double* cameraRhs,
    int* singularPointBlocks) {
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
  LoadDampedPointNormal(pointNormalBlocks, pointGradient, pointSlot,
                        pointBase, lambda, dampingDiagonal, pointBlock,
                        pointRhs);
  if (!Invert3x3(pointBlock, invPoint)) {
    atomicAdd(singularPointBlocks, 1);
    return;
  }

  for (int obsI = begin; obsI < end; ++obsI) {
    const CudaSfmObservation observationI = observations[obsI];
    double E_i_Cinv[27];
    const double* E_i = cameraPointBlocks + 27 * obsI;
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
      const double* E_j = cameraPointBlocks + 27 * obsJ;

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
    const double* pointNormalBlocks, const double* pointGradient,
    const double* cameraPointBlocks, int cameraDim, double lambda,
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
    LoadDampedPointNormal(pointNormalBlocks, pointGradient, pointSlot,
                          pointBase, lambda, dampingDiagonal, pointBlock,
                          pointRhs);
    invertible = Invert3x3(pointBlock, invPoint) ? 1 : 0;
    if (!invertible) {
      atomicAdd(singularPointBlocks, 1);
    }
  }
  __syncthreads();
  if (!invertible) return;

  for (int work = threadIdx.x; work < degree * 9; work += blockDim.x) {
    const int obsIndex = begin + work / 9;
    const int a = work % 9;
    const CudaSfmObservation observation = observations[obsIndex];
    const int cameraBase = 9 * observation.cameraSlot;
    const double* E_i = cameraPointBlocks + 27 * obsIndex + 3 * a;
    double E_i_Cinv_row[3];
    for (int column = 0; column < 3; ++column) {
      E_i_Cinv_row[column] =
          E_i[0] * invPoint[column] + E_i[1] * invPoint[3 + column] +
          E_i[2] * invPoint[6 + column];
    }
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

    const double* E_i = cameraPointBlocks + 27 * obsI + 3 * a;
    const double* E_j_row = cameraPointBlocks + 27 * obsJ + 3 * b;
    double E_i_Cinv_row[3];
    for (int column = 0; column < 3; ++column) {
      E_i_Cinv_row[column] =
          E_i[0] * invPoint[column] + E_i[1] * invPoint[3 + column] +
          E_i[2] * invPoint[6 + column];
    }
    const double value = E_i_Cinv_row[0] * E_j_row[0] +
                         E_i_Cinv_row[1] * E_j_row[1] +
                         E_i_Cinv_row[2] * E_j_row[2];
    atomicAdd(&denseCameraSystem[DenseColumnMajorIndex(row, col, cameraDim)],
              -value);
  }
}

__global__ void InitializeDenseCameraSystemKernel(
    int numCameras, int cameraDim, const double* cameraNormalBlocks,
    const double* cameraGradient, double lambda,
    const double* dampingDiagonal, double* denseCameraSystem,
    double* cameraRhs) {
  const int camera = blockIdx.x * blockDim.x + threadIdx.x;
  if (camera >= numCameras) return;
  const int base = 9 * camera;
  const double* U = cameraNormalBlocks + 81 * camera;
  for (int row = 0; row < 9; ++row) {
    cameraRhs[base + row] = cameraGradient[base + row];
    for (int column = 0; column <= row; ++column) {
      denseCameraSystem[DenseColumnMajorIndex(base + row, base + column,
                                               cameraDim)] =
          U[9 * row + column];
    }
    denseCameraSystem[DenseColumnMajorIndex(base + row, base + row,
                                             cameraDim)] +=
        lambda *
        (dampingDiagonal ? dampingDiagonal[base + row] : 1.0);
  }
}

__global__ void RecoverPointDeltaKernel(
    const CudaSfmObservation* observations, const int* pointOffsets,
    const double* pointNormalBlocks, const double* pointGradient,
    const double* cameraPointBlocks, int numPoints, int cameraDim,
    double lambda, const double* dampingDiagonal, double* delta,
    int* singularPointBlocks) {
  const int pointSlot = blockIdx.x * blockDim.x + threadIdx.x;
  if (pointSlot >= numPoints) return;

  const int begin = pointOffsets[pointSlot];
  const int end = pointOffsets[pointSlot + 1];
  double pointBlock[9];
  double pointRhs[3];
  double invPoint[9];
  const int pointBase = cameraDim + 3 * pointSlot;
  LoadDampedPointNormal(pointNormalBlocks, pointGradient, pointSlot,
                        pointBase, lambda, dampingDiagonal, pointBlock,
                        pointRhs);
  if (!Invert3x3(pointBlock, invPoint)) {
    atomicAdd(singularPointBlocks, 1);
    return;
  }

  double reducedRhs[3] = {pointRhs[0], pointRhs[1], pointRhs[2]};
  for (int obsIndex = begin; obsIndex < end; ++obsIndex) {
    const CudaSfmObservation observation = observations[obsIndex];
    const double* cameraPoint = cameraPointBlocks + 27 * obsIndex;
    const int cameraBase = 9 * observation.cameraSlot;
    for (int p = 0; p < 3; ++p) {
      double correction = 0.0;
      for (int c = 0; c < 9; ++c) {
        correction += cameraPoint[3 * c + p] * delta[cameraBase + c];
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

struct CudaSfmSchurProblem::Impl {
  CudaDeviceArray<double> denseCameraSystem;
  CudaDeviceArray<double> cameraRhs;
  CudaSfmProjectionLinearization linearization;
  CudaSfmSchurBlocks blocks;
  CudaDeviceArray<int> singularPointBlocks;
  DeviceSparseSpdSystem sparseSystem;
  std::vector<int> sparseRowPointers;
  std::vector<int> sparseColumnIndices;
  std::unique_ptr<CudaSfmSchurOperator> implicitOperator;
  std::unique_ptr<CudaSfmCameraBlockPreconditioner> implicitPreconditioner;
  CudaDeviceArray<double> implicitRhs;
  const CudaSfmProjectionBatch* batch = nullptr;
  int numCameras = 0;
  int cameraDim = 0;
  int numPoints = 0;
  bool isLinearized = false;
  size_t linearizationCount = 0;
  size_t blockBuildCount = 0;
  size_t denseAssemblyCount = 0;

  void initialize(const CudaSfmProjectionBatch& newBatch,
                  int newNumCameras) {
    if (newNumCameras < 0) {
      throw std::invalid_argument("CudaSfmSchurProblem numCameras < 0");
    }
    if (newBatch.numCameras() > static_cast<size_t>(newNumCameras)) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem batch camera count exceeds numCameras");
    }
    if (newBatch.pointObservationOffsets().size() !=
        newBatch.numPoints() + 1) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem point offset size mismatch");
    }
    constexpr size_t kMaxInt =
        static_cast<size_t>(std::numeric_limits<int>::max());
    if (static_cast<size_t>(newNumCameras) > kMaxInt / 81 ||
        newBatch.numPoints() > kMaxInt / 9 ||
        newBatch.numObservations() > kMaxInt / 27 ||
        newBatch.numObservations() > kMaxInt - kDenseSchurBlockSize + 1 ||
        newBatch.numPoints() >
            static_cast<size_t>((std::numeric_limits<int>::max() -
                                 9 * newNumCameras) /
                                3)) {
      throw std::invalid_argument("CudaSfmSchurProblem dimension too large");
    }
    batch = &newBatch;
    numCameras = newNumCameras;
    cameraDim = 9 * numCameras;
    numPoints = static_cast<int>(newBatch.numPoints());
    isLinearized = false;
    implicitOperator.reset();
    implicitPreconditioner.reset();
  }

  int totalDim() const { return cameraDim + 3 * numPoints; }

  void linearizeValues(const DeviceValues& values, cudaStream_t stream) {
    if (!batch) {
      throw std::logic_error(
          "CudaSfmSchurProblem must be initialized before linearize");
    }
    const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
        kDevicePinholeCameraCal3BundlerType);
    const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);
    if (batch->numCameras() > cameraBlock.values.size() ||
        batch->numPoints() > pointBlock.values.size()) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem batch/value size mismatch");
    }
    LinearizeCudaSfmProjectionBatch(values, *batch, &linearization, stream);
    const size_t observations = batch->numObservations();
    blocks.cameraNormalBlocks.resize(81 * static_cast<size_t>(numCameras));
    blocks.cameraGradient.resize(9 * static_cast<size_t>(numCameras));
    blocks.pointNormalBlocks.resize(9 * static_cast<size_t>(numPoints));
    blocks.pointGradient.resize(3 * static_cast<size_t>(numPoints));
    blocks.cameraPointBlocks.resize(27 * observations);
    blocks.cameraNormalBlocks.zero(stream);
    blocks.cameraGradient.zero(stream);
    blocks.pointNormalBlocks.zero(stream);
    blocks.pointGradient.zero(stream);
    blocks.cameraPointBlocks.zero(stream);
    if (observations > 0) {
      if (observations >
          static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument(
            "CudaSfmSchurProblem observation count exceeds int range");
      }
      const int observationCount = static_cast<int>(observations);
      const int grid =
          (observationCount + kDenseSchurBlockSize - 1) /
          kDenseSchurBlockSize;
      BuildPersistentSchurBlocksKernel<<<grid, kDenseSchurBlockSize, 0,
                                         stream>>>(
          batch->observations().data(), linearization.residuals.data(),
          linearization.cameraJacobians.data(),
          linearization.pointJacobians.data(), observationCount,
          blocks.cameraNormalBlocks.data(), blocks.cameraGradient.data(),
          blocks.pointNormalBlocks.data(), blocks.pointGradient.data(),
          blocks.cameraPointBlocks.data());
      GTSAM_CUDA_CHECK(cudaGetLastError());
    }
    isLinearized = true;
    ++linearizationCount;
    ++blockBuildCount;
  }

  CudaDenseSpdSystemView prepareDense(
      double lambda, const CudaDeviceArray<double>* dampingDiagonal,
      cudaStream_t stream) {
    if (!isLinearized || !batch) {
      throw std::logic_error(
          "CudaSfmSchurProblem must be linearized before prepareDense");
    }
    if (!(lambda > 0.0) || !std::isfinite(lambda)) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem requires finite lambda > 0");
    }
    if (dampingDiagonal &&
        dampingDiagonal->size() != static_cast<size_t>(totalDim())) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem damping diagonal size mismatch");
    }
    if (cameraDim == 0) {
      ++denseAssemblyCount;
      return {0, 0, nullptr, nullptr};
    }
    if (static_cast<size_t>(cameraDim) >
        std::numeric_limits<size_t>::max() / static_cast<size_t>(cameraDim)) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem camera system too big");
    }

    denseCameraSystem.resize(static_cast<size_t>(cameraDim) *
                             static_cast<size_t>(cameraDim));
    cameraRhs.resize(static_cast<size_t>(cameraDim));
    singularPointBlocks.resize(1);
    denseCameraSystem.zero(stream);
    cameraRhs.zero(stream);
    singularPointBlocks.zero(stream);

    const int cameraBlockGrid =
        (numCameras + kDenseSchurBlockSize - 1) /
        kDenseSchurBlockSize;
    if (cameraBlockGrid > 0) {
      InitializeDenseCameraSystemKernel<<<cameraBlockGrid,
                                          kDenseSchurBlockSize, 0, stream>>>(
          numCameras, cameraDim, blocks.cameraNormalBlocks.data(),
          blocks.cameraGradient.data(), lambda,
          dampingDiagonal ? dampingDiagonal->data() : nullptr,
          denseCameraSystem.data(), cameraRhs.data());
      GTSAM_CUDA_CHECK(cudaGetLastError());
    }

    const int pointGrid =
        (numPoints + kDenseSchurBlockSize - 1) / kDenseSchurBlockSize;
    if (pointGrid > 0) {
      AccumulateDenseSchurKernel<<<pointGrid, kDenseSchurBlockSize, 0,
                                   stream>>>(
          batch->observations().data(),
          batch->pointObservationOffsets().data(),
          blocks.pointNormalBlocks.data(), blocks.pointGradient.data(),
          blocks.cameraPointBlocks.data(), numPoints, cameraDim, lambda,
          dampingDiagonal ? dampingDiagonal->data() : nullptr,
          denseCameraSystem.data(), cameraRhs.data(),
          singularPointBlocks.data());
      GTSAM_CUDA_CHECK(cudaGetLastError());
    }

    const int longTrackGrid =
        static_cast<int>(batch->longTrackPointSlots().size());
    if (longTrackGrid > 0) {
      AccumulateLongTrackDenseSchurKernel<<<longTrackGrid,
                                            kLongTrackSchurBlockSize, 0,
                                            stream>>>(
          batch->longTrackPointSlots().data(), longTrackGrid,
          batch->observations().data(),
          batch->pointObservationOffsets().data(),
          blocks.pointNormalBlocks.data(), blocks.pointGradient.data(),
          blocks.cameraPointBlocks.data(), cameraDim, lambda,
          dampingDiagonal ? dampingDiagonal->data() : nullptr,
          denseCameraSystem.data(), cameraRhs.data(),
          singularPointBlocks.data());
      GTSAM_CUDA_CHECK(cudaGetLastError());
    }

    ++denseAssemblyCount;
    return {cameraDim, cameraDim, denseCameraSystem.data(), cameraRhs.data()};
  }

  DeviceSparseSpdSystem& prepareSparse(
      double lambda, const CudaDeviceArray<double>* dampingDiagonal,
      const CudaSfmReducedCsrPlan& plan, cudaStream_t stream) {
    if (!isLinearized || !batch) {
      throw std::logic_error(
          "CudaSfmSchurProblem must be linearized before prepareSparse");
    }
    if (!(lambda > 0.0) || !std::isfinite(lambda)) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem requires finite lambda > 0");
    }
    if (plan.dimension() != cameraDim) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem sparse plan dimension mismatch");
    }
    if (dampingDiagonal &&
        dampingDiagonal->size() != static_cast<size_t>(totalDim())) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem damping diagonal size mismatch");
    }
    if (sparseRowPointers != plan.rowPointers() ||
        sparseColumnIndices != plan.columnIndices()) {
      sparseSystem.uploadPattern(plan.dimension(), plan.rowPointers(),
                                 plan.columnIndices(), stream);
      sparseRowPointers = plan.rowPointers();
      sparseColumnIndices = plan.columnIndices();
    }
    AssembleCudaSfmSparseSchur(*batch, blocks, numCameras, lambda,
                              dampingDiagonal, &sparseSystem,
                              &singularPointBlocks, stream);
    return sparseSystem;
  }

  CudaSfmImplicitSchurView prepareImplicit(
      double lambda, const CudaDeviceArray<double>* dampingDiagonal,
      cudaStream_t stream) {
    if (!isLinearized || !batch) {
      throw std::logic_error(
          "CudaSfmSchurProblem must be linearized before prepareImplicit");
    }
    if (!implicitOperator) {
      implicitOperator = std::make_unique<CudaSfmSchurOperator>(
          *batch, blocks, numCameras);
      implicitPreconditioner =
          std::make_unique<CudaSfmCameraBlockPreconditioner>(
              *batch, blocks, numCameras);
    }
    implicitOperator->configure(lambda, dampingDiagonal);
    implicitPreconditioner->build(lambda, dampingDiagonal, &implicitRhs,
                                  stream);
    return {implicitOperator.get(), implicitPreconditioner.get(),
            implicitRhs.data(), cameraDim};
  }

  void recoverPoints(double lambda,
                     const CudaDeviceArray<double>* dampingDiagonal,
                     const CudaDeviceArray<double>& cameraDelta,
                     CudaDeviceArray<double>* fullDelta,
                     cudaStream_t stream) {
    if (!fullDelta) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem requires full delta output");
    }
    if (!isLinearized || !batch) {
      throw std::logic_error(
          "CudaSfmSchurProblem must be linearized before point recovery");
    }
    if (!(lambda > 0.0) || !std::isfinite(lambda)) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem requires finite lambda > 0");
    }
    if (cameraDelta.size() != static_cast<size_t>(cameraDim)) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem camera delta size mismatch");
    }
    if (dampingDiagonal &&
        dampingDiagonal->size() != static_cast<size_t>(totalDim())) {
      throw std::invalid_argument(
          "CudaSfmSchurProblem damping diagonal size mismatch");
    }

    fullDelta->resize(static_cast<size_t>(totalDim()));
    fullDelta->zero(stream);
    if (cameraDim > 0) {
      GTSAM_CUDA_CHECK(cudaMemcpyAsync(
          fullDelta->data(), cameraDelta.data(), sizeof(double) * cameraDim,
          cudaMemcpyDeviceToDevice, stream));
    }

    singularPointBlocks.zero(stream);
    const int pointGrid =
        (numPoints + kDenseSchurBlockSize - 1) / kDenseSchurBlockSize;
    if (pointGrid > 0) {
      RecoverPointDeltaKernel<<<pointGrid, kDenseSchurBlockSize, 0, stream>>>(
          batch->observations().data(),
          batch->pointObservationOffsets().data(),
          blocks.pointNormalBlocks.data(), blocks.pointGradient.data(),
          blocks.cameraPointBlocks.data(), numPoints, cameraDim, lambda,
          dampingDiagonal ? dampingDiagonal->data() : nullptr,
          fullDelta->data(), singularPointBlocks.data());
      GTSAM_CUDA_CHECK(cudaGetLastError());
    }
  }
};

CudaSfmSchurProblem::CudaSfmSchurProblem()
    : impl_(std::make_unique<Impl>()) {}
CudaSfmSchurProblem::~CudaSfmSchurProblem() = default;
CudaSfmSchurProblem::CudaSfmSchurProblem(CudaSfmSchurProblem&&) noexcept =
    default;
CudaSfmSchurProblem& CudaSfmSchurProblem::operator=(
    CudaSfmSchurProblem&&) noexcept = default;

void CudaSfmSchurProblem::initialize(const CudaSfmProjectionBatch& batch,
                                     int numCameras) {
  impl_->initialize(batch, numCameras);
}

void CudaSfmSchurProblem::linearize(const DeviceValues& values,
                                    cudaStream_t stream) {
  impl_->linearizeValues(values, stream);
}

CudaDenseSpdSystemView CudaSfmSchurProblem::prepareDense(
    double lambda, cudaStream_t stream) {
  return impl_->prepareDense(lambda, nullptr, stream);
}

CudaDenseSpdSystemView CudaSfmSchurProblem::prepareDense(
    double lambda, const CudaDeviceArray<double>& dampingDiagonal,
    cudaStream_t stream) {
  return impl_->prepareDense(lambda, &dampingDiagonal, stream);
}

DeviceSparseSpdSystem& CudaSfmSchurProblem::prepareSparse(
    double lambda, const CudaSfmReducedCsrPlan& plan, cudaStream_t stream) {
  return impl_->prepareSparse(lambda, nullptr, plan, stream);
}

DeviceSparseSpdSystem& CudaSfmSchurProblem::prepareSparse(
    double lambda, const CudaDeviceArray<double>& dampingDiagonal,
    const CudaSfmReducedCsrPlan& plan, cudaStream_t stream) {
  return impl_->prepareSparse(lambda, &dampingDiagonal, plan, stream);
}

CudaSfmImplicitSchurView CudaSfmSchurProblem::prepareImplicit(
    double lambda, cudaStream_t stream) {
  return impl_->prepareImplicit(lambda, nullptr, stream);
}

CudaSfmImplicitSchurView CudaSfmSchurProblem::prepareImplicit(
    double lambda, const CudaDeviceArray<double>& dampingDiagonal,
    cudaStream_t stream) {
  return impl_->prepareImplicit(lambda, &dampingDiagonal, stream);
}

void CudaSfmSchurProblem::recoverPoints(
    double lambda, const CudaDeviceArray<double>& cameraDelta,
    CudaDeviceArray<double>* fullDelta, cudaStream_t stream) {
  impl_->recoverPoints(lambda, nullptr, cameraDelta, fullDelta, stream);
}

void CudaSfmSchurProblem::recoverPoints(
    double lambda, const CudaDeviceArray<double>& dampingDiagonal,
    const CudaDeviceArray<double>& cameraDelta,
    CudaDeviceArray<double>* fullDelta, cudaStream_t stream) {
  impl_->recoverPoints(lambda, &dampingDiagonal, cameraDelta, fullDelta,
                       stream);
}

int CudaSfmSchurProblem::cameraDimension() const { return impl_->cameraDim; }
int CudaSfmSchurProblem::totalDimension() const { return impl_->totalDim(); }
size_t CudaSfmSchurProblem::linearizationCount() const {
  return impl_->linearizationCount;
}
size_t CudaSfmSchurProblem::blockBuildCount() const {
  return impl_->blockBuildCount;
}
size_t CudaSfmSchurProblem::denseAssemblyCount() const {
  return impl_->denseAssemblyCount;
}
const CudaSfmProjectionLinearization&
CudaSfmSchurProblem::linearization() const {
  if (!impl_->isLinearized) {
    throw std::logic_error("CudaSfmSchurProblem is not linearized");
  }
  return impl_->linearization;
}
const CudaSfmSchurBlocks& CudaSfmSchurProblem::blocks() const {
  if (!impl_->isLinearized) {
    throw std::logic_error("CudaSfmSchurProblem is not linearized");
  }
  return impl_->blocks;
}
const CudaDeviceArray<double>& CudaSfmSchurProblem::cameraRhs() const {
  return impl_->cameraRhs;
}

struct CudaSfmDenseSchurSolver::Impl {
  CudaSfmSchurProblem problem;
  CudaLinearSolverSession denseSession{
      CudaLinearSolverOptions{CudaLinearSolverType::DenseCholesky, false}};
  CudaDeviceArray<double> cameraSolution;
  int analyzedDimension = -1;

  void linearize(const DeviceValues& values, const CudaSfmProjectionBatch& batch,
                 int numCameras, cudaStream_t stream) {
    problem.initialize(batch, numCameras);
    problem.linearize(values, stream);
  }

  void solveLinearized(double lambda,
                       const CudaDeviceArray<double>* dampingDiagonal,
                       CudaDeviceArray<double>* delta, cudaStream_t stream) {
    CudaDenseSpdSystemView dense =
        dampingDiagonal ? problem.prepareDense(lambda, *dampingDiagonal, stream)
                        : problem.prepareDense(lambda, stream);
    if (dense.dimension == 0) {
      delta->resize(static_cast<size_t>(problem.totalDimension()));
      delta->zero(stream);
      return;
    }
    if (analyzedDimension != dense.dimension) {
      denseSession.analyze(dense.dimension, stream);
      analyzedDimension = dense.dimension;
    }
    denseSession.solve(dense, &cameraSolution, stream);
    if (dampingDiagonal) {
      problem.recoverPoints(lambda, *dampingDiagonal, cameraSolution,
                            delta, stream);
    } else {
      problem.recoverPoints(lambda, cameraSolution, delta, stream);
    }
  }

  void solve(const DeviceValues& values, const CudaSfmProjectionBatch& batch,
             int numCameras, double lambda,
             const CudaDeviceArray<double>* dampingDiagonal,
             CudaDeviceArray<double>* delta, cudaStream_t stream) {
    CheckSchurInputs(values, batch, numCameras, lambda, dampingDiagonal, delta);
    linearize(values, batch, numCameras, stream);
    solveLinearized(lambda, dampingDiagonal, delta, stream);
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
  impl_->solve(values, batch, numCameras, lambda, nullptr, delta, stream);
}

void CudaSfmDenseSchurSolver::linearize(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, cudaStream_t stream) {
  impl_->linearize(values, batch, numCameras, stream);
}

void CudaSfmDenseSchurSolver::solveLinearized(
    double lambda, CudaDeviceArray<double>* delta, cudaStream_t stream) {
  impl_->solveLinearized(lambda, nullptr, delta, stream);
}

void CudaSfmDenseSchurSolver::solveLinearized(
    double lambda, const CudaDeviceArray<double>& dampingDiagonal,
    CudaDeviceArray<double>* delta, cudaStream_t stream) {
  impl_->solveLinearized(lambda, &dampingDiagonal, delta, stream);
}

size_t CudaSfmDenseSchurSolver::linearizationCount() const {
  return impl_->problem.linearizationCount();
}

size_t CudaSfmDenseSchurSolver::denseAssemblyCount() const {
  return impl_->problem.denseAssemblyCount();
}
const CudaSfmProjectionLinearization&
CudaSfmDenseSchurSolver::linearization() const {
  return impl_->problem.linearization();
}

const CudaLinearSolveStats& CudaSfmDenseSchurSolver::linearSolveStats() const {
  return impl_->denseSession.stats();
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
