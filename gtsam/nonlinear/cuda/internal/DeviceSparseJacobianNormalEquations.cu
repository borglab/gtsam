/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DeviceSparseJacobianNormalEquations.cu
 * @brief   Device pipeline from a packed sparse Jacobian to normal equations
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#include <cusparse.h>
#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/base/cuda/PinnedHostArray.h>
#include <gtsam/nonlinear/cuda/internal/PcgOperatorBuilder.h>
#include <gtsam/nonlinear/cuda/internal/DeviceSparseJacobianNormalEquations.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cub/device/device_reduce.cuh>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#ifndef GTSAM_CUSPARSE_HAS_SPGEMM_REUSE
#if defined(CUSPARSE_VERSION) && CUSPARSE_VERSION >= 11600
#define GTSAM_CUSPARSE_HAS_SPGEMM_REUSE 1
#else
#define GTSAM_CUSPARSE_HAS_SPGEMM_REUSE 0
#endif
#endif

namespace gtsam::cuda {
namespace {

constexpr cusparseOperation_t kNoTranspose = CUSPARSE_OPERATION_NON_TRANSPOSE;
constexpr cusparseSpGEMMAlg_t kSpGemmAlgorithm = CUSPARSE_SPGEMM_DEFAULT;
constexpr cusparseSpMVAlg_t kSpMvAlgorithm = CUSPARSE_SPMV_ALG_DEFAULT;
constexpr cusparseCsr2CscAlg_t kCsr2CscAlgorithm = CUSPARSE_CSR2CSC_ALG_DEFAULT;
constexpr double kAlpha = 1.0;
constexpr double kBeta = 0.0;
constexpr int kKernelBlockSize = 256;

void checkRuntimeStatus(cudaError_t status, const char* stage) {
  if (status == cudaSuccess) return;
  std::ostringstream message;
  message << "CUDA failure during " << stage << ": " << cudaGetErrorName(status)
          << " (" << cudaGetErrorString(status) << ")";
  throw std::runtime_error(message.str());
}

void checkCusparse(cusparseStatus_t status, const char* stage) {
  if (status == CUSPARSE_STATUS_SUCCESS) return;
  const char* name = cusparseGetErrorName(status);
  const char* detail = cusparseGetErrorString(status);
  std::ostringstream message;
  message << "cuSPARSE failure during " << stage << ": "
          << (name ? name : "unknown status") << " ("
          << (detail ? detail : "no detail") << ")";
  throw std::runtime_error(message.str());
}

void validatePositivePlan(const SparseJacobianPlan& plan) {
  if (plan.rows() <= 0 || plan.columns() <= 0 || plan.nonzeros() <= 0) {
    throw std::invalid_argument(
        "DeviceSparseJacobianNormalEquations positive rows, columns, and "
        "nonzeros are required");
  }
}

void validateDiscoveredPattern(int rows, int expectedNonzeros,
                               const std::vector<int>& rowPointers,
                               const std::vector<int>& columnIndices,
                               std::vector<int>* diagonalOffsets) {
  if (rowPointers.size() != static_cast<size_t>(rows) + 1 ||
      columnIndices.size() != static_cast<size_t>(expectedNonzeros) ||
      rowPointers.front() != 0 || rowPointers.back() != expectedNonzeros) {
    throw std::runtime_error(
        "cuSPARSE SpGEMM discovery returned an invalid CSR size");
  }

  diagonalOffsets->assign(static_cast<size_t>(rows), -1);
  for (int row = 0; row < rows; ++row) {
    const int begin = rowPointers[static_cast<size_t>(row)];
    const int end = rowPointers[static_cast<size_t>(row + 1)];
    if (begin < 0 || end < begin || end > expectedNonzeros) {
      throw std::runtime_error(
          "cuSPARSE SpGEMM discovery returned invalid row offsets");
    }

    int previousColumn = -1;
    for (int index = begin; index < end; ++index) {
      const int column = columnIndices[static_cast<size_t>(index)];
      if (column < 0 || column >= rows || column <= previousColumn) {
        throw std::runtime_error(
            "cuSPARSE SpGEMM discovery returned an unsorted or invalid CSR "
            "column");
      }
      previousColumn = column;
      if (column == row) {
        (*diagonalOffsets)[static_cast<size_t>(row)] = index;
      }
    }
    if ((*diagonalOffsets)[static_cast<size_t>(row)] < 0) {
      throw std::runtime_error(
          "cuSPARSE SpGEMM discovery omitted a scalar diagonal entry");
    }
  }
}

__global__ void gatherDiagonalKernel(const double* values,
                                     const int* diagonalOffsets, int rows,
                                     double* diagonal) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row < rows) {
    diagonal[row] = values[diagonalOffsets[row]];
  }
}

// PCG mode: diag(JᵀJ) is the vector of squared Jᵀ row norms, computed
// without forming H.
__global__ void columnSquaredNormsKernel(const int* jtRowPointers,
                                         const double* jtValues, int columns,
                                         double* diagonal) {
  const int column = blockIdx.x * blockDim.x + threadIdx.x;
  if (column >= columns) return;
  double sum = 0.0;
  for (int index = jtRowPointers[column]; index < jtRowPointers[column + 1];
       ++index) {
    const double value = jtValues[index];
    sum += value * value;
  }
  diagonal[column] = sum;
}

__global__ void prepareDampingKernel(const double* undampedDiagonal, int rows,
                                     bool diagonalDamping, double minDiagonal,
                                     double maxDiagonal,
                                     double* dampingDiagonal) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row < rows) {
    dampingDiagonal[row] =
        diagonalDamping
            ? fmin(maxDiagonal, fmax(minDiagonal, undampedDiagonal[row]))
            : 1.0;
  }
}

__global__ void applyDampingKernel(double* values, const int* diagonalOffsets,
                                   const double* undampedDiagonal,
                                   const double* dampingDiagonal, int rows,
                                   double lambda) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row < rows) {
    values[diagonalOffsets[row]] =
        undampedDiagonal[row] + lambda * dampingDiagonal[row];
  }
}

__global__ void oldErrorTermsKernel(const double* rhs, int rows,
                                    double* terms) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row < rows) {
    terms[row] = 0.5 * rhs[row] * rhs[row];
  }
}

__global__ void newErrorTermsKernel(const double* jDelta, const double* rhs,
                                    int rows, double* terms) {
  const int row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row < rows) {
    const double residual = jDelta[row] - rhs[row];
    terms[row] = 0.5 * residual * residual;
  }
}

int kernelGridSize(int count) {
  return count / kKernelBlockSize + (count % kKernelBlockSize != 0);
}

struct DiscoveredPattern {
  std::vector<int> rowPointers;
  std::vector<int> columnIndices;
  std::vector<int> diagonalOffsets;
};

struct DiscoveryResources {
  cusparseSpMatDescr_t h = nullptr;
  cusparseSpGEMMDescr_t reuse = nullptr;
  DeviceArray<int> rowPointers;
  DeviceArray<int> columnIndices;
  DeviceArray<double> values;
  DeviceArray<unsigned char> buffer1;
  DeviceArray<unsigned char> buffer2;
  DeviceArray<unsigned char> buffer3;
  DeviceArray<unsigned char> buffer4;
  DeviceArray<unsigned char> buffer5;

  ~DiscoveryResources() {
    if (reuse) cusparseSpGEMM_destroyDescr(reuse);
    if (h) cusparseDestroySpMat(h);
  }
};

}  // namespace

struct DeviceSparseJacobianNormalEquations::Impl {
  static constexpr size_t kNoEventSpan = std::numeric_limits<size_t>::max();

  struct PendingEventSpan {
    cudaEvent_t begin = nullptr;
    cudaEvent_t end = nullptr;
    double* accumulator = nullptr;
    bool ended = false;
  };

  cudaStream_t stream = nullptr;
  bool collectProfile = false;
  DeviceSparseJacobianProfile profile;
  std::vector<PendingEventSpan> pendingEventSpans;
  int jacobianRows = 0;
  int jacobianColumns = 0;
  int jacobianNonzeros = 0;
  uint64_t jacobianStructuralFingerprint = 0;

  cusparseHandle_t handle = nullptr;
  cusparseSpMatDescr_t jDescriptor = nullptr;
  cusparseSpMatDescr_t jtDescriptor = nullptr;
  cusparseSpMatDescr_t hDescriptor = nullptr;
  cusparseDnVecDescr_t bDescriptor = nullptr;
  cusparseDnVecDescr_t gDescriptor = nullptr;
  cusparseDnVecDescr_t deltaDescriptor = nullptr;
  cusparseDnVecDescr_t jDeltaDescriptor = nullptr;
  cusparseSpGEMMDescr_t reuseDescriptor = nullptr;

  DeviceNormalSolverBackend backend = DeviceNormalSolverBackend::Cudss;
  DeviceNormalSolverOptions solverOptions;

  DeviceArray<int> jRowPointers;
  DeviceArray<int> jColumnIndices;
  DeviceArray<double> jValues;
  DeviceArray<double> b;
  DeviceArray<int> jtRowPointers;
  DeviceArray<int> jtColumnIndices;
  DeviceArray<double> jtValues;
  DeviceSparseSpdSystem normalEquations;
  DeviceArray<int> hDiagonalOffsets;
  DeviceArray<double> undampedDiagonal;
  DeviceArray<double> dampingDiagonal;
  DeviceArray<double> delta;
  DeviceArray<double> jDelta;
  DeviceArray<double> squaredTerms;
  DeviceArray<double> oldError;
  DeviceArray<double> newError;
  std::optional<PcgOperatorBuilder> pcgSolver;
  // PCG mode stores g = Jᵀb here; cuDSS mode keeps it in
  // normalEquations.rhs().
  DeviceArray<double> gradient;

  DeviceArray<unsigned char> csr2cscBuffer;
  DeviceArray<unsigned char> spmvBuffer;
  DeviceArray<unsigned char> cubReductionBuffer;
  DeviceArray<unsigned char> reuseBuffer1;
  DeviceArray<unsigned char> reuseBuffer2;
  DeviceArray<unsigned char> reuseBuffer3;
  DeviceArray<unsigned char> reuseBuffer4;
  DeviceArray<unsigned char> reuseBuffer5;
  PinnedHostArray<double> downloadedDelta;
  PinnedHostArray<double> downloadedErrors;

  bool numericsUploaded = false;
  bool formed = false;
  bool dampingPrepared = false;
  bool attemptReady = false;

  struct PersistentPointers {
    const int* jRowPointers = nullptr;
    const int* jColumnIndices = nullptr;
    const double* jValues = nullptr;
    const double* b = nullptr;
    const int* jtRowPointers = nullptr;
    const int* jtColumnIndices = nullptr;
    const double* jtValues = nullptr;
    const int* hRowPointers = nullptr;
    const int* hColumnIndices = nullptr;
    const double* hValues = nullptr;
    const double* g = nullptr;
    const int* hDiagonalOffsets = nullptr;
    const double* undampedDiagonal = nullptr;
    const double* dampingDiagonal = nullptr;
    const double* delta = nullptr;
    const double* jDelta = nullptr;
    const double* squaredTerms = nullptr;
    const double* oldError = nullptr;
    const double* newError = nullptr;
    const unsigned char* csr2cscBuffer = nullptr;
    const unsigned char* spmvBuffer = nullptr;
    const unsigned char* cubReductionBuffer = nullptr;
    const unsigned char* reuseBuffer4 = nullptr;
    const unsigned char* reuseBuffer5 = nullptr;
    const double* downloadedDelta = nullptr;
    const double* downloadedErrors = nullptr;
  } pointers;

  Impl(cudaStream_t fixedStream, bool shouldCollectProfile)
      : stream(fixedStream), collectProfile(shouldCollectProfile) {}

  ~Impl() {
    // cuSPARSE work is asynchronous. Keep every descriptor, workspace, and
    // backing allocation alive until the fixed stream has stopped using it.
    cudaStreamSynchronize(stream);
    destroyPendingEventsNoThrow();
    if (reuseDescriptor) cusparseSpGEMM_destroyDescr(reuseDescriptor);
    if (jDeltaDescriptor) cusparseDestroyDnVec(jDeltaDescriptor);
    if (deltaDescriptor) cusparseDestroyDnVec(deltaDescriptor);
    if (gDescriptor) cusparseDestroyDnVec(gDescriptor);
    if (bDescriptor) cusparseDestroyDnVec(bDescriptor);
    if (hDescriptor) cusparseDestroySpMat(hDescriptor);
    if (jtDescriptor) cusparseDestroySpMat(jtDescriptor);
    if (jDescriptor) cusparseDestroySpMat(jDescriptor);
    if (handle) cusparseDestroy(handle);
  }

  void synchronizeNoThrow() const noexcept { cudaStreamSynchronize(stream); }

  void destroyPendingEventsNoThrow() noexcept {
    for (PendingEventSpan& span : pendingEventSpans) {
      if (span.end) cudaEventDestroy(span.end);
      if (span.begin) cudaEventDestroy(span.begin);
    }
    pendingEventSpans.clear();
  }

  size_t createUnrecordedEventSpan(double* accumulator) {
    if (!collectProfile) return kNoEventSpan;

    PendingEventSpan span;
    span.accumulator = accumulator;
    checkRuntimeStatus(cudaEventCreate(&span.begin), "profile start-event creation");
    const cudaError_t endStatus = cudaEventCreate(&span.end);
    if (endStatus != cudaSuccess) {
      cudaEventDestroy(span.begin);
      checkRuntimeStatus(endStatus, "profile end-event creation");
    }
    try {
      pendingEventSpans.push_back(span);
    } catch (...) {
      cudaEventDestroy(span.end);
      cudaEventDestroy(span.begin);
      throw;
    }
    return pendingEventSpans.size() - 1;
  }

  size_t beginEventSpan(double* accumulator) {
    const size_t index = createUnrecordedEventSpan(accumulator);
    if (index == kNoEventSpan) return index;
    const cudaError_t recordStatus =
        cudaEventRecord(pendingEventSpans[index].begin, stream);
    if (recordStatus != cudaSuccess) {
      cudaEventDestroy(pendingEventSpans[index].end);
      cudaEventDestroy(pendingEventSpans[index].begin);
      pendingEventSpans.pop_back();
      checkRuntimeStatus(recordStatus, "profile start-event record");
    }
    return index;
  }

  void markEventSpanEnded(size_t index) {
    if (index == kNoEventSpan) return;
    if (index >= pendingEventSpans.size() || pendingEventSpans[index].ended) {
      throw std::logic_error(
          "DeviceSparseJacobianNormalEquations invalid profile event span");
    }
    pendingEventSpans[index].ended = true;
  }

  void endEventSpan(size_t index) {
    if (index == kNoEventSpan) return;
    if (index >= pendingEventSpans.size() || pendingEventSpans[index].ended) {
      throw std::logic_error(
          "DeviceSparseJacobianNormalEquations invalid profile event span");
    }
    checkRuntimeStatus(cudaEventRecord(pendingEventSpans[index].end, stream),
              "profile end-event record");
    pendingEventSpans[index].ended = true;
  }

  void harvestPendingEvents() {
    cudaError_t firstFailure = cudaSuccess;
    const char* failureStage = nullptr;
    for (PendingEventSpan& span : pendingEventSpans) {
      if (span.ended && firstFailure == cudaSuccess) {
        float milliseconds = 0.0f;
        const cudaError_t elapsedStatus =
            cudaEventElapsedTime(&milliseconds, span.begin, span.end);
        if (elapsedStatus == cudaSuccess) {
          *span.accumulator += static_cast<double>(milliseconds) * 1e-3;
        } else {
          firstFailure = elapsedStatus;
          failureStage = "profile event elapsed-time query";
        }
      }
      const cudaError_t endDestroyStatus = cudaEventDestroy(span.end);
      if (firstFailure == cudaSuccess && endDestroyStatus != cudaSuccess) {
        firstFailure = endDestroyStatus;
        failureStage = "profile end-event destruction";
      }
      const cudaError_t beginDestroyStatus = cudaEventDestroy(span.begin);
      if (firstFailure == cudaSuccess && beginDestroyStatus != cudaSuccess) {
        firstFailure = beginDestroyStatus;
        failureStage = "profile start-event destruction";
      }
      span.begin = nullptr;
      span.end = nullptr;
    }
    pendingEventSpans.clear();
    if (firstFailure != cudaSuccess) {
      checkRuntimeStatus(firstFailure, failureStage);
    }
  }

  void validateStream(cudaStream_t supplied) const {
    if (supplied != stream) {
      throw std::invalid_argument(
          "DeviceSparseJacobianNormalEquations stream differs from the "
          "initialization stream");
    }
  }

  void capturePointers() {
    pointers.jRowPointers = jRowPointers.data();
    pointers.jColumnIndices = jColumnIndices.data();
    pointers.jValues = jValues.data();
    pointers.b = b.data();
    pointers.jtRowPointers = jtRowPointers.data();
    pointers.jtColumnIndices = jtColumnIndices.data();
    pointers.jtValues = jtValues.data();
    pointers.hRowPointers = normalEquations.rowPointers().data();
    pointers.hColumnIndices = normalEquations.colIndices().data();
    pointers.hValues = normalEquations.values().data();
    pointers.g = backend == DeviceNormalSolverBackend::Cudss
                     ? normalEquations.rhs().data()
                     : gradient.data();
    pointers.hDiagonalOffsets = hDiagonalOffsets.data();
    pointers.undampedDiagonal = undampedDiagonal.data();
    pointers.dampingDiagonal = dampingDiagonal.data();
    pointers.delta = delta.data();
    pointers.jDelta = jDelta.data();
    pointers.squaredTerms = squaredTerms.data();
    pointers.oldError = oldError.data();
    pointers.newError = newError.data();
    pointers.csr2cscBuffer = csr2cscBuffer.data();
    pointers.spmvBuffer = spmvBuffer.data();
    pointers.cubReductionBuffer = cubReductionBuffer.data();
    pointers.reuseBuffer4 = reuseBuffer4.data();
    pointers.reuseBuffer5 = reuseBuffer5.data();
    pointers.downloadedDelta = downloadedDelta.data();
    pointers.downloadedErrors = downloadedErrors.data();
  }

  void validatePointers() const {
    const bool cudssMode = backend == DeviceNormalSolverBackend::Cudss;
    const bool sharedSizesMatch =
        jRowPointers.size() == static_cast<size_t>(jacobianRows) + 1 &&
        jColumnIndices.size() == static_cast<size_t>(jacobianNonzeros) &&
        jValues.size() == static_cast<size_t>(jacobianNonzeros) &&
        b.size() == static_cast<size_t>(jacobianRows) &&
        jtRowPointers.size() == static_cast<size_t>(jacobianColumns) + 1 &&
        jtColumnIndices.size() == static_cast<size_t>(jacobianNonzeros) &&
        jtValues.size() == static_cast<size_t>(jacobianNonzeros) &&
        undampedDiagonal.size() == static_cast<size_t>(jacobianColumns) &&
        dampingDiagonal.size() == static_cast<size_t>(jacobianColumns) &&
        delta.size() == static_cast<size_t>(jacobianColumns) &&
        jDelta.size() == static_cast<size_t>(jacobianRows) &&
        squaredTerms.size() == static_cast<size_t>(jacobianRows) &&
        oldError.size() == 1 && newError.size() == 1 &&
        downloadedDelta.size() == static_cast<size_t>(jacobianColumns) &&
        downloadedErrors.size() == 2;
    const bool backendSizesMatch =
        cudssMode
            ? normalEquations.rows() == jacobianColumns &&
                  hDiagonalOffsets.size() ==
                      static_cast<size_t>(jacobianColumns)
            : gradient.size() == static_cast<size_t>(jacobianColumns);
    const bool sharedPointersMatch =
        pointers.jRowPointers == jRowPointers.data() &&
        pointers.jColumnIndices == jColumnIndices.data() &&
        pointers.jValues == jValues.data() && pointers.b == b.data() &&
        pointers.jtRowPointers == jtRowPointers.data() &&
        pointers.jtColumnIndices == jtColumnIndices.data() &&
        pointers.jtValues == jtValues.data() &&
        pointers.undampedDiagonal == undampedDiagonal.data() &&
        pointers.dampingDiagonal == dampingDiagonal.data() &&
        pointers.delta == delta.data() && pointers.jDelta == jDelta.data() &&
        pointers.squaredTerms == squaredTerms.data() &&
        pointers.oldError == oldError.data() &&
        pointers.newError == newError.data() &&
        pointers.csr2cscBuffer == csr2cscBuffer.data() &&
        pointers.spmvBuffer == spmvBuffer.data() &&
        pointers.cubReductionBuffer == cubReductionBuffer.data() &&
        pointers.downloadedDelta == downloadedDelta.data() &&
        pointers.downloadedErrors == downloadedErrors.data();
    const bool backendPointersMatch =
        cudssMode
            ? pointers.hRowPointers == normalEquations.rowPointers().data() &&
                  pointers.hColumnIndices ==
                      normalEquations.colIndices().data() &&
                  pointers.hValues == normalEquations.values().data() &&
                  pointers.g == normalEquations.rhs().data() &&
                  pointers.hDiagonalOffsets == hDiagonalOffsets.data() &&
                  pointers.reuseBuffer4 == reuseBuffer4.data() &&
                  pointers.reuseBuffer5 == reuseBuffer5.data()
            : pointers.g == gradient.data();
    const bool sharedDescriptorsValid = handle && jDescriptor &&
                                        jtDescriptor && bDescriptor &&
                                        gDescriptor && deltaDescriptor &&
                                        jDeltaDescriptor;
    const bool backendDescriptorsValid =
        cudssMode ? hDescriptor && reuseDescriptor : pcgSolver.has_value();
    if (!sharedSizesMatch || !backendSizesMatch || !sharedPointersMatch ||
        !backendPointersMatch || !sharedDescriptorsValid ||
        !backendDescriptorsValid) {
      throw std::runtime_error(
          "DeviceSparseJacobianNormalEquations persistent storage changed");
    }
  }

  void createJacobianStorage(const SparseJacobianPlan& plan) {
    jacobianRows = plan.rows();
    jacobianColumns = plan.columns();
    jacobianNonzeros = plan.nonzeros();
    jacobianStructuralFingerprint = plan.structuralFingerprint();

    jRowPointers.resize(plan.rowPointers().size());
    jColumnIndices.resize(plan.columnIndices().size());
    const size_t patternSpan = beginEventSpan(&profile.patternH2d);
    checkRuntimeStatus(cudaMemcpyAsync(jRowPointers.data(), plan.rowPointers().data(),
                              sizeof(int) * plan.rowPointers().size(),
                              cudaMemcpyHostToDevice, stream),
              "J row-pointer upload");
    checkRuntimeStatus(
        cudaMemcpyAsync(jColumnIndices.data(), plan.columnIndices().data(),
                        sizeof(int) * plan.columnIndices().size(),
                        cudaMemcpyHostToDevice, stream),
        "J column-index upload");
    profile.patternH2dBytes +=
        sizeof(int) * (plan.rowPointers().size() + plan.columnIndices().size());
    endEventSpan(patternSpan);

    const size_t structureSpan = beginEventSpan(&profile.structureSetup);
    jValues.resize(static_cast<size_t>(jacobianNonzeros));
    jValues.zero(stream);
    b.resize(static_cast<size_t>(jacobianRows));
    b.zero(stream);
    jtRowPointers.resize(static_cast<size_t>(jacobianColumns) + 1);
    jtColumnIndices.resize(static_cast<size_t>(jacobianNonzeros));
    jtValues.resize(static_cast<size_t>(jacobianNonzeros));

    size_t csr2cscBytes = 0;
    checkCusparse(
        cusparseCsr2cscEx2_bufferSize(
            handle, jacobianRows, jacobianColumns, jacobianNonzeros,
            jValues.data(), jRowPointers.data(), jColumnIndices.data(),
            jtValues.data(), jtRowPointers.data(), jtColumnIndices.data(),
            CUDA_R_64F, CUSPARSE_ACTION_NUMERIC, CUSPARSE_INDEX_BASE_ZERO,
            kCsr2CscAlgorithm, &csr2cscBytes),
        "CSR-to-CSC workspace query");
    csr2cscBuffer.resize(csr2cscBytes);
    transposeJacobian();

    checkCusparse(cusparseCreateCsr(&jDescriptor, jacobianRows, jacobianColumns,
                                    jacobianNonzeros, jRowPointers.data(),
                                    jColumnIndices.data(), jValues.data(),
                                    CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                                    CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F),
                  "J descriptor creation");
    checkCusparse(
        cusparseCreateCsr(&jtDescriptor, jacobianColumns, jacobianRows,
                          jacobianNonzeros, jtRowPointers.data(),
                          jtColumnIndices.data(), jtValues.data(),
                          CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                          CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F),
        "JT descriptor creation");
    endEventSpan(structureSpan);
  }

  void transposeJacobian() {
    checkCusparse(
        cusparseCsr2cscEx2(
            handle, jacobianRows, jacobianColumns, jacobianNonzeros,
            jValues.data(), jRowPointers.data(), jColumnIndices.data(),
            jtValues.data(), jtRowPointers.data(), jtColumnIndices.data(),
            CUDA_R_64F, CUSPARSE_ACTION_NUMERIC, CUSPARSE_INDEX_BASE_ZERO,
            kCsr2CscAlgorithm, csr2cscBuffer.data()),
        "numeric CSR-to-CSC transpose");
  }

  DiscoveredPattern discoverNormalPattern();
  void createStableNormalStorage(const DiscoveredPattern& discovered);
  void createSpmvResources();
  void setup(const SparseJacobianPlan& plan);
  void setupPcgStorage();
  void upload(const HostSparseJacobian& host, cudaStream_t suppliedStream);
  void form(cudaStream_t suppliedStream);
  void prepare(bool diagonalDamping, double minDiagonal, double maxDiagonal,
               cudaStream_t suppliedStream);
  void applyExplicitDamping(double lambda, cudaStream_t suppliedStream);
  void evaluateSolvedDelta(cudaStream_t suppliedStream);
  DeviceSparseJacobianAttemptResult downloadAttempt(
      cudaStream_t suppliedStream);
};

DiscoveredPattern
DeviceSparseJacobianNormalEquations::Impl::discoverNormalPattern() {
#if GTSAM_CUSPARSE_HAS_SPGEMM_REUSE
  DiscoveryResources discovery;
  DiscoveredPattern pattern;
  try {
    const size_t structureSpan = beginEventSpan(&profile.structureSetup);
    discovery.rowPointers.resize(static_cast<size_t>(jacobianColumns) + 1);
    discovery.rowPointers.zero(stream);
    checkCusparse(
        cusparseCreateCsr(&discovery.h, jacobianColumns, jacobianColumns, 0,
                          discovery.rowPointers.data(), nullptr, nullptr,
                          CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                          CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F),
        "discovery H descriptor creation");
    checkCusparse(cusparseSpGEMM_createDescr(&discovery.reuse),
                  "discovery reuse descriptor creation");

    size_t bufferSize1 = 0;
    checkCusparse(cusparseSpGEMMreuse_workEstimation(
                      handle, kNoTranspose, kNoTranspose, jtDescriptor,
                      jDescriptor, discovery.h, kSpGemmAlgorithm,
                      discovery.reuse, &bufferSize1, nullptr),
                  "discovery reuse work-estimation query");
    discovery.buffer1.resize(bufferSize1);
    checkCusparse(cusparseSpGEMMreuse_workEstimation(
                      handle, kNoTranspose, kNoTranspose, jtDescriptor,
                      jDescriptor, discovery.h, kSpGemmAlgorithm,
                      discovery.reuse, &bufferSize1, discovery.buffer1.data()),
                  "discovery reuse work-estimation execute");

    size_t bufferSize2 = 0;
    size_t bufferSize3 = 0;
    size_t bufferSize4 = 0;
    checkCusparse(
        cusparseSpGEMMreuse_nnz(
            handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
            discovery.h, kSpGemmAlgorithm, discovery.reuse, &bufferSize2,
            nullptr, &bufferSize3, nullptr, &bufferSize4, nullptr),
        "discovery reuse-nnz workspace query");
    discovery.buffer2.resize(bufferSize2);
    discovery.buffer3.resize(bufferSize3);
    discovery.buffer4.resize(bufferSize4);
    checkCusparse(
        cusparseSpGEMMreuse_nnz(
            handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
            discovery.h, kSpGemmAlgorithm, discovery.reuse, &bufferSize2,
            discovery.buffer2.data(), &bufferSize3, discovery.buffer3.data(),
            &bufferSize4, discovery.buffer4.data()),
        "discovery reuse-nnz execute");

    int64_t discoveredRows = 0;
    int64_t discoveredColumns = 0;
    int64_t discoveredNonzeros = 0;
    checkCusparse(cusparseSpMatGetSize(discovery.h, &discoveredRows,
                                       &discoveredColumns, &discoveredNonzeros),
                  "discovery H size query");
    if (discoveredRows != jacobianColumns ||
        discoveredColumns != jacobianColumns) {
      throw std::runtime_error(
          "cuSPARSE SpGEMM discovery returned a nonsquare H");
    }
    if (discoveredNonzeros <= 0 || discoveredNonzeros > INT_MAX) {
      throw std::runtime_error(
          "cuSPARSE SpGEMM discovery returned an invalid H nonzero count");
    }

    const int hNonzeros = static_cast<int>(discoveredNonzeros);
    discovery.columnIndices.resize(static_cast<size_t>(hNonzeros));
    discovery.values.resize(static_cast<size_t>(hNonzeros));
    discovery.values.zero(stream);
    checkCusparse(cusparseCsrSetPointers(
                      discovery.h, discovery.rowPointers.data(),
                      discovery.columnIndices.data(), discovery.values.data()),
                  "discovery H pointer binding before reuse-copy");

    size_t bufferSize5 = 0;
    checkCusparse(cusparseSpGEMMreuse_copy(
                      handle, kNoTranspose, kNoTranspose, jtDescriptor,
                      jDescriptor, discovery.h, kSpGemmAlgorithm,
                      discovery.reuse, &bufferSize5, nullptr),
                  "discovery reuse-copy workspace query");
    discovery.buffer5.resize(bufferSize5);
    checkCusparse(cusparseSpGEMMreuse_copy(
                      handle, kNoTranspose, kNoTranspose, jtDescriptor,
                      jDescriptor, discovery.h, kSpGemmAlgorithm,
                      discovery.reuse, &bufferSize5, discovery.buffer5.data()),
                  "discovery reuse-copy execute");
    endEventSpan(structureSpan);

    pattern.rowPointers.resize(static_cast<size_t>(jacobianColumns) + 1);
    pattern.columnIndices.resize(static_cast<size_t>(hNonzeros));
    const size_t setupDownloadSpan = beginEventSpan(&profile.setupD2h);
    checkRuntimeStatus(cudaMemcpyAsync(pattern.rowPointers.data(),
                              discovery.rowPointers.data(),
                              sizeof(int) * pattern.rowPointers.size(),
                              cudaMemcpyDeviceToHost, stream),
              "discovery H row-offset download");
    checkRuntimeStatus(cudaMemcpyAsync(pattern.columnIndices.data(),
                              discovery.columnIndices.data(),
                              sizeof(int) * pattern.columnIndices.size(),
                              cudaMemcpyDeviceToHost, stream),
              "discovery H column-index download");
    profile.setupD2hBytes += sizeof(int) * (pattern.rowPointers.size() +
                                            pattern.columnIndices.size());
    endEventSpan(setupDownloadSpan);
    checkRuntimeStatus(cudaStreamSynchronize(stream),
              "discovery H pattern download synchronization");
    validateDiscoveredPattern(jacobianColumns, hNonzeros, pattern.rowPointers,
                              pattern.columnIndices, &pattern.diagonalOffsets);
    return pattern;
  } catch (...) {
    // Locals are declared outside the try and remain alive for this sync.
    synchronizeNoThrow();
    throw;
  }
#else
  throw std::runtime_error("cuSPARSE SpGEMMreuse support was not configured");
#endif
}

void DeviceSparseJacobianNormalEquations::Impl::createStableNormalStorage(
    const DiscoveredPattern& discovered) {
#if GTSAM_CUSPARSE_HAS_SPGEMM_REUSE
  const size_t normalPatternSpan =
      createUnrecordedEventSpan(&profile.patternH2d);
  if (normalPatternSpan == kNoEventSpan) {
    normalEquations.uploadPattern(jacobianColumns, discovered.rowPointers,
                                  discovered.columnIndices, stream);
  } else {
    normalEquations.uploadPattern(jacobianColumns, discovered.rowPointers,
                                  discovered.columnIndices, stream, nullptr,
                                  pendingEventSpans[normalPatternSpan].begin,
                                  pendingEventSpans[normalPatternSpan].end);
    markEventSpanEnded(normalPatternSpan);
  }

  hDiagonalOffsets.resize(discovered.diagonalOffsets.size());
  const size_t diagonalPatternSpan = beginEventSpan(&profile.patternH2d);
  checkRuntimeStatus(cudaMemcpyAsync(hDiagonalOffsets.data(),
                            discovered.diagonalOffsets.data(),
                            sizeof(int) * discovered.diagonalOffsets.size(),
                            cudaMemcpyHostToDevice, stream),
            "H diagonal-offset upload");
  profile.patternH2dBytes += sizeof(int) * (discovered.rowPointers.size() +
                                            discovered.columnIndices.size() +
                                            discovered.diagonalOffsets.size());
  endEventSpan(diagonalPatternSpan);

  const size_t structureSpan = beginEventSpan(&profile.structureSetup);
  undampedDiagonal.resize(static_cast<size_t>(jacobianColumns));
  dampingDiagonal.resize(static_cast<size_t>(jacobianColumns));
  undampedDiagonal.zero(stream);
  dampingDiagonal.zero(stream);
  // beta is zero, but NVIDIA's reuse sample initializes C before binding it.
  normalEquations.zero(stream);

  checkCusparse(cusparseCreateCsr(
                    &hDescriptor, jacobianColumns, jacobianColumns, 0,
                    const_cast<int*>(normalEquations.rowPointers().data()),
                    nullptr, nullptr, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                    CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F),
                "stable H descriptor creation");
  checkCusparse(cusparseSpGEMM_createDescr(&reuseDescriptor),
                "stable reuse descriptor creation");

  size_t bufferSize1 = 0;
  checkCusparse(cusparseSpGEMMreuse_workEstimation(
                    handle, kNoTranspose, kNoTranspose, jtDescriptor,
                    jDescriptor, hDescriptor, kSpGemmAlgorithm, reuseDescriptor,
                    &bufferSize1, nullptr),
                "stable reuse work-estimation query");
  reuseBuffer1.resize(bufferSize1);
  checkCusparse(cusparseSpGEMMreuse_workEstimation(
                    handle, kNoTranspose, kNoTranspose, jtDescriptor,
                    jDescriptor, hDescriptor, kSpGemmAlgorithm, reuseDescriptor,
                    &bufferSize1, reuseBuffer1.data()),
                "stable reuse work-estimation execute");

  size_t bufferSize2 = 0;
  size_t bufferSize3 = 0;
  size_t bufferSize4 = 0;
  checkCusparse(
      cusparseSpGEMMreuse_nnz(handle, kNoTranspose, kNoTranspose, jtDescriptor,
                              jDescriptor, hDescriptor, kSpGemmAlgorithm,
                              reuseDescriptor, &bufferSize2, nullptr,
                              &bufferSize3, nullptr, &bufferSize4, nullptr),
      "stable reuse-nnz workspace query");
  reuseBuffer2.resize(bufferSize2);
  reuseBuffer3.resize(bufferSize3);
  reuseBuffer4.resize(bufferSize4);
  checkCusparse(cusparseSpGEMMreuse_nnz(
                    handle, kNoTranspose, kNoTranspose, jtDescriptor,
                    jDescriptor, hDescriptor, kSpGemmAlgorithm, reuseDescriptor,
                    &bufferSize2, reuseBuffer2.data(), &bufferSize3,
                    reuseBuffer3.data(), &bufferSize4, reuseBuffer4.data()),
                "stable reuse-nnz execute");

  int64_t stableRows = 0;
  int64_t stableColumns = 0;
  int64_t stableNonzeros = 0;
  checkCusparse(cusparseSpMatGetSize(hDescriptor, &stableRows, &stableColumns,
                                     &stableNonzeros),
                "stable H size query");
  if (stableRows != jacobianColumns || stableColumns != jacobianColumns ||
      stableNonzeros != normalEquations.nonzeros()) {
    throw std::runtime_error(
        "stable cuSPARSE SpGEMM discovery disagrees with the discovered H "
        "pattern");
  }

  // This is the final H descriptor's only post-creation pointer binding. It
  // occurs before reuse-copy and is never changed afterward.
  checkCusparse(
      cusparseCsrSetPointers(
          hDescriptor, const_cast<int*>(normalEquations.rowPointers().data()),
          const_cast<int*>(normalEquations.colIndices().data()),
          normalEquations.values().data()),
      "stable H pointer binding before reuse-copy");

  size_t bufferSize5 = 0;
  checkCusparse(
      cusparseSpGEMMreuse_copy(handle, kNoTranspose, kNoTranspose, jtDescriptor,
                               jDescriptor, hDescriptor, kSpGemmAlgorithm,
                               reuseDescriptor, &bufferSize5, nullptr),
      "stable reuse-copy workspace query");
  reuseBuffer5.resize(bufferSize5);
  checkCusparse(cusparseSpGEMMreuse_copy(handle, kNoTranspose, kNoTranspose,
                                         jtDescriptor, jDescriptor, hDescriptor,
                                         kSpGemmAlgorithm, reuseDescriptor,
                                         &bufferSize5, reuseBuffer5.data()),
                "stable reuse-copy execute");
  endEventSpan(structureSpan);

  std::vector<int> stableRowPointers;
  std::vector<int> stableColumnIndices;
  stableRowPointers.resize(static_cast<size_t>(jacobianColumns) + 1);
  stableColumnIndices.resize(discovered.columnIndices.size());
  try {
    const size_t setupDownloadSpan = beginEventSpan(&profile.setupD2h);
    checkRuntimeStatus(cudaMemcpyAsync(stableRowPointers.data(),
                              normalEquations.rowPointers().data(),
                              sizeof(int) * stableRowPointers.size(),
                              cudaMemcpyDeviceToHost, stream),
              "stable H row-offset download");
    checkRuntimeStatus(cudaMemcpyAsync(stableColumnIndices.data(),
                              normalEquations.colIndices().data(),
                              sizeof(int) * stableColumnIndices.size(),
                              cudaMemcpyDeviceToHost, stream),
              "stable H column-index download");
    profile.setupD2hBytes +=
        sizeof(int) * (stableRowPointers.size() + stableColumnIndices.size());
    endEventSpan(setupDownloadSpan);
    checkRuntimeStatus(cudaStreamSynchronize(stream),
              "stable H pattern download synchronization");
  } catch (...) {
    // Keep the host destinations alive until any successful first copy ends.
    synchronizeNoThrow();
    throw;
  }
  if (stableRowPointers != discovered.rowPointers ||
      stableColumnIndices != discovered.columnIndices) {
    throw std::runtime_error(
        "stable cuSPARSE SpGEMM pattern differs from discovery");
  }

  // NVIDIA documents buffers 4 and 5 as the two workspaces retained for
  // reuse-compute. Release 1-3 only after all setup work is synchronized.
  reuseBuffer1.reset();
  reuseBuffer2.reset();
  reuseBuffer3.reset();
#else
  (void)discovered;
  throw std::runtime_error("cuSPARSE SpGEMMreuse support was not configured");
#endif
}

void DeviceSparseJacobianNormalEquations::Impl::createSpmvResources() {
  const size_t structureSpan = beginEventSpan(&profile.structureSetup);
  delta.resize(static_cast<size_t>(jacobianColumns));
  jDelta.resize(static_cast<size_t>(jacobianRows));
  squaredTerms.resize(static_cast<size_t>(jacobianRows));
  oldError.resize(1);
  newError.resize(1);
  delta.zero(stream);
  jDelta.zero(stream);
  squaredTerms.zero(stream);
  oldError.zero(stream);
  newError.zero(stream);

  checkCusparse(
      cusparseCreateDnVec(&bDescriptor, jacobianRows, b.data(), CUDA_R_64F),
      "b dense-vector descriptor creation");
  double* gStorage = backend == DeviceNormalSolverBackend::Cudss
                         ? normalEquations.rhs().data()
                         : gradient.data();
  checkCusparse(cusparseCreateDnVec(&gDescriptor, jacobianColumns, gStorage,
                                    CUDA_R_64F),
                "g dense-vector descriptor creation");
  checkCusparse(cusparseCreateDnVec(&deltaDescriptor, jacobianColumns,
                                    delta.data(), CUDA_R_64F),
                "delta dense-vector descriptor creation");
  checkCusparse(cusparseCreateDnVec(&jDeltaDescriptor, jacobianRows,
                                    jDelta.data(), CUDA_R_64F),
                "J*delta dense-vector descriptor creation");

  size_t transposeMultiplyBytes = 0;
  checkCusparse(
      cusparseSpMV_bufferSize(handle, kNoTranspose, &kAlpha, jtDescriptor,
                              bDescriptor, &kBeta, gDescriptor, CUDA_R_64F,
                              kSpMvAlgorithm, &transposeMultiplyBytes),
      "JT*b SpMV workspace query");
  size_t forwardMultiplyBytes = 0;
  checkCusparse(cusparseSpMV_bufferSize(handle, kNoTranspose, &kAlpha,
                                        jDescriptor, deltaDescriptor, &kBeta,
                                        jDeltaDescriptor, CUDA_R_64F,
                                        kSpMvAlgorithm, &forwardMultiplyBytes),
                "J*delta SpMV workspace query");
  spmvBuffer.resize(std::max(transposeMultiplyBytes, forwardMultiplyBytes));

  size_t cubReductionBytes = 0;
  checkRuntimeStatus(
      cub::DeviceReduce::Sum(nullptr, cubReductionBytes, squaredTerms.data(),
                             oldError.data(), jacobianRows, stream),
      "CUB model-error reduction workspace query");
  cubReductionBuffer.resize(cubReductionBytes);
  endEventSpan(structureSpan);
  downloadedDelta.resize(static_cast<size_t>(jacobianColumns));
  downloadedErrors.resize(2);
}

void DeviceSparseJacobianNormalEquations::Impl::setupPcgStorage() {
  const size_t structureSpan = beginEventSpan(&profile.structureSetup);
  gradient.resize(static_cast<size_t>(jacobianColumns));
  gradient.zero(stream);
  undampedDiagonal.resize(static_cast<size_t>(jacobianColumns));
  dampingDiagonal.resize(static_cast<size_t>(jacobianColumns));
  undampedDiagonal.zero(stream);
  dampingDiagonal.zero(stream);
  endEventSpan(structureSpan);

  pcgSolver.emplace();
  pcgSolver->initialize(handle, jacobianRows, jacobianColumns, jDescriptor,
                        jtDescriptor, jtRowPointers,
                        solverOptions.columnBlockOffsets, solverOptions.pcg,
                        stream, collectProfile);
}

void DeviceSparseJacobianNormalEquations::Impl::setup(
    const SparseJacobianPlan& plan) {
  try {
    checkCusparse(cusparseCreate(&handle), "handle creation");
    checkCusparse(cusparseSetStream(handle, stream), "fixed stream binding");
    checkCusparse(cusparseSetPointerMode(handle, CUSPARSE_POINTER_MODE_HOST),
                  "host pointer-mode selection");

    createJacobianStorage(plan);
    if (backend == DeviceNormalSolverBackend::Cudss) {
      const DiscoveredPattern discovered = discoverNormalPattern();
      createStableNormalStorage(discovered);
    } else {
      setupPcgStorage();
    }
    createSpmvResources();
    capturePointers();
    validatePointers();
    if (collectProfile) {
      checkRuntimeStatus(cudaStreamSynchronize(stream),
                "profiled persistent setup synchronization");
      harvestPendingEvents();
    }
  } catch (...) {
    // The candidate Impl still owns all persistent resources here.
    synchronizeNoThrow();
    throw;
  }
}

void DeviceSparseJacobianNormalEquations::Impl::upload(
    const HostSparseJacobian& host, cudaStream_t suppliedStream) {
  validateStream(suppliedStream);
  validatePointers();
  if (host.structuralFingerprint() != jacobianStructuralFingerprint) {
    throw std::invalid_argument(
        "DeviceSparseJacobianNormalEquations host structural fingerprint "
        "does not match the initialized plan");
  }
  if (host.valuesSize() != static_cast<size_t>(jacobianNonzeros) ||
      host.rhsSize() != static_cast<size_t>(jacobianRows)) {
    throw std::invalid_argument(
        "DeviceSparseJacobianNormalEquations host numerical sizes do not "
        "match the initialized plan");
  }

  numericsUploaded = false;
  formed = false;
  dampingPrepared = false;
  attemptReady = false;
  try {
    const size_t numericUploadSpan = beginEventSpan(&profile.numericH2d);
    checkRuntimeStatus(cudaMemcpyAsync(jValues.data(), host.valuesData(),
                              sizeof(double) * host.valuesSize(),
                              cudaMemcpyHostToDevice, stream),
              "J values upload");
    checkRuntimeStatus(cudaMemcpyAsync(b.data(), host.rhsData(),
                              sizeof(double) * host.rhsSize(),
                              cudaMemcpyHostToDevice, stream),
              "b upload");
    profile.numericH2dBytes +=
        sizeof(double) * (host.valuesSize() + host.rhsSize());
    endEventSpan(numericUploadSpan);
    numericsUploaded = true;
  } catch (...) {
    // A successful first copy still references the caller's pinned storage.
    synchronizeNoThrow();
    throw;
  }
}

void DeviceSparseJacobianNormalEquations::Impl::form(
    cudaStream_t suppliedStream) {
  validateStream(suppliedStream);
  validatePointers();
  if (!numericsUploaded) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations form requires uploaded "
        "numerics");
  }
  formed = false;
  dampingPrepared = false;
  attemptReady = false;
  try {
    size_t stageSpan = beginEventSpan(&profile.transposeUpdate);
    transposeJacobian();
    endEventSpan(stageSpan);

    if (backend == DeviceNormalSolverBackend::Cudss) {
      stageSpan = beginEventSpan(&profile.normalJtJ);
#if GTSAM_CUSPARSE_HAS_SPGEMM_REUSE
      checkCusparse(
          cusparseSpGEMMreuse_compute(handle, kNoTranspose, kNoTranspose,
                                      &kAlpha, jtDescriptor, jDescriptor,
                                      &kBeta, hDescriptor, CUDA_R_64F,
                                      kSpGemmAlgorithm, reuseDescriptor),
          "stable H reuse-compute");
#else
      throw std::runtime_error(
          "cuSPARSE SpGEMMreuse support was not configured");
#endif
      endEventSpan(stageSpan);
    }

    stageSpan = beginEventSpan(&profile.normalJtb);
    checkCusparse(cusparseSpMV(handle, kNoTranspose, &kAlpha, jtDescriptor,
                               bDescriptor, &kBeta, gDescriptor, CUDA_R_64F,
                               kSpMvAlgorithm, spmvBuffer.data()),
                  "JT*b SpMV");
    endEventSpan(stageSpan);

    if (backend == DeviceNormalSolverBackend::Cudss) {
      stageSpan = beginEventSpan(&profile.diagonalExtraction);
      gatherDiagonalKernel<<<kernelGridSize(jacobianColumns), kKernelBlockSize,
                             0, stream>>>(
          normalEquations.values().data(), hDiagonalOffsets.data(),
          jacobianColumns, undampedDiagonal.data());
      checkRuntimeStatus(cudaGetLastError(), "undamped H diagonal gather launch");
      endEventSpan(stageSpan);
    } else {
      stageSpan = beginEventSpan(&profile.diagonalExtraction);
      columnSquaredNormsKernel<<<kernelGridSize(jacobianColumns),
                                 kKernelBlockSize, 0, stream>>>(
          jtRowPointers.data(), jtValues.data(), jacobianColumns,
          undampedDiagonal.data());
      checkRuntimeStatus(cudaGetLastError(), "undamped diagonal column-norm launch");
      endEventSpan(stageSpan);
      pcgSolver->buildPreconditioner(jtValues, stream);
      if (collectProfile) {
        profile.pcgPreconditionerBuild =
            pcgSolver->preconditionerBuildSeconds();
      }
    }

    stageSpan = beginEventSpan(&profile.oldModelError);
    oldErrorTermsKernel<<<kernelGridSize(jacobianRows), kKernelBlockSize, 0,
                          stream>>>(b.data(), jacobianRows,
                                    squaredTerms.data());
    checkRuntimeStatus(cudaGetLastError(), "old model-error terms launch");
    size_t reductionBytes = cubReductionBuffer.size();
    checkRuntimeStatus(cub::DeviceReduce::Sum(cubReductionBuffer.data(), reductionBytes,
                                     squaredTerms.data(), oldError.data(),
                                     jacobianRows, stream),
              "old model-error reduction");
    endEventSpan(stageSpan);
    formed = true;
  } catch (...) {
    synchronizeNoThrow();
    throw;
  }
}

void DeviceSparseJacobianNormalEquations::Impl::prepare(
    bool diagonalDamping, double minDiagonal, double maxDiagonal,
    cudaStream_t suppliedStream) {
  validateStream(suppliedStream);
  validatePointers();
  if (!std::isfinite(minDiagonal) || !std::isfinite(maxDiagonal) ||
      minDiagonal < 0.0 || minDiagonal > maxDiagonal) {
    throw std::invalid_argument(
        "DeviceSparseJacobianNormalEquations damping limits must be finite, "
        "nonnegative, and ordered");
  }
  if (!formed) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations damping requires a formed "
        "undamped system");
  }

  dampingPrepared = false;
  attemptReady = false;
  try {
    const size_t dampingSpan = beginEventSpan(&profile.dampingPreparation);
    prepareDampingKernel<<<kernelGridSize(jacobianColumns), kKernelBlockSize, 0,
                           stream>>>(undampedDiagonal.data(), jacobianColumns,
                                     diagonalDamping, minDiagonal, maxDiagonal,
                                     dampingDiagonal.data());
    checkRuntimeStatus(cudaGetLastError(), "damping diagonal preparation launch");
    endEventSpan(dampingSpan);
    dampingPrepared = true;
  } catch (...) {
    synchronizeNoThrow();
    throw;
  }
}

void DeviceSparseJacobianNormalEquations::Impl::applyExplicitDamping(
    double lambda, cudaStream_t suppliedStream) {
  validateStream(suppliedStream);
  validatePointers();
  if (backend != DeviceNormalSolverBackend::Cudss) {
    throw std::logic_error(
        "explicit damping requires the materialized cuDSS system");
  }
  if (!std::isfinite(lambda) || lambda < 0.0) {
    throw std::invalid_argument(
        "explicit damping lambda must be finite and nonnegative");
  }
  if (!formed || !dampingPrepared) {
    throw std::logic_error(
        "explicit damping requires a formed system and prepared damping");
  }
  const size_t stageSpan = beginEventSpan(&profile.dampingApplication);
  applyDampingKernel<<<kernelGridSize(jacobianColumns), kKernelBlockSize, 0,
                       stream>>>(
      normalEquations.values().data(), hDiagonalOffsets.data(),
      undampedDiagonal.data(), dampingDiagonal.data(), jacobianColumns,
      lambda);
  checkRuntimeStatus(cudaGetLastError(), "damped H diagonal write launch");
  endEventSpan(stageSpan);
  attemptReady = false;
}

void DeviceSparseJacobianNormalEquations::Impl::evaluateSolvedDelta(
    cudaStream_t suppliedStream) {
  validateStream(suppliedStream);
  validatePointers();
  const size_t newErrorSpan = beginEventSpan(&profile.newModelError);
  checkCusparse(cusparseSpMV(handle, kNoTranspose, &kAlpha, jDescriptor,
                             deltaDescriptor, &kBeta, jDeltaDescriptor,
                             CUDA_R_64F, kSpMvAlgorithm, spmvBuffer.data()),
                "J*delta SpMV");
  newErrorTermsKernel<<<kernelGridSize(jacobianRows), kKernelBlockSize, 0,
                        stream>>>(jDelta.data(), b.data(), jacobianRows,
                                  squaredTerms.data());
  checkRuntimeStatus(cudaGetLastError(), "new model-error terms launch");
  size_t reductionBytes = cubReductionBuffer.size();
  checkRuntimeStatus(cub::DeviceReduce::Sum(cubReductionBuffer.data(), reductionBytes,
                                   squaredTerms.data(), newError.data(),
                                   jacobianRows, stream),
            "new model-error reduction");
  endEventSpan(newErrorSpan);
  attemptReady = true;
}

DeviceSparseJacobianAttemptResult
DeviceSparseJacobianNormalEquations::Impl::downloadAttempt(
    cudaStream_t suppliedStream) {
  validateStream(suppliedStream);
  validatePointers();
  if (!attemptReady) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations has no successful attempt to "
        "download");
  }

  try {
    const size_t attemptDownloadSpan = beginEventSpan(&profile.attemptD2h);
    checkRuntimeStatus(
        cudaMemcpyAsync(downloadedDelta.data(), delta.data(),
                        sizeof(double) * static_cast<size_t>(jacobianColumns),
                        cudaMemcpyDeviceToHost, stream),
        "attempt delta download");
    checkRuntimeStatus(cudaMemcpyAsync(downloadedErrors.data(), oldError.data(),
                              sizeof(double), cudaMemcpyDeviceToHost, stream),
              "old model-error download");
    checkRuntimeStatus(cudaMemcpyAsync(downloadedErrors.data() + 1, newError.data(),
                              sizeof(double), cudaMemcpyDeviceToHost, stream),
              "new model-error download");
    profile.attemptD2hBytes +=
        sizeof(double) * (static_cast<size_t>(jacobianColumns) + 2);
    endEventSpan(attemptDownloadSpan);
    checkRuntimeStatus(cudaStreamSynchronize(stream),
              "attempt result download synchronization");
    if (collectProfile) harvestPendingEvents();
  } catch (...) {
    // Keep all pinned destinations alive until any successful copy completes.
    synchronizeNoThrow();
    throw;
  }

  std::chrono::steady_clock::time_point hostBuildStart;
  if (collectProfile) hostBuildStart = std::chrono::steady_clock::now();
  DeviceSparseJacobianAttemptResult result;
  result.delta = Vector(jacobianColumns);
  for (int index = 0; index < jacobianColumns; ++index) {
    result.delta(index) = downloadedDelta.data()[static_cast<size_t>(index)];
  }
  result.model.oldError = downloadedErrors.data()[0];
  result.model.newError = downloadedErrors.data()[1];
  if (collectProfile) {
    profile.attemptHostBuild +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      hostBuildStart)
            .count();
  }
  return result;
}

DeviceSparseJacobianNormalEquations::DeviceSparseJacobianNormalEquations() =
    default;

DeviceSparseJacobianNormalEquations::~DeviceSparseJacobianNormalEquations() =
    default;

DeviceSparseJacobianNormalEquations::DeviceSparseJacobianNormalEquations(
    DeviceSparseJacobianNormalEquations&&) noexcept = default;

DeviceSparseJacobianNormalEquations&
DeviceSparseJacobianNormalEquations::operator=(
    DeviceSparseJacobianNormalEquations&&) noexcept = default;

DeviceSparseJacobianCapability
DeviceSparseJacobianNormalEquations::preflightCapability() {
  return preflightCapability(DeviceNormalSolverBackend::Cudss);
}

DeviceSparseJacobianCapability
DeviceSparseJacobianNormalEquations::preflightCapability(
    DeviceNormalSolverBackend backend) {
  if (backend == DeviceNormalSolverBackend::Pcg) {
    return {true,
            "matrix-free PCG requires only cuSPARSE SpMV and csr2csc"};
  }
#if GTSAM_CUSPARSE_HAS_SPGEMM_REUSE
  return {true, "configured cuSPARSE provides persistent SpGEMMreuse support"};
#else
  return {false,
          "configured cuSPARSE headers and library do not expose "
          "cusparseSpGEMMreuse_workEstimation"};
#endif
}

void DeviceSparseJacobianNormalEquations::initialize(
    const SparseJacobianPlan& plan, cudaStream_t stream, bool collectProfile,
    const DeviceNormalSolverOptions& solverOptions) {
  // validate the host plan before capability checks or any CUDA call.
  validatePositivePlan(plan);
  const DeviceSparseJacobianCapability capability =
      preflightCapability(solverOptions.backend);
  if (!capability.supported) {
    throw std::runtime_error(capability.detail);
  }

  std::chrono::steady_clock::time_point initializeStart;
  if (collectProfile) initializeStart = std::chrono::steady_clock::now();
  auto replacement = std::make_unique<Impl>(stream, collectProfile);
  replacement->backend = solverOptions.backend;
  replacement->solverOptions = solverOptions;
  replacement->setup(plan);
  if (collectProfile) {
    replacement->profile.initializeWall +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      initializeStart)
            .count();
  }
  impl_ = std::move(replacement);
}

void DeviceSparseJacobianNormalEquations::uploadNumerics(
    const HostSparseJacobian& host, cudaStream_t stream) {
  if (!impl_) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations is not initialized");
  }
  impl_->upload(host, stream);
}

void DeviceSparseJacobianNormalEquations::formUndampedSystem(
    cudaStream_t stream) {
  if (!impl_) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations is not initialized");
  }
  impl_->form(stream);
}

void DeviceSparseJacobianNormalEquations::prepareDamping(bool diagonalDamping,
                                                         double minDiagonal,
                                                         double maxDiagonal,
                                                         cudaStream_t stream) {
  if (!impl_) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations is not initialized");
  }
  impl_->prepare(diagonalDamping, minDiagonal, maxDiagonal, stream);
}

void DeviceSparseJacobianNormalEquations::applyExplicitDamping(
    double lambda, cudaStream_t stream) {
  if (!impl_) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations is not initialized");
  }
  impl_->applyExplicitDamping(lambda, stream);
}

void DeviceSparseJacobianNormalEquations::prepareOperatorSystem(
    double lambda, cudaStream_t stream) {
  if (!impl_ || impl_->backend != DeviceNormalSolverBackend::Pcg ||
      !impl_->pcgSolver) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations has no PCG operator system");
  }
  impl_->validateStream(stream);
  if (!impl_->formed || !impl_->dampingPrepared) {
    throw std::logic_error(
        "PCG operator preparation requires a formed system and damping");
  }
  impl_->pcgSolver->prepare(lambda, impl_->dampingDiagonal, stream);
  impl_->attemptReady = false;
}

const LinearOperator&
DeviceSparseJacobianNormalEquations::linearOperator() const {
  if (!impl_ || impl_->backend != DeviceNormalSolverBackend::Pcg ||
      !impl_->pcgSolver) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations has no PCG linear operator");
  }
  return impl_->pcgSolver->linearOperator();
}

const Preconditioner&
DeviceSparseJacobianNormalEquations::preconditioner() const {
  if (!impl_ || impl_->backend != DeviceNormalSolverBackend::Pcg ||
      !impl_->pcgSolver) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations has no PCG preconditioner");
  }
  return impl_->pcgSolver->preconditioner();
}

const double* DeviceSparseJacobianNormalEquations::deviceRhs() const {
  if (!impl_ || impl_->backend != DeviceNormalSolverBackend::Pcg) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations has no PCG right-hand side");
  }
  return impl_->gradient.data();
}

void DeviceSparseJacobianNormalEquations::evaluateSolvedDelta(
    cudaStream_t stream) {
  if (!impl_) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations is not initialized");
  }
  impl_->evaluateSolvedDelta(stream);
}

DeviceSparseJacobianAttemptResult
DeviceSparseJacobianNormalEquations::downloadAttemptResult(
    cudaStream_t stream) const {
  if (!impl_) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations is not initialized");
  }
  return impl_->downloadAttempt(stream);
}

const DeviceSparseJacobianProfile&
DeviceSparseJacobianNormalEquations::profile() const {
  static const DeviceSparseJacobianProfile emptyProfile;
  return impl_ ? impl_->profile : emptyProfile;
}

bool DeviceSparseJacobianNormalEquations::hasNormalMatrix() const {
  return impl_ && impl_->backend == DeviceNormalSolverBackend::Cudss;
}

DeviceSparseSpdSystem&
DeviceSparseJacobianNormalEquations::mutableSystem() {
  if (!impl_ || impl_->backend != DeviceNormalSolverBackend::Cudss) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations has no explicit normal system");
  }
  return impl_->normalEquations;
}

DeviceArray<double>& DeviceSparseJacobianNormalEquations::deviceDelta() {
  if (!impl_) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations is not initialized");
  }
  return impl_->delta;
}

const DeviceSparseSpdSystem& DeviceSparseJacobianNormalEquations::system()
    const {
  if (!impl_) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations is not initialized");
  }
  if (impl_->backend != DeviceNormalSolverBackend::Cudss) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations does not materialize the normal "
        "matrix in PCG mode");
  }
  return impl_->normalEquations;
}

}  // namespace gtsam::cuda
