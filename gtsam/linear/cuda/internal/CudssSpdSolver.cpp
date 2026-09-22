/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CudssSpdSolver.cpp
 * @brief   Reusable cuDSS analysis, factorization, and solve for one pattern
 * @author  Ruogu Li
 * @date    Jun 17, 2026
 */

#if GTSAM_ENABLE_CUDA

#include <gtsam/base/cuda/Errors.h>
#include <gtsam/linear/cuda/internal/CudssSpdSolver.h>

#include <chrono>
#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>

#if GTSAM_ENABLE_CUDSS

class EventSpan {
 public:
  EventSpan() {
    GTSAM_CUDA_CHECK(cudaEventCreate(&begin_));
    try {
      GTSAM_CUDA_CHECK(cudaEventCreate(&end_));
    } catch (...) {
      cudaEventDestroy(begin_);
      throw;
    }
  }
  ~EventSpan() {
    if (end_) cudaEventDestroy(end_);
    if (begin_) cudaEventDestroy(begin_);
  }
  EventSpan(const EventSpan&) = delete;
  EventSpan& operator=(const EventSpan&) = delete;

  void recordBegin(cudaStream_t stream) {
    GTSAM_CUDA_CHECK(cudaEventRecord(begin_, stream));
  }
  void recordEnd(cudaStream_t stream) {
    GTSAM_CUDA_CHECK(cudaEventRecord(end_, stream));
  }
  double completedSeconds() const {
    GTSAM_CUDA_CHECK(cudaEventSynchronize(end_));
    float milliseconds = 0.0f;
    GTSAM_CUDA_CHECK(cudaEventElapsedTime(&milliseconds, begin_, end_));
    return static_cast<double>(milliseconds) / 1000.0;
  }

 private:
  cudaEvent_t begin_ = nullptr;
  cudaEvent_t end_ = nullptr;
};
#include <cudss.h>
#endif

namespace gtsam::cuda {
namespace {

#if GTSAM_ENABLE_CUDSS

const char* cudssStatusName(cudssStatus_t status) {
  switch (status) {
    case CUDSS_STATUS_SUCCESS:
      return "CUDSS_STATUS_SUCCESS";
    case CUDSS_STATUS_NOT_INITIALIZED:
      return "CUDSS_STATUS_NOT_INITIALIZED";
    case CUDSS_STATUS_ALLOC_FAILED:
      return "CUDSS_STATUS_ALLOC_FAILED";
    case CUDSS_STATUS_INVALID_VALUE:
      return "CUDSS_STATUS_INVALID_VALUE";
    case CUDSS_STATUS_NOT_SUPPORTED:
      return "CUDSS_STATUS_NOT_SUPPORTED";
    case CUDSS_STATUS_EXECUTION_FAILED:
      return "CUDSS_STATUS_EXECUTION_FAILED";
    case CUDSS_STATUS_INTERNAL_ERROR:
      return "CUDSS_STATUS_INTERNAL_ERROR";
    case CUDSS_STATUS_IR_FAILED:
      return "CUDSS_STATUS_IR_FAILED";
  }
  return "CUDSS_STATUS_UNKNOWN";
}

const char* cudssPhaseName(int phase) {
  switch (phase) {
    case CUDSS_PHASE_ANALYSIS:
      return "CUDSS_PHASE_ANALYSIS";
    case CUDSS_PHASE_FACTORIZATION:
      return "CUDSS_PHASE_FACTORIZATION";
    case CUDSS_PHASE_SOLVE:
      return "CUDSS_PHASE_SOLVE";
  }
  return "CUDSS_PHASE_UNKNOWN";
}

void checkCudss(cudssStatus_t status, const char* expression) {
  if (status == CUDSS_STATUS_SUCCESS) return;
  std::ostringstream os;
  os << "cuDSS call failed: " << expression << " returned "
     << cudssStatusName(status) << " (" << static_cast<int>(status) << ")";
  throw std::runtime_error(os.str());
}

void checkCudssExecute(cudssStatus_t status, int phase) {
  if (status == CUDSS_STATUS_SUCCESS) return;
  std::ostringstream os;
  os << "cuDSS " << cudssPhaseName(phase) << " failed with "
     << cudssStatusName(status) << " (" << static_cast<int>(status) << ")";
  throw std::runtime_error(os.str());
}

#define GTSAM_CUDSS_CHECK(expr) checkCudss((expr), #expr)

struct CudssHandle {
  cudssHandle_t value = nullptr;

  CudssHandle() { GTSAM_CUDSS_CHECK(cudssCreate(&value)); }

  CudssHandle(const CudssHandle&) = delete;
  CudssHandle& operator=(const CudssHandle&) = delete;

  ~CudssHandle() {
    if (value) {
      cudssDestroy(value);
    }
  }
};

struct CudssConfig {
  cudssConfig_t value = nullptr;

  CudssConfig() { GTSAM_CUDSS_CHECK(cudssConfigCreate(&value)); }

  CudssConfig(const CudssConfig&) = delete;
  CudssConfig& operator=(const CudssConfig&) = delete;

  ~CudssConfig() {
    if (value) {
      cudssConfigDestroy(value);
    }
  }
};

struct CudssData {
  cudssHandle_t handle = nullptr;
  cudssData_t value = nullptr;

  explicit CudssData(cudssHandle_t handleIn) : handle(handleIn) {
    GTSAM_CUDSS_CHECK(cudssDataCreate(handle, &value));
  }

  CudssData(const CudssData&) = delete;
  CudssData& operator=(const CudssData&) = delete;

  ~CudssData() {
    if (value) {
      cudssDataDestroy(handle, value);
    }
  }
};

struct CudssMatrix {
  cudssMatrix_t value = nullptr;

  CudssMatrix() = default;

  CudssMatrix(const CudssMatrix&) = delete;
  CudssMatrix& operator=(const CudssMatrix&) = delete;

  ~CudssMatrix() {
    if (value) {
      cudssMatrixDestroy(value);
    }
  }

  void reset() {
    if (value) {
      cudssMatrixDestroy(value);
      value = nullptr;
    }
  }
};

void validateSystemForSolve(const DeviceSparseSpdSystem& system,
                            const DeviceArray<double>* solution) {
  if (!solution) {
    throw std::invalid_argument("CudssSpdSolver requires solution storage");
  }
  if (system.rows() <= 0) {
    throw std::invalid_argument("CudssSpdSolver requires non-empty system");
  }
  if (system.nonzeros() <= 0) {
    throw std::invalid_argument("CudssSpdSolver requires non-empty matrix");
  }
  if (system.rowPointers().size() != static_cast<size_t>(system.rows()) + 1) {
    throw std::invalid_argument("CudssSpdSolver row pointer size mismatch");
  }
  if (system.colIndices().size() != static_cast<size_t>(system.nonzeros())) {
    throw std::invalid_argument("CudssSpdSolver column index size mismatch");
  }
  if (system.values().size() != static_cast<size_t>(system.nonzeros())) {
    throw std::invalid_argument("CudssSpdSolver value size mismatch");
  }
  if (system.rhs().size() != static_cast<size_t>(system.rows())) {
    throw std::invalid_argument("CudssSpdSolver rhs size mismatch");
  }
}

void validatePermutation(const std::vector<int>& permutation, int rows) {
  if (permutation.size() != static_cast<size_t>(rows)) {
    throw std::invalid_argument(
        "CudssSpdSolver permutation size must equal system dimension");
  }
  std::vector<bool> seen(static_cast<size_t>(rows), false);
  for (const int scalar : permutation) {
    if (scalar < 0 || scalar >= rows) {
      throw std::invalid_argument(
          "CudssSpdSolver permutation entry is out of range");
    }
    if (seen[static_cast<size_t>(scalar)]) {
      throw std::invalid_argument(
          "CudssSpdSolver permutation contains a duplicate entry");
    }
    seen[static_cast<size_t>(scalar)] = true;
  }
}

cudssStatus_t createSpdCsrMatrix(CudssMatrix* matrix,
                                 const DeviceSparseSpdSystem& system) {
  const int rows = system.rows();
  cudssStatus_t csrStatus = cudssMatrixCreateCsr(
      &matrix->value, rows, rows, system.nonzeros(),
      system.rowPointers().data(), system.rowPointers().data() + 1,
      system.colIndices().data(), system.values().data(), CUDSS_R_32I,
      CUDSS_R_32I, CUDSS_R_64F, CUDSS_MTYPE_SPD, CUDSS_MVIEW_UPPER,
      CUDSS_BASE_ZERO);
  if (csrStatus == CUDSS_STATUS_NOT_SUPPORTED) {
    matrix->reset();
    // cuDSS 0.8 accepts standard CSR offsets with rowEnd == nullptr.
    csrStatus = cudssMatrixCreateCsr(
        &matrix->value, rows, rows, system.nonzeros(),
        system.rowPointers().data(), nullptr, system.colIndices().data(),
        system.values().data(), CUDSS_R_32I, CUDSS_R_32I, CUDSS_R_64F,
        CUDSS_MTYPE_SPD, CUDSS_MVIEW_UPPER, CUDSS_BASE_ZERO);
  }
  return csrStatus;
}

#endif

}  // namespace

struct CudssSpdSolver::Impl {
  mutable LinearSolveStats stats;
  std::vector<int> appliedPermutation;
#if GTSAM_ENABLE_CUDSS
  CudssHandle handle;
  CudssConfig config;
  CudssData data;
  CudssMatrix matrix;
  CudssMatrix x;
  CudssMatrix b;
  int rows = 0;
  int nonzeros = 0;
  const int* rowPointers = nullptr;
  const int* colIndices = nullptr;
  const double* values = nullptr;
  const double* rhs = nullptr;
  double* solution = nullptr;
  bool analyzed = false;
  mutable std::vector<std::unique_ptr<EventSpan>> pendingSolveSpans;

  Impl() : data(handle.value) {}

  void harvestSolveTimings() const {
    for (const auto& span : pendingSolveSpans) {
      stats.solveSeconds += span->completedSeconds();
    }
    pendingSolveSpans.clear();
  }

  void analyze(const DeviceSparseSpdSystem& system,
               DeviceArray<double>* solutionArray,
               const std::vector<int>* scalarPermutation,
               cudaStream_t stream) {
    validateSystemForSolve(system, solutionArray);
    if (scalarPermutation) {
      validatePermutation(*scalarPermutation, system.rows());
    }

    rows = system.rows();
    nonzeros = system.nonzeros();
    solutionArray->resize(static_cast<size_t>(rows));

    GTSAM_CUDSS_CHECK(cudssSetStream(handle.value, stream));

    matrix.reset();
    x.reset();
    b.reset();

    checkCudss(createSpdCsrMatrix(&matrix, system), "cudssMatrixCreateCsr");
    GTSAM_CUDSS_CHECK(cudssMatrixCreateDn(&x.value, rows, 1, rows,
                                          solutionArray->data(), CUDSS_R_64F,
                                          CUDSS_LAYOUT_COL_MAJOR));
    GTSAM_CUDSS_CHECK(cudssMatrixCreateDn(&b.value, rows, 1, rows,
                                          system.rhs().data(), CUDSS_R_64F,
                                          CUDSS_LAYOUT_COL_MAJOR));

    if (scalarPermutation) {
      const cudssReorderingAlg_t reordering = CUDSS_REORDERING_ALG_NONE;
      GTSAM_CUDSS_CHECK(cudssConfigSet(
          config.value, CUDSS_CONFIG_REORDERING_ALG, &reordering,
          sizeof(reordering)));
      GTSAM_CUDSS_CHECK(cudssDataSet(
          handle.value, data.value, CUDSS_DATA_USER_PERM,
          scalarPermutation->data(), scalarPermutation->size() * sizeof(int)));
    }

    EventSpan analysisSpan;
    analysisSpan.recordBegin(stream);
    checkCudssExecute(
        cudssExecute(handle.value, CUDSS_PHASE_ANALYSIS, config.value,
                     data.value, matrix.value, x.value, b.value),
        CUDSS_PHASE_ANALYSIS);
    analysisSpan.recordEnd(stream);
    stats.backend = LinearSolverType::Cudss;
    stats.userOrderingApplied = scalarPermutation != nullptr;
    ++stats.analysisCount;
    stats.analysisSeconds += analysisSpan.completedSeconds();
    if (scalarPermutation) {
      appliedPermutation.resize(static_cast<size_t>(rows));
      size_t permutationBytes = 0;
      GTSAM_CUDSS_CHECK(cudssDataGet(
          handle.value, data.value, CUDSS_DATA_USER_PERM,
          appliedPermutation.data(), appliedPermutation.size() * sizeof(int),
          &permutationBytes));
      if (permutationBytes != appliedPermutation.size() * sizeof(int) ||
          appliedPermutation != *scalarPermutation) {
        throw std::runtime_error(
            "cuDSS did not retain the requested user permutation");
      }
    }

    rowPointers = system.rowPointers().data();
    colIndices = system.colIndices().data();
    values = system.values().data();
    rhs = system.rhs().data();
    solution = solutionArray->data();
    analyzed = true;
  }

  void solve(const DeviceSparseSpdSystem& system,
             DeviceArray<double>* solutionArray, cudaStream_t stream) {
    validateSystemForSolve(system, solutionArray);
    if (!analyzed) {
      throw std::logic_error("CudssSpdSolver::solve called before analyze");
    }
    if (system.rows() != rows || system.nonzeros() != nonzeros ||
        system.rowPointers().data() != rowPointers ||
        system.colIndices().data() != colIndices ||
        system.values().data() != values || system.rhs().data() != rhs) {
      throw std::invalid_argument(
          "CudssSpdSolver::solve requires the analyzed CSR storage");
    }

    solutionArray->resize(static_cast<size_t>(rows));
    if (solutionArray->data() != solution) {
      throw std::invalid_argument(
          "CudssSpdSolver::solve requires the analyzed solution storage");
    }

    GTSAM_CUDSS_CHECK(cudssSetStream(handle.value, stream));
    EventSpan factorizationSpan;
    factorizationSpan.recordBegin(stream);
    checkCudssExecute(
        cudssExecute(handle.value, CUDSS_PHASE_FACTORIZATION, config.value,
                     data.value, matrix.value, x.value, b.value),
        CUDSS_PHASE_FACTORIZATION);
    factorizationSpan.recordEnd(stream);
    ++stats.factorizationCount;

    int info = 0;
    size_t bytesWritten = 0;
    const auto dataInfoStart = std::chrono::steady_clock::now();
    const cudssStatus_t dataInfoStatus =
        cudssDataGet(handle.value, data.value, CUDSS_DATA_INFO, &info,
                     sizeof(info), &bytesWritten);
    stats.dataInfoBoundarySeconds +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      dataInfoStart)
            .count();
    checkCudss(dataInfoStatus, "cudssDataGet(CUDSS_DATA_INFO)");
    if (bytesWritten != sizeof(info)) {
      throw std::runtime_error(
          "cuDSS CUDSS_DATA_INFO returned an unexpected byte count");
    }
    if (info != 0) {
      std::ostringstream os;
      os << "cuDSS factorization CUDSS_DATA_INFO=" << info
         << " (reordered 1-based first non-positive minor)";
      throw std::runtime_error(os.str());
    }
    // DATA_INFO is the mandatory numerical-completion boundary. Measure the
    // completed GPU factorization rather than only cudssExecute enqueue time.
    stats.factorizationSeconds += factorizationSpan.completedSeconds();

    auto solveSpan = std::make_unique<EventSpan>();
    solveSpan->recordBegin(stream);
    checkCudssExecute(
        cudssExecute(handle.value, CUDSS_PHASE_SOLVE, config.value, data.value,
                     matrix.value, x.value, b.value),
        CUDSS_PHASE_SOLVE);
    solveSpan->recordEnd(stream);
    pendingSolveSpans.push_back(std::move(solveSpan));
    ++stats.solveCount;
  }
#endif
};

CudssSpdSolver::CudssSpdSolver() : impl_(std::make_unique<Impl>()) {}

CudssSpdSolver::~CudssSpdSolver() = default;

CudssSpdSolver::CudssSpdSolver(CudssSpdSolver&&) noexcept = default;

CudssSpdSolver& CudssSpdSolver::operator=(CudssSpdSolver&&) noexcept = default;

void CudssSpdSolver::analyze(const DeviceSparseSpdSystem& system,
                             DeviceArray<double>* solution,
                             cudaStream_t stream) {
#if !GTSAM_ENABLE_CUDSS
  (void)system;
  (void)solution;
  (void)stream;
  throw std::runtime_error("CudssSpdSolver::analyze requires cuDSS");
#else
  impl_ = std::make_unique<Impl>();
  impl_->analyze(system, solution, nullptr, stream);
#endif
}

void CudssSpdSolver::analyze(
    const DeviceSparseSpdSystem& system,
    DeviceArray<double>* solution,
    const std::vector<int>& scalarPermutation, cudaStream_t stream) {
#if !GTSAM_ENABLE_CUDSS
  (void)system;
  (void)solution;
  (void)scalarPermutation;
  (void)stream;
  throw std::runtime_error("CudssSpdSolver::analyze requires cuDSS");
#else
  validatePermutation(scalarPermutation, system.rows());
  impl_ = std::make_unique<Impl>();
  impl_->analyze(system, solution, &scalarPermutation, stream);
#endif
}

void CudssSpdSolver::solve(const DeviceSparseSpdSystem& system,
                           DeviceArray<double>* solution,
                           cudaStream_t stream) {
#if !GTSAM_ENABLE_CUDSS
  (void)system;
  (void)solution;
  (void)stream;
  throw std::runtime_error("CudssSpdSolver::solve requires cuDSS");
#else
  impl_->solve(system, solution, stream);
#endif
}

const LinearSolveStats& CudssSpdSolver::stats() const {
#if GTSAM_ENABLE_CUDSS
  impl_->harvestSolveTimings();
#endif
  return impl_->stats;
}

const std::vector<int>& CudssSpdSolver::appliedPermutation() const {
  return impl_->appliedPermutation;
}

}  // namespace gtsam::cuda

#endif
