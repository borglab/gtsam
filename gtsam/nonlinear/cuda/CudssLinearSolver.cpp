#if GTSAM_ENABLE_CUDA

#include <gtsam/nonlinear/cuda/CudssLinearSolver.h>

#include <sstream>
#include <stdexcept>

#if GTSAM_ENABLE_CUDSS
#include <cudss.h>
#endif

namespace gtsam::cuda {
namespace {

#if GTSAM_ENABLE_CUDSS

const char* CudssStatusName(cudssStatus_t status) {
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

const char* CudssPhaseName(int phase) {
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

void CheckCudss(cudssStatus_t status, const char* expression) {
  if (status == CUDSS_STATUS_SUCCESS) return;
  std::ostringstream os;
  os << "cuDSS call failed: " << expression << " returned "
     << CudssStatusName(status) << " (" << static_cast<int>(status) << ")";
  throw std::runtime_error(os.str());
}

void CheckCudssExecute(cudssStatus_t status, int phase) {
  if (status == CUDSS_STATUS_SUCCESS) return;
  std::ostringstream os;
  os << "cuDSS " << CudssPhaseName(phase) << " failed with "
     << CudssStatusName(status) << " (" << static_cast<int>(status) << ")";
  throw std::runtime_error(os.str());
}

#define GTSAM_CUDSS_CHECK(expr) CheckCudss((expr), #expr)

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

void ValidateSystemForSolve(const DeviceSparseNormalEquations& system,
                            const CudaDeviceArray<double>* solution) {
  if (!solution) {
    throw std::invalid_argument("CudssLinearSolver requires solution storage");
  }
  if (system.rows() <= 0) {
    throw std::invalid_argument("CudssLinearSolver requires non-empty system");
  }
  if (system.nonzeros() <= 0) {
    throw std::invalid_argument("CudssLinearSolver requires non-empty matrix");
  }
  if (system.rowPointers().size() !=
      static_cast<size_t>(system.rows()) + 1) {
    throw std::invalid_argument("CudssLinearSolver row pointer size mismatch");
  }
  if (system.colIndices().size() != static_cast<size_t>(system.nonzeros())) {
    throw std::invalid_argument("CudssLinearSolver column index size mismatch");
  }
  if (system.values().size() != static_cast<size_t>(system.nonzeros())) {
    throw std::invalid_argument("CudssLinearSolver value size mismatch");
  }
  if (system.rhs().size() != static_cast<size_t>(system.rows())) {
    throw std::invalid_argument("CudssLinearSolver rhs size mismatch");
  }
}

#endif

}  // namespace

void CudssLinearSolver::solveSpd(const DeviceSparseNormalEquations& system,
                                 CudaDeviceArray<double>* solution,
                                 cudaStream_t stream) const {
#if !GTSAM_ENABLE_CUDSS
  (void)system;
  (void)solution;
  (void)stream;
  throw std::runtime_error("CudssLinearSolver::solveSpd requires cuDSS");
#else
  ValidateSystemForSolve(system, solution);

  const int rows = system.rows();
  solution->resize(static_cast<size_t>(rows));

  CudssHandle handle;
  GTSAM_CUDSS_CHECK(cudssSetStream(handle.value, stream));
  CudssConfig config;
  CudssData data(handle.value);

  CudssMatrix matrix;
  cudssStatus_t csrStatus = cudssMatrixCreateCsr(
      &matrix.value, rows, rows, system.nonzeros(), system.rowPointers().data(),
      system.rowPointers().data() + 1, system.colIndices().data(),
      system.values().data(), CUDSS_R_32I, CUDSS_R_32I, CUDSS_R_64F,
      CUDSS_MTYPE_SPD, CUDSS_MVIEW_UPPER, CUDSS_BASE_ZERO);
  if (csrStatus == CUDSS_STATUS_NOT_SUPPORTED) {
    matrix.reset();
    // cuDSS 0.8 accepts standard CSR offsets with rowEnd == nullptr.
    csrStatus = cudssMatrixCreateCsr(
        &matrix.value, rows, rows, system.nonzeros(),
        system.rowPointers().data(), nullptr, system.colIndices().data(),
        system.values().data(), CUDSS_R_32I, CUDSS_R_32I, CUDSS_R_64F,
        CUDSS_MTYPE_SPD, CUDSS_MVIEW_UPPER, CUDSS_BASE_ZERO);
  }
  CheckCudss(csrStatus, "cudssMatrixCreateCsr");

  CudssMatrix x;
  GTSAM_CUDSS_CHECK(cudssMatrixCreateDn(&x.value, rows, 1, rows,
                                        solution->data(), CUDSS_R_64F,
                                        CUDSS_LAYOUT_COL_MAJOR));

  CudssMatrix b;
  GTSAM_CUDSS_CHECK(cudssMatrixCreateDn(&b.value, rows, 1, rows,
                                        system.rhs().data(), CUDSS_R_64F,
                                        CUDSS_LAYOUT_COL_MAJOR));

  CheckCudssExecute(cudssExecute(handle.value, CUDSS_PHASE_ANALYSIS,
                                 config.value, data.value, matrix.value,
                                 x.value, b.value),
                    CUDSS_PHASE_ANALYSIS);
  CheckCudssExecute(cudssExecute(handle.value, CUDSS_PHASE_FACTORIZATION,
                                 config.value, data.value, matrix.value,
                                 x.value, b.value),
                    CUDSS_PHASE_FACTORIZATION);
  CheckCudssExecute(cudssExecute(handle.value, CUDSS_PHASE_SOLVE, config.value,
                                 data.value, matrix.value, x.value, b.value),
                    CUDSS_PHASE_SOLVE);
#endif
}

}  // namespace gtsam::cuda

#endif
