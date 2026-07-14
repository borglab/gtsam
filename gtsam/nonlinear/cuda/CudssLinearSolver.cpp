#if GTSAM_ENABLE_CUDA

#include <gtsam/nonlinear/cuda/CudssLinearSolver.h>

#include <chrono>
#include <memory>
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

cudssStatus_t CreateSpdCsrMatrix(CudssMatrix* matrix,
                                 const DeviceSparseNormalEquations& system) {
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

  Impl() : data(handle.value) {}

  void analyze(const DeviceSparseNormalEquations& system,
               CudaDeviceArray<double>* solutionArray,
               cudaStream_t stream) {
    ValidateSystemForSolve(system, solutionArray);

    rows = system.rows();
    nonzeros = system.nonzeros();
    solutionArray->resize(static_cast<size_t>(rows));

    GTSAM_CUDSS_CHECK(cudssSetStream(handle.value, stream));

    matrix.reset();
    x.reset();
    b.reset();

    CheckCudss(CreateSpdCsrMatrix(&matrix, system), "cudssMatrixCreateCsr");
    GTSAM_CUDSS_CHECK(cudssMatrixCreateDn(
        &x.value, rows, 1, rows, solutionArray->data(), CUDSS_R_64F,
        CUDSS_LAYOUT_COL_MAJOR));
    GTSAM_CUDSS_CHECK(cudssMatrixCreateDn(
        &b.value, rows, 1, rows, system.rhs().data(), CUDSS_R_64F,
        CUDSS_LAYOUT_COL_MAJOR));

    CheckCudssExecute(cudssExecute(handle.value, CUDSS_PHASE_ANALYSIS,
                                   config.value, data.value, matrix.value,
                                   x.value, b.value),
                      CUDSS_PHASE_ANALYSIS);

    rowPointers = system.rowPointers().data();
    colIndices = system.colIndices().data();
    values = system.values().data();
    rhs = system.rhs().data();
    solution = solutionArray->data();
    analyzed = true;
  }

  void solve(const DeviceSparseNormalEquations& system,
             CudaDeviceArray<double>* solutionArray,
             cudaStream_t stream, CudssSpdSolveProfile* profile) {
    ValidateSystemForSolve(system, solutionArray);
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
    CheckCudssExecute(cudssExecute(handle.value, CUDSS_PHASE_FACTORIZATION,
                                   config.value, data.value, matrix.value,
                                   x.value, b.value),
                      CUDSS_PHASE_FACTORIZATION);

    int info = 0;
    size_t bytesWritten = 0;
    cudssStatus_t dataInfoStatus;
    if (profile) {
      const auto start = std::chrono::steady_clock::now();
      dataInfoStatus =
          cudssDataGet(handle.value, data.value, CUDSS_DATA_INFO, &info,
                       sizeof(info), &bytesWritten);
      profile->dataInfoBoundaryWall +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        start)
              .count();
    } else {
      dataInfoStatus =
          cudssDataGet(handle.value, data.value, CUDSS_DATA_INFO, &info,
                       sizeof(info), &bytesWritten);
    }
    CheckCudss(dataInfoStatus, "cudssDataGet(CUDSS_DATA_INFO)");
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

    CheckCudssExecute(cudssExecute(handle.value, CUDSS_PHASE_SOLVE,
                                   config.value, data.value, matrix.value,
                                   x.value, b.value),
                      CUDSS_PHASE_SOLVE);
  }
#endif
};

CudssSpdSolver::CudssSpdSolver() : impl_(std::make_unique<Impl>()) {}

CudssSpdSolver::~CudssSpdSolver() = default;

CudssSpdSolver::CudssSpdSolver(CudssSpdSolver&&) noexcept = default;

CudssSpdSolver& CudssSpdSolver::operator=(CudssSpdSolver&&) noexcept =
    default;

void CudssSpdSolver::analyze(const DeviceSparseNormalEquations& system,
                             CudaDeviceArray<double>* solution,
                             cudaStream_t stream) {
#if !GTSAM_ENABLE_CUDSS
  (void)system;
  (void)solution;
  (void)stream;
  throw std::runtime_error("CudssSpdSolver::analyze requires cuDSS");
#else
  impl_ = std::make_unique<Impl>();
  impl_->analyze(system, solution, stream);
#endif
}

void CudssSpdSolver::solve(const DeviceSparseNormalEquations& system,
                           CudaDeviceArray<double>* solution,
                           cudaStream_t stream,
                           CudssSpdSolveProfile* profile) {
#if !GTSAM_ENABLE_CUDSS
  (void)system;
  (void)solution;
  (void)stream;
  (void)profile;
  throw std::runtime_error("CudssSpdSolver::solve requires cuDSS");
#else
  impl_->solve(system, solution, stream, profile);
#endif
}

void CudssLinearSolver::solveSpd(const DeviceSparseNormalEquations& system,
                                 CudaDeviceArray<double>* solution,
                                 cudaStream_t stream) const {
#if !GTSAM_ENABLE_CUDSS
  (void)system;
  (void)solution;
  (void)stream;
  throw std::runtime_error("CudssLinearSolver::solveSpd requires cuDSS");
#else
  CudssSpdSolver solver;
  solver.analyze(system, solution, stream);
  solver.solve(system, solution, stream);
#endif
}

}  // namespace gtsam::cuda

#endif
