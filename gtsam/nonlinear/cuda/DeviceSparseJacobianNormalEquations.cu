#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h>

#include <cusparse.h>

#include <climits>
#include <cstdint>
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

constexpr cusparseOperation_t kNoTranspose =
    CUSPARSE_OPERATION_NON_TRANSPOSE;
constexpr cusparseSpGEMMAlg_t kSpGemmAlgorithm = CUSPARSE_SPGEMM_DEFAULT;
constexpr cusparseSpMVAlg_t kSpMvAlgorithm = CUSPARSE_SPMV_ALG_DEFAULT;
constexpr cusparseCsr2CscAlg_t kCsr2CscAlgorithm =
    CUSPARSE_CSR2CSC_ALG_DEFAULT;
constexpr double kAlpha = 1.0;
constexpr double kBeta = 0.0;

void CheckCuda(cudaError_t status, const char* stage) {
  if (status == cudaSuccess) return;
  std::ostringstream message;
  message << "CUDA failure during " << stage << ": "
          << cudaGetErrorName(status) << " (" << cudaGetErrorString(status)
          << ")";
  throw std::runtime_error(message.str());
}

void CheckCusparse(cusparseStatus_t status, const char* stage) {
  if (status == CUSPARSE_STATUS_SUCCESS) return;
  const char* name = cusparseGetErrorName(status);
  const char* detail = cusparseGetErrorString(status);
  std::ostringstream message;
  message << "cuSPARSE failure during " << stage << ": "
          << (name ? name : "unknown status") << " ("
          << (detail ? detail : "no detail") << ")";
  throw std::runtime_error(message.str());
}

void ValidatePositivePlan(const SparseJacobianPlan& plan) {
  if (plan.rows() <= 0 || plan.columns() <= 0 || plan.nonzeros() <= 0) {
    throw std::invalid_argument(
        "DeviceSparseJacobianNormalEquations positive rows, columns, and "
        "nonzeros are required");
  }
}

void ValidateDiscoveredPattern(int rows, int expectedNonzeros,
                               const std::vector<int>& rowPointers,
                               const std::vector<int>& columnIndices) {
  if (rowPointers.size() != static_cast<size_t>(rows) + 1 ||
      columnIndices.size() != static_cast<size_t>(expectedNonzeros) ||
      rowPointers.front() != 0 || rowPointers.back() != expectedNonzeros) {
    throw std::runtime_error(
        "cuSPARSE SpGEMM discovery returned an invalid CSR size");
  }

  for (int row = 0; row < rows; ++row) {
    const int begin = rowPointers[static_cast<size_t>(row)];
    const int end = rowPointers[static_cast<size_t>(row + 1)];
    if (begin < 0 || end < begin || end > expectedNonzeros) {
      throw std::runtime_error(
          "cuSPARSE SpGEMM discovery returned invalid row offsets");
    }

    bool hasDiagonal = false;
    int previousColumn = -1;
    for (int index = begin; index < end; ++index) {
      const int column = columnIndices[static_cast<size_t>(index)];
      if (column < 0 || column >= rows || column <= previousColumn) {
        throw std::runtime_error(
            "cuSPARSE SpGEMM discovery returned an unsorted or invalid CSR "
            "column");
      }
      previousColumn = column;
      hasDiagonal = hasDiagonal || column == row;
    }
    if (!hasDiagonal) {
      throw std::runtime_error(
          "cuSPARSE SpGEMM discovery omitted a scalar diagonal entry");
    }
  }
}

struct DiscoveredPattern {
  std::vector<int> rowPointers;
  std::vector<int> columnIndices;
};

struct DiscoveryResources {
  cusparseSpMatDescr_t h = nullptr;
  cusparseSpGEMMDescr_t reuse = nullptr;
  CudaDeviceArray<int> rowPointers;
  CudaDeviceArray<int> columnIndices;
  CudaDeviceArray<double> values;
  CudaDeviceArray<unsigned char> buffer1;
  CudaDeviceArray<unsigned char> buffer2;
  CudaDeviceArray<unsigned char> buffer3;
  CudaDeviceArray<unsigned char> buffer4;
  CudaDeviceArray<unsigned char> buffer5;

  ~DiscoveryResources() {
    if (reuse) cusparseSpGEMM_destroyDescr(reuse);
    if (h) cusparseDestroySpMat(h);
  }
};

}  // namespace

struct DeviceSparseJacobianNormalEquations::Impl {
  cudaStream_t stream = nullptr;
  int jacobianRows = 0;
  int jacobianColumns = 0;
  int jacobianNonzeros = 0;

  cusparseHandle_t handle = nullptr;
  cusparseSpMatDescr_t jDescriptor = nullptr;
  cusparseSpMatDescr_t jtDescriptor = nullptr;
  cusparseSpMatDescr_t hDescriptor = nullptr;
  cusparseDnVecDescr_t bDescriptor = nullptr;
  cusparseDnVecDescr_t gDescriptor = nullptr;
  cusparseSpGEMMDescr_t reuseDescriptor = nullptr;

  CudaDeviceArray<int> jRowPointers;
  CudaDeviceArray<int> jColumnIndices;
  CudaDeviceArray<double> jValues;
  CudaDeviceArray<double> b;
  CudaDeviceArray<int> jtRowPointers;
  CudaDeviceArray<int> jtColumnIndices;
  CudaDeviceArray<double> jtValues;
  DeviceSparseNormalEquations normalEquations;

  CudaDeviceArray<unsigned char> csr2cscBuffer;
  CudaDeviceArray<unsigned char> spmvBuffer;
  CudaDeviceArray<unsigned char> reuseBuffer1;
  CudaDeviceArray<unsigned char> reuseBuffer2;
  CudaDeviceArray<unsigned char> reuseBuffer3;
  CudaDeviceArray<unsigned char> reuseBuffer4;
  CudaDeviceArray<unsigned char> reuseBuffer5;

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
    const unsigned char* csr2cscBuffer = nullptr;
    const unsigned char* spmvBuffer = nullptr;
    const unsigned char* reuseBuffer4 = nullptr;
    const unsigned char* reuseBuffer5 = nullptr;
  } pointers;

  explicit Impl(cudaStream_t fixedStream) : stream(fixedStream) {}

  ~Impl() {
    // cuSPARSE work is asynchronous. Keep every descriptor, workspace, and
    // backing allocation alive until the fixed stream has stopped using it.
    cudaStreamSynchronize(stream);
    if (reuseDescriptor) cusparseSpGEMM_destroyDescr(reuseDescriptor);
    if (gDescriptor) cusparseDestroyDnVec(gDescriptor);
    if (bDescriptor) cusparseDestroyDnVec(bDescriptor);
    if (hDescriptor) cusparseDestroySpMat(hDescriptor);
    if (jtDescriptor) cusparseDestroySpMat(jtDescriptor);
    if (jDescriptor) cusparseDestroySpMat(jDescriptor);
    if (handle) cusparseDestroy(handle);
  }

  void synchronizeNoThrow() const noexcept { cudaStreamSynchronize(stream); }

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
    pointers.g = normalEquations.rhs().data();
    pointers.csr2cscBuffer = csr2cscBuffer.data();
    pointers.spmvBuffer = spmvBuffer.data();
    pointers.reuseBuffer4 = reuseBuffer4.data();
    pointers.reuseBuffer5 = reuseBuffer5.data();
  }

  void validatePointers() const {
    const bool sizesMatch =
        jRowPointers.size() == static_cast<size_t>(jacobianRows) + 1 &&
        jColumnIndices.size() == static_cast<size_t>(jacobianNonzeros) &&
        jValues.size() == static_cast<size_t>(jacobianNonzeros) &&
        b.size() == static_cast<size_t>(jacobianRows) &&
        jtRowPointers.size() == static_cast<size_t>(jacobianColumns) + 1 &&
        jtColumnIndices.size() == static_cast<size_t>(jacobianNonzeros) &&
        jtValues.size() == static_cast<size_t>(jacobianNonzeros) &&
        normalEquations.rows() == jacobianColumns;
    const bool pointersMatch =
        pointers.jRowPointers == jRowPointers.data() &&
        pointers.jColumnIndices == jColumnIndices.data() &&
        pointers.jValues == jValues.data() && pointers.b == b.data() &&
        pointers.jtRowPointers == jtRowPointers.data() &&
        pointers.jtColumnIndices == jtColumnIndices.data() &&
        pointers.jtValues == jtValues.data() &&
        pointers.hRowPointers == normalEquations.rowPointers().data() &&
        pointers.hColumnIndices == normalEquations.colIndices().data() &&
        pointers.hValues == normalEquations.values().data() &&
        pointers.g == normalEquations.rhs().data() &&
        pointers.csr2cscBuffer == csr2cscBuffer.data() &&
        pointers.spmvBuffer == spmvBuffer.data() &&
        pointers.reuseBuffer4 == reuseBuffer4.data() &&
        pointers.reuseBuffer5 == reuseBuffer5.data();
    if (!sizesMatch || !pointersMatch) {
      throw std::runtime_error(
          "DeviceSparseJacobianNormalEquations persistent storage changed");
    }
  }

  void createJacobianStorage(const SparseJacobianPlan& plan) {
    jacobianRows = plan.rows();
    jacobianColumns = plan.columns();
    jacobianNonzeros = plan.nonzeros();

    jRowPointers.upload(plan.rowPointers(), stream);
    jColumnIndices.upload(plan.columnIndices(), stream);
    jValues.resize(static_cast<size_t>(jacobianNonzeros));
    jValues.zero(stream);
    b.resize(static_cast<size_t>(jacobianRows));
    b.zero(stream);
    jtRowPointers.resize(static_cast<size_t>(jacobianColumns) + 1);
    jtColumnIndices.resize(static_cast<size_t>(jacobianNonzeros));
    jtValues.resize(static_cast<size_t>(jacobianNonzeros));

    size_t csr2cscBytes = 0;
    CheckCusparse(
        cusparseCsr2cscEx2_bufferSize(
            handle, jacobianRows, jacobianColumns, jacobianNonzeros,
            jValues.data(), jRowPointers.data(), jColumnIndices.data(),
            jtValues.data(), jtRowPointers.data(), jtColumnIndices.data(),
            CUDA_R_64F, CUSPARSE_ACTION_NUMERIC, CUSPARSE_INDEX_BASE_ZERO,
            kCsr2CscAlgorithm, &csr2cscBytes),
        "CSR-to-CSC workspace query");
    csr2cscBuffer.resize(csr2cscBytes);
    transposeJacobian();

    CheckCusparse(
        cusparseCreateCsr(
            &jDescriptor, jacobianRows, jacobianColumns, jacobianNonzeros,
            jRowPointers.data(), jColumnIndices.data(), jValues.data(),
            CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
            CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F),
        "J descriptor creation");
    CheckCusparse(
        cusparseCreateCsr(
            &jtDescriptor, jacobianColumns, jacobianRows, jacobianNonzeros,
            jtRowPointers.data(), jtColumnIndices.data(), jtValues.data(),
            CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
            CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F),
        "JT descriptor creation");
  }

  void transposeJacobian() {
    CheckCusparse(
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
  void upload(const HostSparseJacobian& host, cudaStream_t suppliedStream);
  void form(cudaStream_t suppliedStream);
};

DiscoveredPattern
DeviceSparseJacobianNormalEquations::Impl::discoverNormalPattern() {
#if GTSAM_CUSPARSE_HAS_SPGEMM_REUSE
  DiscoveryResources discovery;
  DiscoveredPattern pattern;
  try {
    discovery.rowPointers.resize(static_cast<size_t>(jacobianColumns) + 1);
    discovery.rowPointers.zero(stream);
    CheckCusparse(
        cusparseCreateCsr(
            &discovery.h, jacobianColumns, jacobianColumns, 0,
            discovery.rowPointers.data(), nullptr, nullptr,
            CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
            CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F),
        "discovery H descriptor creation");
    CheckCusparse(cusparseSpGEMM_createDescr(&discovery.reuse),
                  "discovery reuse descriptor creation");

    size_t bufferSize1 = 0;
    CheckCusparse(
        cusparseSpGEMMreuse_workEstimation(
            handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
            discovery.h, kSpGemmAlgorithm, discovery.reuse, &bufferSize1,
            nullptr),
        "discovery reuse work-estimation query");
    discovery.buffer1.resize(bufferSize1);
    CheckCusparse(
        cusparseSpGEMMreuse_workEstimation(
            handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
            discovery.h, kSpGemmAlgorithm, discovery.reuse, &bufferSize1,
            discovery.buffer1.data()),
        "discovery reuse work-estimation execute");

    size_t bufferSize2 = 0;
    size_t bufferSize3 = 0;
    size_t bufferSize4 = 0;
    CheckCusparse(
        cusparseSpGEMMreuse_nnz(
            handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
            discovery.h, kSpGemmAlgorithm, discovery.reuse, &bufferSize2,
            nullptr, &bufferSize3, nullptr, &bufferSize4, nullptr),
        "discovery reuse-nnz workspace query");
    discovery.buffer2.resize(bufferSize2);
    discovery.buffer3.resize(bufferSize3);
    discovery.buffer4.resize(bufferSize4);
    CheckCusparse(
        cusparseSpGEMMreuse_nnz(
            handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
            discovery.h, kSpGemmAlgorithm, discovery.reuse, &bufferSize2,
            discovery.buffer2.data(), &bufferSize3,
            discovery.buffer3.data(), &bufferSize4,
            discovery.buffer4.data()),
        "discovery reuse-nnz execute");

    int64_t discoveredRows = 0;
    int64_t discoveredColumns = 0;
    int64_t discoveredNonzeros = 0;
    CheckCusparse(
        cusparseSpMatGetSize(discovery.h, &discoveredRows,
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
    CheckCusparse(
        cusparseCsrSetPointers(
            discovery.h, discovery.rowPointers.data(),
            discovery.columnIndices.data(), discovery.values.data()),
        "discovery H pointer binding before reuse-copy");

    size_t bufferSize5 = 0;
    CheckCusparse(
        cusparseSpGEMMreuse_copy(
            handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
            discovery.h, kSpGemmAlgorithm, discovery.reuse, &bufferSize5,
            nullptr),
        "discovery reuse-copy workspace query");
    discovery.buffer5.resize(bufferSize5);
    CheckCusparse(
        cusparseSpGEMMreuse_copy(
            handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
            discovery.h, kSpGemmAlgorithm, discovery.reuse, &bufferSize5,
            discovery.buffer5.data()),
        "discovery reuse-copy execute");

    pattern.rowPointers.resize(static_cast<size_t>(jacobianColumns) + 1);
    pattern.columnIndices.resize(static_cast<size_t>(hNonzeros));
    CheckCuda(cudaMemcpyAsync(
                  pattern.rowPointers.data(), discovery.rowPointers.data(),
                  sizeof(int) * pattern.rowPointers.size(),
                  cudaMemcpyDeviceToHost, stream),
              "discovery H row-offset download");
    CheckCuda(cudaMemcpyAsync(
                  pattern.columnIndices.data(),
                  discovery.columnIndices.data(),
                  sizeof(int) * pattern.columnIndices.size(),
                  cudaMemcpyDeviceToHost, stream),
              "discovery H column-index download");
    CheckCuda(cudaStreamSynchronize(stream),
              "discovery H pattern download synchronization");
    ValidateDiscoveredPattern(jacobianColumns, hNonzeros,
                              pattern.rowPointers, pattern.columnIndices);
    return pattern;
  } catch (...) {
    // Locals are declared outside the try and remain alive for this sync.
    synchronizeNoThrow();
    throw;
  }
#else
  throw std::runtime_error(
      "cuSPARSE SpGEMMreuse support was not configured");
#endif
}

void DeviceSparseJacobianNormalEquations::Impl::createStableNormalStorage(
    const DiscoveredPattern& discovered) {
#if GTSAM_CUSPARSE_HAS_SPGEMM_REUSE
  normalEquations.uploadPattern(jacobianColumns, discovered.rowPointers,
                                discovered.columnIndices, stream);
  // beta is zero, but NVIDIA's reuse sample initializes C before binding it.
  normalEquations.zero(stream);

  CheckCusparse(
      cusparseCreateCsr(
          &hDescriptor, jacobianColumns, jacobianColumns, 0,
          const_cast<int*>(normalEquations.rowPointers().data()), nullptr,
          nullptr,
          CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
          CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F),
      "stable H descriptor creation");
  CheckCusparse(cusparseSpGEMM_createDescr(&reuseDescriptor),
                "stable reuse descriptor creation");

  size_t bufferSize1 = 0;
  CheckCusparse(
      cusparseSpGEMMreuse_workEstimation(
          handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
          hDescriptor, kSpGemmAlgorithm, reuseDescriptor, &bufferSize1,
          nullptr),
      "stable reuse work-estimation query");
  reuseBuffer1.resize(bufferSize1);
  CheckCusparse(
      cusparseSpGEMMreuse_workEstimation(
          handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
          hDescriptor, kSpGemmAlgorithm, reuseDescriptor, &bufferSize1,
          reuseBuffer1.data()),
      "stable reuse work-estimation execute");

  size_t bufferSize2 = 0;
  size_t bufferSize3 = 0;
  size_t bufferSize4 = 0;
  CheckCusparse(
      cusparseSpGEMMreuse_nnz(
          handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
          hDescriptor, kSpGemmAlgorithm, reuseDescriptor, &bufferSize2,
          nullptr, &bufferSize3, nullptr, &bufferSize4, nullptr),
      "stable reuse-nnz workspace query");
  reuseBuffer2.resize(bufferSize2);
  reuseBuffer3.resize(bufferSize3);
  reuseBuffer4.resize(bufferSize4);
  CheckCusparse(
      cusparseSpGEMMreuse_nnz(
          handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
          hDescriptor, kSpGemmAlgorithm, reuseDescriptor, &bufferSize2,
          reuseBuffer2.data(), &bufferSize3, reuseBuffer3.data(),
          &bufferSize4, reuseBuffer4.data()),
      "stable reuse-nnz execute");

  int64_t stableRows = 0;
  int64_t stableColumns = 0;
  int64_t stableNonzeros = 0;
  CheckCusparse(
      cusparseSpMatGetSize(hDescriptor, &stableRows, &stableColumns,
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
  CheckCusparse(
      cusparseCsrSetPointers(
          hDescriptor,
          const_cast<int*>(normalEquations.rowPointers().data()),
          const_cast<int*>(normalEquations.colIndices().data()),
          normalEquations.values().data()),
      "stable H pointer binding before reuse-copy");

  size_t bufferSize5 = 0;
  CheckCusparse(
      cusparseSpGEMMreuse_copy(
          handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
          hDescriptor, kSpGemmAlgorithm, reuseDescriptor, &bufferSize5,
          nullptr),
      "stable reuse-copy workspace query");
  reuseBuffer5.resize(bufferSize5);
  CheckCusparse(
      cusparseSpGEMMreuse_copy(
          handle, kNoTranspose, kNoTranspose, jtDescriptor, jDescriptor,
          hDescriptor, kSpGemmAlgorithm, reuseDescriptor, &bufferSize5,
          reuseBuffer5.data()),
      "stable reuse-copy execute");

  std::vector<int> stableRowPointers;
  std::vector<int> stableColumnIndices;
  stableRowPointers.resize(static_cast<size_t>(jacobianColumns) + 1);
  stableColumnIndices.resize(discovered.columnIndices.size());
  try {
    CheckCuda(cudaMemcpyAsync(
                  stableRowPointers.data(),
                  normalEquations.rowPointers().data(),
                  sizeof(int) * stableRowPointers.size(),
                  cudaMemcpyDeviceToHost, stream),
              "stable H row-offset download");
    CheckCuda(cudaMemcpyAsync(
                  stableColumnIndices.data(),
                  normalEquations.colIndices().data(),
                  sizeof(int) * stableColumnIndices.size(),
                  cudaMemcpyDeviceToHost, stream),
              "stable H column-index download");
    CheckCuda(cudaStreamSynchronize(stream),
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
  throw std::runtime_error(
      "cuSPARSE SpGEMMreuse support was not configured");
#endif
}

void DeviceSparseJacobianNormalEquations::Impl::createSpmvResources() {
  CheckCusparse(
      cusparseCreateDnVec(&bDescriptor, jacobianRows, b.data(), CUDA_R_64F),
      "b dense-vector descriptor creation");
  CheckCusparse(
      cusparseCreateDnVec(&gDescriptor, jacobianColumns,
                          normalEquations.rhs().data(), CUDA_R_64F),
      "g dense-vector descriptor creation");

  size_t spmvBytes = 0;
  CheckCusparse(
      cusparseSpMV_bufferSize(
          handle, kNoTranspose, &kAlpha, jtDescriptor, bDescriptor, &kBeta,
          gDescriptor, CUDA_R_64F, kSpMvAlgorithm, &spmvBytes),
      "JT*b SpMV workspace query");
  spmvBuffer.resize(spmvBytes);
}

void DeviceSparseJacobianNormalEquations::Impl::setup(
    const SparseJacobianPlan& plan) {
  try {
    CheckCusparse(cusparseCreate(&handle), "handle creation");
    CheckCusparse(cusparseSetStream(handle, stream), "fixed stream binding");
    CheckCusparse(cusparseSetPointerMode(handle, CUSPARSE_POINTER_MODE_HOST),
                  "host pointer-mode selection");

    createJacobianStorage(plan);
    const DiscoveredPattern discovered = discoverNormalPattern();
    createStableNormalStorage(discovered);
    createSpmvResources();
    capturePointers();
    validatePointers();
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
  if (host.valuesSize() != static_cast<size_t>(jacobianNonzeros) ||
      host.rhsSize() != static_cast<size_t>(jacobianRows)) {
    throw std::invalid_argument(
        "DeviceSparseJacobianNormalEquations host numerical sizes do not "
        "match the initialized plan");
  }

  try {
    CheckCuda(cudaMemcpyAsync(jValues.data(), host.valuesData(),
                              sizeof(double) * host.valuesSize(),
                              cudaMemcpyHostToDevice, stream),
              "J values upload");
    CheckCuda(cudaMemcpyAsync(b.data(), host.rhsData(),
                              sizeof(double) * host.rhsSize(),
                              cudaMemcpyHostToDevice, stream),
              "b upload");
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
  try {
    transposeJacobian();
#if GTSAM_CUSPARSE_HAS_SPGEMM_REUSE
    CheckCusparse(
        cusparseSpGEMMreuse_compute(
            handle, kNoTranspose, kNoTranspose, &kAlpha, jtDescriptor,
            jDescriptor, &kBeta, hDescriptor, CUDA_R_64F, kSpGemmAlgorithm,
            reuseDescriptor),
        "stable H reuse-compute");
#else
    throw std::runtime_error(
        "cuSPARSE SpGEMMreuse support was not configured");
#endif
    CheckCusparse(
        cusparseSpMV(handle, kNoTranspose, &kAlpha, jtDescriptor, bDescriptor,
                     &kBeta, gDescriptor, CUDA_R_64F, kSpMvAlgorithm,
                     spmvBuffer.data()),
        "JT*b SpMV");
  } catch (...) {
    synchronizeNoThrow();
    throw;
  }
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

DeviceSparseNormalEquationCapability
DeviceSparseJacobianNormalEquations::preflightCapability() {
#if GTSAM_CUSPARSE_HAS_SPGEMM_REUSE
  return {true,
          "configured cuSPARSE provides persistent SpGEMMreuse support"};
#else
  return {false,
          "configured cuSPARSE headers and library do not expose "
          "cusparseSpGEMMreuse_workEstimation"};
#endif
}

void DeviceSparseJacobianNormalEquations::initialize(
    const SparseJacobianPlan& plan, cudaStream_t stream) {
  // Validate the host plan before capability checks or any CUDA call.
  ValidatePositivePlan(plan);
  const DeviceSparseNormalEquationCapability capability =
      preflightCapability();
  if (!capability.supported) {
    throw std::runtime_error(capability.detail);
  }

  auto replacement = std::make_unique<Impl>(stream);
  replacement->setup(plan);
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

const DeviceSparseNormalEquations&
DeviceSparseJacobianNormalEquations::system() const {
  if (!impl_) {
    throw std::logic_error(
        "DeviceSparseJacobianNormalEquations is not initialized");
  }
  return impl_->normalEquations;
}

}  // namespace gtsam::cuda
