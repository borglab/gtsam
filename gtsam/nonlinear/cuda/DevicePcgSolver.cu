#include <cuda_runtime_api.h>
#include <cusparse.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/base/cuda/CudaPinnedHostArray.h>
#include <gtsam/nonlinear/cuda/DevicePcgSolver.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtsam::cuda {
namespace {

constexpr int kPcgBlockSize = 256;
constexpr int kMaxBlockWidth = 16;
constexpr double kPivotRelativeTolerance = 1e-14;
constexpr double kTinyDiagonal = 1e-300;
constexpr int kResidualRefreshInterval = 100;
constexpr double kAlphaOne = 1.0;
constexpr double kBetaZero = 0.0;

// Device scalar-slot indices.
enum PcgSlot {
  kRzOld = 0,
  kRzNew,
  kPAp,
  kRr,
  kGg,
  kGx,
  kTheta,
  kAlpha,
  kBeta,
  kLambda,
  kSlotCount,
};

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

int GridSize(int count) {
  return count / kPcgBlockSize + (count % kPcgBlockSize != 0);
}

__device__ int PackedIndex(int i, int j) {  // requires i >= j
  return i * (i + 1) / 2 + j;
}

__device__ double BlockReduceSum(double value) {
  __shared__ double shared[kPcgBlockSize / 32];
  const int lane = threadIdx.x % 32;
  const int warp = threadIdx.x / 32;
  for (int offset = 16; offset > 0; offset /= 2) {
    value += __shfl_down_sync(0xffffffff, value, offset);
  }
  if (lane == 0) shared[warp] = value;
  __syncthreads();
  value = threadIdx.x < blockDim.x / 32 ? shared[lane] : 0.0;
  if (warp == 0) {
    for (int offset = 16; offset > 0; offset /= 2) {
      value += __shfl_down_sync(0xffffffff, value, offset);
    }
  }
  return value;
}

__global__ void DotKernel(const double* a, const double* b, int n,
                          double* out) {
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
       i += gridDim.x * blockDim.x) {
    local += a[i] * b[i];
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(out, total);
}

__global__ void ColumnSquaredNormsKernel(const int* jtRowPointers,
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

/**
 * One warp per variable block. The sparse plan guarantees the w columns of a
 * variable share an identical residual-row pattern, so the variable's Jᵀ
 * value data is one contiguous w-by-nnzPerColumn chunk and the Gram block is
 * the sum of outer products of its w-wide slices.
 */
__global__ void BuildGramBlocksKernel(const int* columnBegins, int count,
                                      int width, const int* jtRowPointers,
                                      const double* jtValues,
                                      double* gramBlocks) {
  const int warpsPerBlock = blockDim.x / 32;
  const int variable = blockIdx.x * warpsPerBlock + threadIdx.x / 32;
  if (variable >= count) return;
  const int lane = threadIdx.x % 32;
  const int packedCount = width * (width + 1) / 2;

  const int firstColumn = columnBegins[variable];
  const int base = jtRowPointers[firstColumn];
  const int nnzPerColumn = jtRowPointers[firstColumn + 1] - base;

  double accumulators[kMaxBlockWidth * (kMaxBlockWidth + 1) / 2];
  for (int entry = 0; entry < packedCount; ++entry) accumulators[entry] = 0.0;

  double slice[kMaxBlockWidth];
  for (int j = lane; j < nnzPerColumn; j += 32) {
    for (int k = 0; k < width; ++k) {
      slice[k] = jtValues[base + k * nnzPerColumn + j];
    }
    for (int i = 0; i < width; ++i) {
      for (int k = 0; k <= i; ++k) {
        accumulators[PackedIndex(i, k)] += slice[i] * slice[k];
      }
    }
  }

  for (int entry = 0; entry < packedCount; ++entry) {
    double value = accumulators[entry];
    for (int offset = 16; offset > 0; offset /= 2) {
      value += __shfl_down_sync(0xffffffff, value, offset);
    }
    if (lane == 0) {
      gramBlocks[entry * count + variable] = value;
    }
  }
}

/**
 * One thread per variable block: damp the Gram block, Cholesky-factor it,
 * and store the explicit symmetric inverse (packed lower triangle, SoA).
 * A non-positive pivot falls back to a scalar-Jacobi inverse for that block
 * only, keeping the preconditioner SPD.
 */
__global__ void DampInvertBlocksKernel(const double* gramBlocks,
                                       const int* columnBegins, int count,
                                       int width,
                                       const double* dampingDiagonal,
                                       const double* slots,
                                       double* inverseBlocks) {
  const int variable = blockIdx.x * blockDim.x + threadIdx.x;
  if (variable >= count) return;
  const double lambda = slots[kLambda];
  const int packedCount = width * (width + 1) / 2;
  const int firstColumn = columnBegins[variable];

  double damped[kMaxBlockWidth * (kMaxBlockWidth + 1) / 2];
  for (int entry = 0; entry < packedCount; ++entry) {
    damped[entry] = gramBlocks[entry * count + variable];
  }
  double maxDiagonal = 0.0;
  for (int i = 0; i < width; ++i) {
    damped[PackedIndex(i, i)] +=
        lambda * dampingDiagonal[firstColumn + i];
    maxDiagonal = fmax(maxDiagonal, fabs(damped[PackedIndex(i, i)]));
  }

  // In-place packed Cholesky: damped becomes L.
  bool factorizationFailed = false;
  for (int j = 0; j < width && !factorizationFailed; ++j) {
    double pivot = damped[PackedIndex(j, j)];
    for (int k = 0; k < j; ++k) {
      const double ljk = damped[PackedIndex(j, k)];
      pivot -= ljk * ljk;
    }
    if (!isfinite(pivot) || pivot <= kPivotRelativeTolerance * maxDiagonal) {
      factorizationFailed = true;
      break;
    }
    const double ljj = sqrt(pivot);
    damped[PackedIndex(j, j)] = ljj;
    for (int i = j + 1; i < width; ++i) {
      double value = damped[PackedIndex(i, j)];
      for (int k = 0; k < j; ++k) {
        value -= damped[PackedIndex(i, k)] * damped[PackedIndex(j, k)];
      }
      damped[PackedIndex(i, j)] = value / ljj;
    }
  }

  if (factorizationFailed) {
    for (int entry = 0; entry < packedCount; ++entry) {
      inverseBlocks[entry * count + variable] = 0.0;
    }
    for (int i = 0; i < width; ++i) {
      const double diagonal = gramBlocks[PackedIndex(i, i) * count + variable] +
                              lambda * dampingDiagonal[firstColumn + i];
      inverseBlocks[PackedIndex(i, i) * count + variable] =
          1.0 / fmax(diagonal, kTinyDiagonal);
    }
    return;
  }

  // S = L^{-1} by forward substitution on the identity (S lower triangular).
  double inverseFactor[kMaxBlockWidth * (kMaxBlockWidth + 1) / 2];
  for (int j = 0; j < width; ++j) {
    inverseFactor[PackedIndex(j, j)] = 1.0 / damped[PackedIndex(j, j)];
    for (int i = j + 1; i < width; ++i) {
      double value = 0.0;
      for (int k = j; k < i; ++k) {
        value -= damped[PackedIndex(i, k)] * inverseFactor[PackedIndex(k, j)];
      }
      inverseFactor[PackedIndex(i, j)] = value / damped[PackedIndex(i, i)];
    }
  }

  // Minv = SᵀS, exactly symmetric by construction; packed lower triangle.
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j <= i; ++j) {
      double value = 0.0;
      for (int k = i; k < width; ++k) {
        value += inverseFactor[PackedIndex(k, i)] *
                 inverseFactor[PackedIndex(k, j)];
      }
      inverseBlocks[PackedIndex(i, j) * count + variable] = value;
    }
  }
}

/** Ablation modes: scalar Jacobi z = r/(diag + lambda*D) and identity. */
__global__ void JacobiApplyDotKernel(const double* undampedDiagonal,
                                     const double* dampingDiagonal,
                                     const double* slots, const double* r,
                                     double* z, int n, double* rzSlot) {
  const double lambda = slots[kLambda];
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
       i += gridDim.x * blockDim.x) {
    const double denom =
        fmax(undampedDiagonal[i] + lambda * dampingDiagonal[i], kTinyDiagonal);
    const double value = r[i] / denom;
    z[i] = value;
    local += r[i] * value;
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(rzSlot, total);
}

__global__ void IdentityApplyDotKernel(const double* r, double* z, int n,
                                       double* rzSlot) {
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
       i += gridDim.x * blockDim.x) {
    const double value = r[i];
    z[i] = value;
    local += value * value;
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(rzSlot, total);
}

/** z_v = Minv_v · r_v per variable block, accumulating rᵀz into a slot. */
__global__ void PrecondApplyDotKernel(const double* inverseBlocks,
                                      const int* columnBegins, int count,
                                      int width, const double* r, double* z,
                                      double* rzSlot) {
  const int variable = blockIdx.x * blockDim.x + threadIdx.x;
  double local = 0.0;
  if (variable < count) {
    const int firstColumn = columnBegins[variable];
    double rLocal[kMaxBlockWidth];
    for (int i = 0; i < width; ++i) rLocal[i] = r[firstColumn + i];
    for (int i = 0; i < width; ++i) {
      double sum = 0.0;
      for (int j = 0; j < width; ++j) {
        const int entry = i >= j ? PackedIndex(i, j) : PackedIndex(j, i);
        sum += inverseBlocks[entry * count + variable] * rLocal[j];
      }
      z[firstColumn + i] = sum;
      local += rLocal[i] * sum;
    }
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(rzSlot, total);
}

/** q += lambda·(D∘p) completing the damped operator; accumulate pᵀq. */
__global__ void DampedOpTailDotKernel(double* q, const double* p,
                                      const double* dampingDiagonal, int n,
                                      const double* slots, double* pApSlot) {
  const double lambda = slots[kLambda];
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
       i += gridDim.x * blockDim.x) {
    const double value = q[i] + lambda * dampingDiagonal[i] * p[i];
    q[i] = value;
    local += p[i] * value;
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(pApSlot, total);
}

__global__ void ScalarAlphaKernel(double* slots, int* flags) {
  const double pAp = slots[kPAp];
  const double rz = slots[kRzOld];
  if (!isfinite(pAp) || pAp <= 0.0 || !isfinite(rz) || rz <= 0.0) {
    slots[kAlpha] = 0.0;
    flags[0] = 1;
  } else {
    slots[kAlpha] = rz / pAp;
  }
  slots[kRr] = 0.0;
  slots[kRzNew] = 0.0;
}

__global__ void UpdateXRKernel(double* x, double* r, const double* p,
                               const double* q, int n, const double* slots,
                               double* rrSlot) {
  const double alpha = slots[kAlpha];
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
       i += gridDim.x * blockDim.x) {
    x[i] += alpha * p[i];
    const double residual = r[i] - alpha * q[i];
    r[i] = residual;
    local += residual * residual;
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(rrSlot, total);
}

__global__ void ScalarBetaKernel(double* slots, int* flags) {
  const double rzNew = slots[kRzNew];
  const double rzOld = slots[kRzOld];
  if (!isfinite(rzNew) || !isfinite(rzOld) || rzOld <= 0.0) {
    slots[kBeta] = 0.0;
    flags[0] = 1;
  } else {
    slots[kBeta] = rzNew / rzOld;
  }
  slots[kRzOld] = rzNew;
  slots[kRzNew] = 0.0;
  slots[kPAp] = 0.0;
}

__global__ void UpdatePKernel(double* p, const double* z, int n,
                              const double* slots) {
  const double beta = slots[kBeta];
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
       i += gridDim.x * blockDim.x) {
    p[i] = z[i] + beta * p[i];
  }
}

__global__ void ColdStartKernel(double* x, double* r, const double* g, int n,
                                double* rrSlot) {
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
       i += gridDim.x * blockDim.x) {
    x[i] = 0.0;
    const double value = g[i];
    r[i] = value;
    local += value * value;
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(rrSlot, total);
}

__global__ void ThetaKernel(double* slots) {
  const double xw = slots[kPAp];
  const double gx = slots[kGx];
  slots[kTheta] =
      (isfinite(xw) && xw > 0.0 && isfinite(gx)) ? gx / xw : 0.0;
  slots[kPAp] = 0.0;
}

/** Warm start: x = theta·x_prev (in p), r = g − theta·(A·x_prev) (in q). */
__global__ void WarmStartApplyKernel(double* x, double* r, const double* g,
                                     const double* p, const double* q, int n,
                                     const double* slots, double* rrSlot) {
  const double theta = slots[kTheta];
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
       i += gridDim.x * blockDim.x) {
    x[i] = theta * p[i];
    const double residual = g[i] - theta * q[i];
    r[i] = residual;
    local += residual * residual;
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(rrSlot, total);
}

/** True-residual refresh: r = g − q where q = A·x; accumulate rᵀr. */
__global__ void TrueResidualKernel(double* r, const double* g, const double* q,
                                   int n, double* rrSlot) {
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
       i += gridDim.x * blockDim.x) {
    const double residual = g[i] - q[i];
    r[i] = residual;
    local += residual * residual;
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(rrSlot, total);
}

__global__ void InitRzKernel(double* slots) {
  slots[kRzOld] = slots[kRzNew];
  slots[kRzNew] = 0.0;
  // The restart path bypasses ScalarBetaKernel, which normally clears the
  // pᵀAp accumulator; clear it here so the next iteration starts clean.
  slots[kPAp] = 0.0;
}

struct WidthClass {
  int width = 0;
  int count = 0;
  CudaDeviceArray<int> columnBegins;
  CudaDeviceArray<double> gramBlocks;
  CudaDeviceArray<double> inverseBlocks;
};

}  // namespace

struct DevicePcgSolver::Impl {
  cusparseHandle_t handle = nullptr;
  cusparseSpMatDescr_t j = nullptr;
  cusparseSpMatDescr_t jt = nullptr;
  const int* jtRowPointers = nullptr;
  int rows = 0;
  int columns = 0;
  DevicePcgOptions options;
  bool collectProfile = false;

  std::vector<WidthClass> widthClasses;
  // Scalar-Jacobi ablation mode: diag(JtJ) built per linearization; the
  // damping vector is borrowed from the current solve() call.
  CudaDeviceArray<double> undampedDiagonal;
  const double* currentDampingDiagonal = nullptr;
  CudaDeviceArray<double> r;
  CudaDeviceArray<double> z;
  CudaDeviceArray<double> p;
  CudaDeviceArray<double> q;
  CudaDeviceArray<double> t;  // m-length J·p scratch
  CudaDeviceArray<double> slots;
  CudaDeviceArray<int> flags;
  CudaPinnedHostArray<double> hostScalars;  // rr, gg
  CudaPinnedHostArray<int> hostFlags;

  cusparseDnVecDescr_t pDescriptor = nullptr;
  cusparseDnVecDescr_t tDescriptor = nullptr;
  cusparseDnVecDescr_t qDescriptor = nullptr;
  // Per-multiply workspaces: cusparseSpMV_preprocess stores its analysis in
  // the external buffer, so J·p and Jᵀ·t must not share one.
  CudaDeviceArray<unsigned char> spmvBufferJ;
  CudaDeviceArray<unsigned char> spmvBufferJt;
  cusparseSpMVAlg_t spmvAlgorithm = CUSPARSE_SPMV_ALG_DEFAULT;

  bool warmStartValid = false;
  DevicePcgSolveStats stats;
  DevicePcgProfile profileData;

  ~Impl() {
    if (pDescriptor || tDescriptor || qDescriptor) {
      cudaDeviceSynchronize();
    }
    if (qDescriptor) cusparseDestroyDnVec(qDescriptor);
    if (tDescriptor) cusparseDestroyDnVec(tDescriptor);
    if (pDescriptor) cusparseDestroyDnVec(pDescriptor);
  }

  void applyOperator(cudaStream_t stream) {
    // t = J·p; q = Jᵀ·t; q += lambda·D∘p with pᵀq accumulated into kPAp.
    CheckCusparse(cusparseSpMV(handle, CUSPARSE_OPERATION_NON_TRANSPOSE,
                               &kAlphaOne, j, pDescriptor, &kBetaZero,
                               tDescriptor, CUDA_R_64F, spmvAlgorithm,
                               spmvBufferJ.data()),
                  "PCG J*p SpMV");
    CheckCusparse(cusparseSpMV(handle, CUSPARSE_OPERATION_NON_TRANSPOSE,
                               &kAlphaOne, jt, tDescriptor, &kBetaZero,
                               qDescriptor, CUDA_R_64F, spmvAlgorithm,
                               spmvBufferJt.data()),
                  "PCG JT*t SpMV");
  }

  void applyDampedTail(const CudaDeviceArray<double>& dampingDiagonal,
                       cudaStream_t stream) {
    DampedOpTailDotKernel<<<GridSize(columns), kPcgBlockSize, 0, stream>>>(
        q.data(), p.data(), dampingDiagonal.data(), columns, slots.data(),
        slots.data() + kPAp);
    CheckCuda(cudaGetLastError(), "PCG damped operator tail launch");
  }

  void applyPreconditioner(const double* residual, double* preconditioned,
                           double* rzSlot, cudaStream_t stream) {
    switch (options.preconditioner) {
      case DevicePcgPreconditioner::BlockJacobi:
        for (const WidthClass& widthClass : widthClasses) {
          PrecondApplyDotKernel<<<GridSize(widthClass.count), kPcgBlockSize,
                                  0, stream>>>(
              widthClass.inverseBlocks.data(), widthClass.columnBegins.data(),
              widthClass.count, widthClass.width, residual, preconditioned,
              rzSlot);
          CheckCuda(cudaGetLastError(), "PCG preconditioner apply launch");
        }
        break;
      case DevicePcgPreconditioner::Jacobi:
        JacobiApplyDotKernel<<<GridSize(columns), kPcgBlockSize, 0, stream>>>(
            undampedDiagonal.data(), currentDampingDiagonal, slots.data(),
            residual, preconditioned, columns, rzSlot);
        CheckCuda(cudaGetLastError(), "PCG scalar-Jacobi apply launch");
        break;
      case DevicePcgPreconditioner::None:
        IdentityApplyDotKernel<<<GridSize(columns), kPcgBlockSize, 0,
                                 stream>>>(residual, preconditioned, columns,
                                           rzSlot);
        CheckCuda(cudaGetLastError(), "PCG identity apply launch");
        break;
    }
  }

  void restartRecurrence(cudaStream_t stream) {
    applyPreconditioner(r.data(), z.data(), slots.data() + kRzNew, stream);
    InitRzKernel<<<1, 1, 0, stream>>>(slots.data());
    CheckCuda(cudaGetLastError(), "PCG recurrence restart launch");
    CheckCuda(cudaMemcpyAsync(p.data(), z.data(), sizeof(double) * columns,
                              cudaMemcpyDeviceToDevice, stream),
              "PCG search-direction reset copy");
  }

  /** Compute the true residual r = g − A·x (x in delta) and its norm. */
  void refreshTrueResidual(const CudaDeviceArray<double>& gradient,
                           const CudaDeviceArray<double>& dampingDiagonal,
                           const CudaDeviceArray<double>& delta,
                           cudaStream_t stream) {
    CheckCuda(cudaMemcpyAsync(p.data(), delta.data(), sizeof(double) * columns,
                              cudaMemcpyDeviceToDevice, stream),
              "PCG true-residual x copy");
    CheckCuda(cudaMemsetAsync(slots.data() + kPAp, 0, sizeof(double), stream),
              "PCG true-residual pAp clear");
    CheckCuda(cudaMemsetAsync(slots.data() + kRr, 0, sizeof(double), stream),
              "PCG true-residual rr clear");
    applyOperator(stream);
    applyDampedTail(dampingDiagonal, stream);
    TrueResidualKernel<<<GridSize(columns), kPcgBlockSize, 0, stream>>>(
        r.data(), gradient.data(), q.data(), columns, slots.data() + kRr);
    CheckCuda(cudaGetLastError(), "PCG true-residual launch");
  }

  struct CheckResult {
    double rr = 0.0;
    double gg = 0.0;
    bool broken = false;
  };

  CheckResult downloadCheck(cudaStream_t stream) {
    CheckCuda(cudaMemcpyAsync(hostScalars.data(), slots.data() + kRr,
                              sizeof(double), cudaMemcpyDeviceToHost, stream),
              "PCG residual-norm download");
    CheckCuda(cudaMemcpyAsync(hostScalars.data() + 1, slots.data() + kGg,
                              sizeof(double), cudaMemcpyDeviceToHost, stream),
              "PCG gradient-norm download");
    CheckCuda(cudaMemcpyAsync(hostFlags.data(), flags.data(), sizeof(int),
                              cudaMemcpyDeviceToHost, stream),
              "PCG breakdown-flag download");
    CheckCuda(cudaStreamSynchronize(stream), "PCG convergence-check sync");
    CheckResult result;
    result.rr = hostScalars.data()[0];
    result.gg = hostScalars.data()[1];
    result.broken = hostFlags.data()[0] != 0;
    return result;
  }
};

DevicePcgSolver::DevicePcgSolver() : impl_(std::make_unique<Impl>()) {}

DevicePcgSolver::~DevicePcgSolver() = default;

DevicePcgSolver::DevicePcgSolver(DevicePcgSolver&&) noexcept = default;

DevicePcgSolver& DevicePcgSolver::operator=(DevicePcgSolver&&) noexcept =
    default;

void DevicePcgSolver::initialize(cusparseHandle_t handle, int rows,
                                 int columns, cusparseSpMatDescr_t j,
                                 cusparseSpMatDescr_t jt,
                                 const CudaDeviceArray<int>& jtRowPointers,
                                 const std::vector<int>& blockOffsets,
                                 const DevicePcgOptions& options,
                                 cudaStream_t stream, bool collectProfile) {
  if (rows <= 0 || columns <= 0) {
    throw std::invalid_argument(
        "DevicePcgSolver requires positive dimensions");
  }
  if (!handle || !j || !jt) {
    throw std::invalid_argument(
        "DevicePcgSolver requires cuSPARSE handle and matrix descriptors");
  }
  if (blockOffsets.size() < 2 || blockOffsets.front() != 0 ||
      blockOffsets.back() != columns) {
    throw std::invalid_argument(
        "DevicePcgSolver block offsets must start at 0 and end at columns");
  }
  for (size_t index = 1; index < blockOffsets.size(); ++index) {
    const int width = blockOffsets[index] - blockOffsets[index - 1];
    if (width <= 0) {
      throw std::invalid_argument(
          "DevicePcgSolver block offsets must be strictly ascending");
    }
    if (width > kMaxBlockWidth) {
      throw std::invalid_argument(
          "DevicePcgSolver supports variable blocks of at most width 16");
    }
  }
  if (options.relativeTolerance <= 0.0 ||
      !std::isfinite(options.relativeTolerance)) {
    throw std::invalid_argument(
        "DevicePcgSolver requires a positive finite relative tolerance");
  }
  if (options.maxIterations < 0 || options.convergenceCheckInterval <= 0) {
    throw std::invalid_argument(
        "DevicePcgSolver iteration controls must be nonnegative and the "
        "check interval positive");
  }

  auto replacement = std::make_unique<Impl>();
  replacement->handle = handle;
  replacement->j = j;
  replacement->jt = jt;
  replacement->jtRowPointers = jtRowPointers.data();
  replacement->rows = rows;
  replacement->columns = columns;
  replacement->options = options;
  if (replacement->options.maxIterations == 0) {
    replacement->options.maxIterations = std::min(columns, 250);
  }
  replacement->collectProfile = collectProfile;

  // Group variables by width class for the block kernels.
  std::map<int, std::vector<int>> beginsByWidth;
  for (size_t index = 1; index < blockOffsets.size(); ++index) {
    const int width = blockOffsets[index] - blockOffsets[index - 1];
    beginsByWidth[width].push_back(blockOffsets[index - 1]);
  }

  // The Gram kernel requires each variable's columns to share one Jᵀ row
  // pattern; the plan construction guarantees it. Verify the necessary
  // equal-count condition once from the host copy of the row pointers.
  std::vector<int> hostRowPointers(static_cast<size_t>(columns) + 1);
  CheckCuda(cudaMemcpyAsync(hostRowPointers.data(), jtRowPointers.data(),
                            sizeof(int) * hostRowPointers.size(),
                            cudaMemcpyDeviceToHost, stream),
            "PCG JT row-pointer validation download");
  CheckCuda(cudaStreamSynchronize(stream),
            "PCG JT row-pointer validation sync");
  for (size_t index = 1; index < blockOffsets.size(); ++index) {
    const int begin = blockOffsets[index - 1];
    const int end = blockOffsets[index];
    const int expected = hostRowPointers[static_cast<size_t>(begin) + 1] -
                         hostRowPointers[static_cast<size_t>(begin)];
    for (int column = begin + 1; column < end; ++column) {
      const int nnz = hostRowPointers[static_cast<size_t>(column) + 1] -
                      hostRowPointers[static_cast<size_t>(column)];
      if (nnz != expected) {
        throw std::runtime_error(
            "DevicePcgSolver variable-block columns have unequal Jacobian "
            "column patterns");
      }
    }
  }

  try {
    if (options.preconditioner == DevicePcgPreconditioner::BlockJacobi) {
      for (auto& [width, begins] : beginsByWidth) {
        WidthClass widthClass;
        widthClass.width = width;
        widthClass.count = static_cast<int>(begins.size());
        widthClass.columnBegins.upload(begins, stream);
        const size_t packedSize = static_cast<size_t>(width) * (width + 1) /
                                  2 * begins.size();
        widthClass.gramBlocks.resize(packedSize);
        widthClass.gramBlocks.zero(stream);
        widthClass.inverseBlocks.resize(packedSize);
        widthClass.inverseBlocks.zero(stream);
        replacement->widthClasses.push_back(std::move(widthClass));
      }
    } else if (options.preconditioner == DevicePcgPreconditioner::Jacobi) {
      replacement->undampedDiagonal.resize(static_cast<size_t>(columns));
      replacement->undampedDiagonal.zero(stream);
    }

    replacement->r.resize(static_cast<size_t>(columns));
    replacement->z.resize(static_cast<size_t>(columns));
    replacement->p.resize(static_cast<size_t>(columns));
    replacement->q.resize(static_cast<size_t>(columns));
    replacement->t.resize(static_cast<size_t>(rows));
    replacement->r.zero(stream);
    replacement->z.zero(stream);
    replacement->p.zero(stream);
    replacement->q.zero(stream);
    replacement->t.zero(stream);
    replacement->slots.resize(kSlotCount);
    replacement->slots.zero(stream);
    replacement->flags.resize(1);
    replacement->flags.zero(stream);
    replacement->hostScalars.resize(2);
    replacement->hostFlags.resize(1);

    CheckCusparse(cusparseCreateDnVec(&replacement->pDescriptor, columns,
                                      replacement->p.data(), CUDA_R_64F),
                  "PCG p descriptor creation");
    CheckCusparse(cusparseCreateDnVec(&replacement->tDescriptor, rows,
                                      replacement->t.data(), CUDA_R_64F),
                  "PCG t descriptor creation");
    CheckCusparse(cusparseCreateDnVec(&replacement->qDescriptor, columns,
                                      replacement->q.data(), CUDA_R_64F),
                  "PCG q descriptor creation");

    // Experiment switch: GTSAM_PCG_SPMV_ALG=alg1|alg2 overrides the default.
    if (const char* algName = std::getenv("GTSAM_PCG_SPMV_ALG")) {
      if (std::string(algName) == "alg1") {
        replacement->spmvAlgorithm = CUSPARSE_SPMV_CSR_ALG1;
      } else if (std::string(algName) == "alg2") {
        replacement->spmvAlgorithm = CUSPARSE_SPMV_CSR_ALG2;
      }
    }

    size_t forwardBytes = 0;
    CheckCusparse(
        cusparseSpMV_bufferSize(handle, CUSPARSE_OPERATION_NON_TRANSPOSE,
                                &kAlphaOne, j, replacement->pDescriptor,
                                &kBetaZero, replacement->tDescriptor,
                                CUDA_R_64F, replacement->spmvAlgorithm,
                                &forwardBytes),
        "PCG J*p SpMV workspace query");
    size_t transposeBytes = 0;
    CheckCusparse(
        cusparseSpMV_bufferSize(handle, CUSPARSE_OPERATION_NON_TRANSPOSE,
                                &kAlphaOne, jt, replacement->tDescriptor,
                                &kBetaZero, replacement->qDescriptor,
                                CUDA_R_64F, replacement->spmvAlgorithm,
                                &transposeBytes),
        "PCG JT*t SpMV workspace query");
    replacement->spmvBufferJ.resize(forwardBytes);
    replacement->spmvBufferJt.resize(transposeBytes);

    // One-time structure analysis amortized over the hundreds of SpMVs per
    // solve. Values change every linearization, but preprocess results are
    // structure-only and remain valid.
    CheckCusparse(
        cusparseSpMV_preprocess(handle, CUSPARSE_OPERATION_NON_TRANSPOSE,
                                &kAlphaOne, j, replacement->pDescriptor,
                                &kBetaZero, replacement->tDescriptor,
                                CUDA_R_64F, replacement->spmvAlgorithm,
                                replacement->spmvBufferJ.data()),
        "PCG J*p SpMV preprocess");
    CheckCusparse(
        cusparseSpMV_preprocess(handle, CUSPARSE_OPERATION_NON_TRANSPOSE,
                                &kAlphaOne, jt, replacement->tDescriptor,
                                &kBetaZero, replacement->qDescriptor,
                                CUDA_R_64F, replacement->spmvAlgorithm,
                                replacement->spmvBufferJt.data()),
        "PCG JT*t SpMV preprocess");

    CheckCuda(cudaStreamSynchronize(stream), "PCG setup synchronization");
  } catch (...) {
    cudaStreamSynchronize(stream);
    throw;
  }

  impl_ = std::move(replacement);
}

void DevicePcgSolver::buildPreconditioner(
    const CudaDeviceArray<double>& jtValues, cudaStream_t stream) {
  if (!impl_ || !impl_->handle) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  std::chrono::steady_clock::time_point start;
  if (impl_->collectProfile) start = std::chrono::steady_clock::now();

  switch (impl_->options.preconditioner) {
    case DevicePcgPreconditioner::BlockJacobi:
      for (const WidthClass& widthClass : impl_->widthClasses) {
        const int warpsPerBlock = kPcgBlockSize / 32;
        const int grid = widthClass.count / warpsPerBlock +
                         (widthClass.count % warpsPerBlock != 0);
        BuildGramBlocksKernel<<<grid, kPcgBlockSize, 0, stream>>>(
            widthClass.columnBegins.data(), widthClass.count,
            widthClass.width, impl_->jtRowPointers, jtValues.data(),
            const_cast<double*>(widthClass.gramBlocks.data()));
        CheckCuda(cudaGetLastError(), "PCG Gram-block build launch");
      }
      break;
    case DevicePcgPreconditioner::Jacobi:
      ColumnSquaredNormsKernel<<<GridSize(impl_->columns), kPcgBlockSize, 0,
                                 stream>>>(impl_->jtRowPointers,
                                           jtValues.data(), impl_->columns,
                                           impl_->undampedDiagonal.data());
      CheckCuda(cudaGetLastError(), "PCG scalar-diagonal build launch");
      break;
    case DevicePcgPreconditioner::None:
      break;
  }
  impl_->warmStartValid = false;

  if (impl_->collectProfile) {
    CheckCuda(cudaStreamSynchronize(stream),
              "PCG profiled preconditioner-build sync");
    impl_->profileData.preconditionerBuild +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
  }
}

void DevicePcgSolver::solve(double lambda,
                            const CudaDeviceArray<double>& gradient,
                            const CudaDeviceArray<double>& dampingDiagonal,
                            CudaDeviceArray<double>* delta,
                            cudaStream_t stream) {
  if (!impl_ || !impl_->handle) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  if (!delta || delta->size() != static_cast<size_t>(impl_->columns) ||
      gradient.size() != static_cast<size_t>(impl_->columns) ||
      dampingDiagonal.size() != static_cast<size_t>(impl_->columns)) {
    throw std::invalid_argument(
        "DevicePcgSolver solve storage sizes do not match the system");
  }
  if (!std::isfinite(lambda) || lambda < 0.0) {
    throw std::invalid_argument(
        "DevicePcgSolver lambda must be finite and nonnegative");
  }

  Impl& state = *impl_;
  const DevicePcgOptions& options = state.options;
  const int n = state.columns;
  std::chrono::steady_clock::time_point solveStart;
  if (state.collectProfile) solveStart = std::chrono::steady_clock::now();

  state.stats = {};
  state.currentDampingDiagonal = dampingDiagonal.data();

  try {
    state.slots.zero(stream);
    state.flags.zero(stream);
    state.hostScalars.data()[0] = lambda;
    CheckCuda(cudaMemcpyAsync(state.slots.data() + kLambda,
                              state.hostScalars.data(), sizeof(double),
                              cudaMemcpyHostToDevice, stream),
              "PCG lambda upload");

    // Damp, factor, and invert the preconditioner blocks for this lambda.
    // (Block-Jacobi only; the scalar-Jacobi ablation mode damps inline in
    // its apply kernel, and identity needs nothing.)
    if (options.preconditioner == DevicePcgPreconditioner::BlockJacobi) {
      for (const WidthClass& widthClass : state.widthClasses) {
        DampInvertBlocksKernel<<<GridSize(widthClass.count), kPcgBlockSize, 0,
                                 stream>>>(
            widthClass.gramBlocks.data(), widthClass.columnBegins.data(),
            widthClass.count, widthClass.width, dampingDiagonal.data(),
            state.slots.data(),
            const_cast<double*>(widthClass.inverseBlocks.data()));
        CheckCuda(cudaGetLastError(), "PCG damped block factorization launch");
      }
    }

    DotKernel<<<GridSize(n), kPcgBlockSize, 0, stream>>>(
        gradient.data(), gradient.data(), n, state.slots.data() + kGg);
    CheckCuda(cudaGetLastError(), "PCG gradient-norm launch");

    if (state.warmStartValid && options.warmStart) {
      // Scaled warm start: x0 = theta*x_prev minimizes the damped quadratic
      // along the previous delta, so it is never worse than a cold start.
      CheckCuda(cudaMemcpyAsync(state.p.data(), delta->data(),
                                sizeof(double) * n, cudaMemcpyDeviceToDevice,
                                stream),
                "PCG warm-start delta copy");
      state.applyOperator(stream);
      state.applyDampedTail(dampingDiagonal, stream);
      DotKernel<<<GridSize(n), kPcgBlockSize, 0, stream>>>(
          gradient.data(), state.p.data(), n, state.slots.data() + kGx);
      CheckCuda(cudaGetLastError(), "PCG warm-start gradient dot launch");
      ThetaKernel<<<1, 1, 0, stream>>>(state.slots.data());
      CheckCuda(cudaGetLastError(), "PCG warm-start theta launch");
      WarmStartApplyKernel<<<GridSize(n), kPcgBlockSize, 0, stream>>>(
          delta->data(), state.r.data(), gradient.data(), state.p.data(),
          state.q.data(), n, state.slots.data(), state.slots.data() + kRr);
      CheckCuda(cudaGetLastError(), "PCG warm-start apply launch");
    } else {
      ColdStartKernel<<<GridSize(n), kPcgBlockSize, 0, stream>>>(
          delta->data(), state.r.data(), gradient.data(), n,
          state.slots.data() + kRr);
      CheckCuda(cudaGetLastError(), "PCG cold-start launch");
    }

    state.applyPreconditioner(state.r.data(), state.z.data(),
                              state.slots.data() + kRzNew, stream);
    InitRzKernel<<<1, 1, 0, stream>>>(state.slots.data());
    CheckCuda(cudaGetLastError(), "PCG initial recurrence launch");
    CheckCuda(cudaMemcpyAsync(state.p.data(), state.z.data(),
                              sizeof(double) * n, cudaMemcpyDeviceToDevice,
                              stream),
              "PCG initial search-direction copy");

    const double toleranceSquared =
        options.relativeTolerance * options.relativeTolerance;
    double gradientNormSquared = -1.0;
    int iteration = 0;
    bool converged = false;
    bool broken = false;

    // Initial check: the warm start (or a zero gradient) may already satisfy
    // the tolerance.
    {
      const Impl::CheckResult check = state.downloadCheck(stream);
      gradientNormSquared = check.gg;
      state.stats.residualNormSquared = check.rr;
      if (!std::isfinite(check.rr)) {
        delta->zero(stream);
        CheckCuda(cudaStreamSynchronize(stream), "PCG non-finite reset sync");
        broken = true;
      } else if (check.gg <= 0.0 ||
                 check.rr <= toleranceSquared * check.gg) {
        converged = true;
        if (check.gg <= 0.0) delta->zero(stream);
      }
    }

    while (!converged && !broken && iteration < options.maxIterations) {
      state.applyOperator(stream);
      state.applyDampedTail(dampingDiagonal, stream);
      ScalarAlphaKernel<<<1, 1, 0, stream>>>(state.slots.data(),
                                             state.flags.data());
      CheckCuda(cudaGetLastError(), "PCG alpha launch");
      UpdateXRKernel<<<GridSize(n), kPcgBlockSize, 0, stream>>>(
          delta->data(), state.r.data(), state.p.data(), state.q.data(), n,
          state.slots.data(), state.slots.data() + kRr);
      CheckCuda(cudaGetLastError(), "PCG x/r update launch");
      ++iteration;

      const bool atCheck = iteration % options.convergenceCheckInterval == 0 ||
                           iteration == options.maxIterations;
      if (atCheck) {
        const Impl::CheckResult check = state.downloadCheck(stream);
        state.stats.residualNormSquared = check.rr;
        if (check.broken) {
          broken = true;
          break;
        }
        if (!std::isfinite(check.rr)) {
          delta->zero(stream);
          CheckCuda(cudaStreamSynchronize(stream),
                    "PCG non-finite reset sync");
          broken = true;
          break;
        }
        if (check.rr <= toleranceSquared * gradientNormSquared) {
          // Verify against the true residual before accepting; the
          // recurrence can drift on ill-conditioned systems.
          state.refreshTrueResidual(gradient, dampingDiagonal, *delta,
                                    stream);
          const Impl::CheckResult verified = state.downloadCheck(stream);
          state.stats.residualNormSquared = verified.rr;
          if (!std::isfinite(verified.rr)) {
            delta->zero(stream);
            CheckCuda(cudaStreamSynchronize(stream),
                      "PCG non-finite reset sync");
            broken = true;
            break;
          }
          if (verified.rr <= toleranceSquared * gradientNormSquared) {
            converged = true;
            break;
          }
          state.restartRecurrence(stream);
          continue;
        }
      }
      if (iteration % kResidualRefreshInterval == 0 && !atCheck) {
        state.refreshTrueResidual(gradient, dampingDiagonal, *delta, stream);
        state.restartRecurrence(stream);
        continue;
      }

      state.applyPreconditioner(state.r.data(), state.z.data(),
                                state.slots.data() + kRzNew, stream);
      ScalarBetaKernel<<<1, 1, 0, stream>>>(state.slots.data(),
                                            state.flags.data());
      CheckCuda(cudaGetLastError(), "PCG beta launch");
      UpdatePKernel<<<GridSize(n), kPcgBlockSize, 0, stream>>>(
          state.p.data(), state.z.data(), n, state.slots.data());
      CheckCuda(cudaGetLastError(), "PCG p update launch");
    }

    CheckCuda(cudaStreamSynchronize(stream), "PCG solve completion sync");

    state.stats.iterations = iteration;
    state.stats.gradientNormSquared = gradientNormSquared;
    state.stats.converged = converged;
    state.stats.breakdown = broken;
    state.warmStartValid = true;

    state.profileData.iterationsTotal += static_cast<size_t>(iteration);
    ++state.profileData.solveCount;
    if (!converged && !broken) ++state.profileData.maxIterationHits;
    if (state.collectProfile) {
      state.profileData.solve +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        solveStart)
              .count();
    }
  } catch (...) {
    cudaStreamSynchronize(stream);
    throw;
  }
}

const DevicePcgSolveStats& DevicePcgSolver::lastSolveStats() const {
  if (!impl_) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  return impl_->stats;
}

const DevicePcgProfile& DevicePcgSolver::profile() const {
  if (!impl_) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  return impl_->profileData;
}

}  // namespace gtsam::cuda
