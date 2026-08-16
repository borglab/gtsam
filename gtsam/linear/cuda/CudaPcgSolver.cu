#include <gtsam/linear/cuda/CudaPcgSolver.h>

#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/base/cuda/CudaPinnedHostArray.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace gtsam::cuda {
namespace {

constexpr int kBlockSize = 256;
constexpr int kResidualRefreshInterval = 100;

enum PcgSlot {
  kRzOld = 0,
  kRzNew,
  kPAp,
  kRr,
  kRhsNorm,
  kRhsSolution,
  kSolutionProduct,
  kTheta,
  kAlpha,
  kBeta,
  kSlotCount,
};

int GridSize(int count) {
  return count / kBlockSize + (count % kBlockSize != 0);
}

__device__ double BlockReduceSum(double value) {
  __shared__ double shared[kBlockSize / 32];
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

__global__ void DotKernel(const double* left, const double* right, int count,
                          double* result) {
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += gridDim.x * blockDim.x) {
    local += left[i] * right[i];
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(result, total);
}

__global__ void ColdStartKernel(double* solution, double* residual,
                                const double* rhs, int count,
                                double* residualNormSlot) {
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += gridDim.x * blockDim.x) {
    solution[i] = 0.0;
    residual[i] = rhs[i];
    local += rhs[i] * rhs[i];
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(residualNormSlot, total);
}

__global__ void ThetaKernel(double* slots) {
  const double solutionProduct = slots[kSolutionProduct];
  const double rhsSolution = slots[kRhsSolution];
  slots[kTheta] =
      isfinite(solutionProduct) && solutionProduct > 0.0 &&
              isfinite(rhsSolution)
          ? rhsSolution / solutionProduct
          : 0.0;
}

__global__ void WarmStartKernel(double* solution, double* residual,
                                const double* rhs, const double* product,
                                int count, const double* slots,
                                double* residualNormSlot) {
  const double theta = slots[kTheta];
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += gridDim.x * blockDim.x) {
    const double scaledSolution = theta * solution[i];
    solution[i] = scaledSolution;
    const double value = rhs[i] - theta * product[i];
    residual[i] = value;
    local += value * value;
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(residualNormSlot, total);
}

__global__ void InitRecurrenceKernel(double* slots) {
  slots[kRzOld] = slots[kRzNew];
  slots[kRzNew] = 0.0;
  slots[kPAp] = 0.0;
}

__global__ void ScalarAlphaKernel(double* slots, int* breakdownFlag) {
  const double pAp = slots[kPAp];
  const double rz = slots[kRzOld];
  if (!isfinite(pAp) || pAp <= 0.0 || !isfinite(rz) || rz <= 0.0) {
    slots[kAlpha] = 0.0;
    breakdownFlag[0] = 1;
  } else {
    slots[kAlpha] = rz / pAp;
  }
  slots[kRr] = 0.0;
  slots[kRzNew] = 0.0;
}

__global__ void UpdateSolutionResidualKernel(
    double* solution, double* residual, const double* direction,
    const double* product, int count, const double* slots,
    double* residualNormSlot) {
  const double alpha = slots[kAlpha];
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += gridDim.x * blockDim.x) {
    solution[i] += alpha * direction[i];
    const double value = residual[i] - alpha * product[i];
    residual[i] = value;
    local += value * value;
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(residualNormSlot, total);
}

__global__ void ScalarBetaKernel(double* slots, int* breakdownFlag) {
  const double rzNew = slots[kRzNew];
  const double rzOld = slots[kRzOld];
  if (!isfinite(rzNew) || !isfinite(rzOld) || rzOld <= 0.0) {
    slots[kBeta] = 0.0;
    breakdownFlag[0] = 1;
  } else {
    slots[kBeta] = rzNew / rzOld;
  }
  slots[kRzOld] = rzNew;
  slots[kRzNew] = 0.0;
  slots[kPAp] = 0.0;
}

__global__ void UpdateDirectionKernel(double* direction,
                                      const double* preconditioned, int count,
                                      const double* slots) {
  const double beta = slots[kBeta];
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += gridDim.x * blockDim.x) {
    direction[i] = preconditioned[i] + beta * direction[i];
  }
}

__global__ void TrueResidualKernel(double* residual, const double* rhs,
                                   const double* product, int count,
                                   double* residualNormSlot) {
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += gridDim.x * blockDim.x) {
    const double value = rhs[i] - product[i];
    residual[i] = value;
    local += value * value;
  }
  const double total = BlockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(residualNormSlot, total);
}

void CheckCuda(cudaError_t status, const char* operation) {
  if (status == cudaSuccess) return;
  std::ostringstream os;
  os << "CUDA PCG failure during " << operation << ": "
     << cudaGetErrorName(status) << " (" << cudaGetErrorString(status) << ")";
  throw std::runtime_error(os.str());
}

}  // namespace

struct CudaPcgSolver::Impl {
  int dimension = 0;
  CudaPcgOptions options;
  bool collectProfile = false;
  bool initialized = false;
  bool warmStartValid = false;
  CudaDeviceArray<double> residual;
  CudaDeviceArray<double> preconditioned;
  CudaDeviceArray<double> direction;
  CudaDeviceArray<double> product;
  CudaDeviceArray<double> slots;
  CudaDeviceArray<int> flags;
  CudaPinnedHostArray<double> hostScalars;
  CudaPinnedHostArray<int> hostFlags;
  CudaLinearSolveStats statistics;

  void dot(const double* left, const double* right, double* slot,
           cudaStream_t stream) {
    DotKernel<<<GridSize(dimension), kBlockSize, 0, stream>>>(
        left, right, dimension, slot);
    CheckCuda(cudaGetLastError(), "dot launch");
  }

  struct CheckResult {
    double residualNormSquared = 0.0;
    double rhsNormSquared = 0.0;
    bool breakdown = false;
  };

  CheckResult downloadCheck(cudaStream_t stream) {
    CheckCuda(cudaMemcpyAsync(hostScalars.data(), slots.data() + kRr,
                              sizeof(double), cudaMemcpyDeviceToHost, stream),
              "residual-norm download");
    CheckCuda(cudaMemcpyAsync(hostScalars.data() + 1,
                              slots.data() + kRhsNorm, sizeof(double),
                              cudaMemcpyDeviceToHost, stream),
              "rhs-norm download");
    CheckCuda(cudaMemcpyAsync(hostFlags.data(), flags.data(), sizeof(int),
                              cudaMemcpyDeviceToHost, stream),
              "breakdown-flag download");
    CheckCuda(cudaStreamSynchronize(stream), "convergence-check sync");
    ++statistics.pcgHostConvergenceChecks;
    return {hostScalars.data()[0], hostScalars.data()[1],
            hostFlags.data()[0] != 0};
  }

  void restart(const CudaPreconditioner& preconditioner,
               cudaStream_t stream) {
    preconditioner.apply(residual.data(), preconditioned.data(), stream);
    dot(residual.data(), preconditioned.data(), slots.data() + kRzNew,
        stream);
    InitRecurrenceKernel<<<1, 1, 0, stream>>>(slots.data());
    CheckCuda(cudaGetLastError(), "recurrence restart launch");
    CheckCuda(cudaMemcpyAsync(direction.data(), preconditioned.data(),
                              sizeof(double) * dimension,
                              cudaMemcpyDeviceToDevice, stream),
              "search-direction reset copy");
  }

  void refreshTrueResidual(const CudaLinearOperator& linearOperator,
                           const double* rhs, const double* solution,
                           cudaStream_t stream) {
    CheckCuda(cudaMemsetAsync(slots.data() + kRr, 0, sizeof(double), stream),
              "true-residual norm clear");
    linearOperator.apply(solution, product.data(), stream);
    TrueResidualKernel<<<GridSize(dimension), kBlockSize, 0, stream>>>(
        residual.data(), rhs, product.data(), dimension, slots.data() + kRr);
    CheckCuda(cudaGetLastError(), "true-residual launch");
  }
};

CudaPcgSolver::CudaPcgSolver() : impl_(std::make_unique<Impl>()) {}
CudaPcgSolver::~CudaPcgSolver() = default;
CudaPcgSolver::CudaPcgSolver(CudaPcgSolver&&) noexcept = default;
CudaPcgSolver& CudaPcgSolver::operator=(CudaPcgSolver&&) noexcept = default;

void CudaPcgSolver::initialize(int dimension, const CudaPcgOptions& options,
                               cudaStream_t stream, bool collectProfile) {
  if (dimension <= 0) {
    throw std::invalid_argument("CudaPcgSolver requires positive dimension");
  }
  if (options.maxIterations < 0 || options.convergenceCheckInterval <= 0 ||
      !std::isfinite(options.relativeTolerance) ||
      options.relativeTolerance <= 0.0) {
    throw std::invalid_argument("CudaPcgSolver has invalid iteration options");
  }
  auto replacement = std::make_unique<Impl>();
  replacement->dimension = dimension;
  replacement->options = options;
  if (replacement->options.maxIterations == 0) {
    replacement->options.maxIterations = std::min(dimension, 250);
  }
  replacement->collectProfile = collectProfile;
  replacement->residual.resize(static_cast<size_t>(dimension));
  replacement->preconditioned.resize(static_cast<size_t>(dimension));
  replacement->direction.resize(static_cast<size_t>(dimension));
  replacement->product.resize(static_cast<size_t>(dimension));
  replacement->slots.resize(kSlotCount);
  replacement->flags.resize(1);
  replacement->hostScalars.resize(2);
  replacement->hostFlags.resize(1);
  replacement->residual.zero(stream);
  replacement->preconditioned.zero(stream);
  replacement->direction.zero(stream);
  replacement->product.zero(stream);
  replacement->slots.zero(stream);
  replacement->flags.zero(stream);
  replacement->statistics.backend = CudaLinearSolverType::Pcg;
  replacement->statistics.analysisCount = 1;
  replacement->initialized = true;
  impl_ = std::move(replacement);
}

void CudaPcgSolver::solve(const CudaLinearOperator& linearOperator,
                          const CudaPreconditioner& preconditioner,
                          const double* rhs,
                          CudaDeviceArray<double>* solution,
                          cudaStream_t stream) {
  if (!impl_->initialized) {
    throw std::logic_error("CudaPcgSolver is not initialized");
  }
  Impl& state = *impl_;
  if (!rhs || !solution ||
      solution->size() != static_cast<size_t>(state.dimension)) {
    throw std::invalid_argument("CudaPcgSolver storage size mismatch");
  }
  if (linearOperator.dimension() != state.dimension ||
      preconditioner.dimension() != state.dimension) {
    throw std::invalid_argument("CudaPcgSolver operator dimension mismatch");
  }

  const auto start = std::chrono::steady_clock::now();
  try {
    state.slots.zero(stream);
    state.flags.zero(stream);
    state.dot(rhs, rhs, state.slots.data() + kRhsNorm, stream);

    if (state.warmStartValid && state.options.warmStart) {
      linearOperator.apply(solution->data(), state.product.data(), stream);
      state.dot(rhs, solution->data(), state.slots.data() + kRhsSolution,
                stream);
      state.dot(solution->data(), state.product.data(),
                state.slots.data() + kSolutionProduct, stream);
      ThetaKernel<<<1, 1, 0, stream>>>(state.slots.data());
      CheckCuda(cudaGetLastError(), "warm-start scale launch");
      WarmStartKernel<<<GridSize(state.dimension), kBlockSize, 0, stream>>>(
          solution->data(), state.residual.data(), rhs, state.product.data(),
          state.dimension, state.slots.data(), state.slots.data() + kRr);
      CheckCuda(cudaGetLastError(), "warm-start residual launch");
    } else {
      ColdStartKernel<<<GridSize(state.dimension), kBlockSize, 0, stream>>>(
          solution->data(), state.residual.data(), rhs, state.dimension,
          state.slots.data() + kRr);
      CheckCuda(cudaGetLastError(), "cold-start launch");
    }

    const double toleranceSquared = state.options.relativeTolerance *
                                    state.options.relativeTolerance;
    double rhsNormSquared = 0.0;
    double residualNormSquared = 0.0;
    bool converged = false;
    bool breakdown = false;
    int iteration = 0;

    state.restart(preconditioner, stream);
    {
      const Impl::CheckResult check = state.downloadCheck(stream);
      residualNormSquared = check.residualNormSquared;
      rhsNormSquared = check.rhsNormSquared;
      breakdown = check.breakdown || !std::isfinite(residualNormSquared) ||
                  !std::isfinite(rhsNormSquared);
      converged = !breakdown &&
                  (rhsNormSquared <= 0.0 ||
                   residualNormSquared <= toleranceSquared * rhsNormSquared);
      if (rhsNormSquared <= 0.0) solution->zero(stream);
    }

    while (!converged && !breakdown &&
           iteration < state.options.maxIterations) {
      linearOperator.apply(state.direction.data(), state.product.data(),
                           stream);
      state.dot(state.direction.data(), state.product.data(),
                state.slots.data() + kPAp, stream);
      ScalarAlphaKernel<<<1, 1, 0, stream>>>(state.slots.data(),
                                             state.flags.data());
      CheckCuda(cudaGetLastError(), "alpha launch");
      UpdateSolutionResidualKernel<<<GridSize(state.dimension), kBlockSize, 0,
                                     stream>>>(
          solution->data(), state.residual.data(), state.direction.data(),
          state.product.data(), state.dimension, state.slots.data(),
          state.slots.data() + kRr);
      CheckCuda(cudaGetLastError(), "solution/residual update launch");
      ++iteration;

      const bool atCheck =
          iteration % state.options.convergenceCheckInterval == 0 ||
          iteration == state.options.maxIterations;
      if (atCheck) {
        const Impl::CheckResult check = state.downloadCheck(stream);
        residualNormSquared = check.residualNormSquared;
        breakdown = check.breakdown || !std::isfinite(residualNormSquared);
        if (breakdown) break;
        if (residualNormSquared <= toleranceSquared * rhsNormSquared) {
          state.refreshTrueResidual(linearOperator, rhs, solution->data(),
                                    stream);
          const Impl::CheckResult verified = state.downloadCheck(stream);
          residualNormSquared = verified.residualNormSquared;
          breakdown =
              verified.breakdown || !std::isfinite(residualNormSquared);
          converged = !breakdown &&
                      residualNormSquared <= toleranceSquared * rhsNormSquared;
          if (converged || breakdown) break;
          state.restart(preconditioner, stream);
          continue;
        }
      }
      if (iteration % kResidualRefreshInterval == 0 && !atCheck) {
        state.refreshTrueResidual(linearOperator, rhs, solution->data(),
                                  stream);
        state.restart(preconditioner, stream);
        continue;
      }

      preconditioner.apply(state.residual.data(), state.preconditioned.data(),
                           stream);
      state.dot(state.residual.data(), state.preconditioned.data(),
                state.slots.data() + kRzNew, stream);
      ScalarBetaKernel<<<1, 1, 0, stream>>>(state.slots.data(),
                                            state.flags.data());
      CheckCuda(cudaGetLastError(), "beta launch");
      UpdateDirectionKernel<<<GridSize(state.dimension), kBlockSize, 0,
                              stream>>>(
          state.direction.data(), state.preconditioned.data(), state.dimension,
          state.slots.data());
      CheckCuda(cudaGetLastError(), "direction update launch");
    }

    CheckCuda(cudaStreamSynchronize(stream), "solve completion");
    ++state.statistics.solveCount;
    state.statistics.pcgIterationsTotal += static_cast<size_t>(iteration);
    state.statistics.lastPcgIterations = static_cast<size_t>(iteration);
    state.statistics.lastPcgConverged = converged;
    state.statistics.lastPcgBreakdown = breakdown;
    state.statistics.lastPcgResidualNormSquared = residualNormSquared;
    state.statistics.lastPcgRhsNormSquared = rhsNormSquared;
    if (!converged && !breakdown &&
        iteration == state.options.maxIterations) {
      ++state.statistics.pcgMaxIterationHits;
    }
    if (state.collectProfile) {
      state.statistics.solveSeconds +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        start)
              .count();
    }
    state.warmStartValid = true;
  } catch (...) {
    cudaStreamSynchronize(stream);
    throw;
  }
}

const CudaLinearSolveStats& CudaPcgSolver::stats() const {
  return impl_->statistics;
}

void CudaPcgSolver::invalidateWarmStart() { impl_->warmStartValid = false; }

}  // namespace gtsam::cuda
