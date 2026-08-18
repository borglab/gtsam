#include <gtsam/linear/cuda/PcgSolver.h>

#include <gtsam/base/cuda/Errors.h>
#include <gtsam/base/cuda/PinnedHostArray.h>

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

int gridSize(int count) {
  return count / kBlockSize + (count % kBlockSize != 0);
}

__device__ double blockReduceSum(double value) {
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

__global__ void dotKernel(const double* left, const double* right, int count,
                          double* result) {
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += gridDim.x * blockDim.x) {
    local += left[i] * right[i];
  }
  const double total = blockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(result, total);
}

__global__ void coldStartKernel(double* solution, double* residual,
                                const double* rhs, int count,
                                double* residualNormSlot) {
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += gridDim.x * blockDim.x) {
    solution[i] = 0.0;
    residual[i] = rhs[i];
    local += rhs[i] * rhs[i];
  }
  const double total = blockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(residualNormSlot, total);
}

__global__ void thetaKernel(double* slots) {
  const double solutionProduct = slots[kSolutionProduct];
  const double rhsSolution = slots[kRhsSolution];
  slots[kTheta] =
      isfinite(solutionProduct) && solutionProduct > 0.0 &&
              isfinite(rhsSolution)
          ? rhsSolution / solutionProduct
          : 0.0;
}

__global__ void warmStartKernel(double* solution, double* residual,
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
  const double total = blockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(residualNormSlot, total);
}

__global__ void initializeRecurrenceKernel(double* slots) {
  slots[kRzOld] = slots[kRzNew];
  slots[kRzNew] = 0.0;
  slots[kPAp] = 0.0;
}

__global__ void scalarAlphaKernel(double* slots, int* breakdownFlag) {
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

__global__ void updateSolutionResidualKernel(
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
  const double total = blockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(residualNormSlot, total);
}

__global__ void scalarBetaKernel(double* slots, int* breakdownFlag) {
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

__global__ void updateDirectionKernel(double* direction,
                                      const double* preconditioned, int count,
                                      const double* slots) {
  const double beta = slots[kBeta];
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += gridDim.x * blockDim.x) {
    direction[i] = preconditioned[i] + beta * direction[i];
  }
}

__global__ void trueResidualKernel(double* residual, const double* rhs,
                                   const double* product, int count,
                                   double* residualNormSlot) {
  double local = 0.0;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += gridDim.x * blockDim.x) {
    const double value = rhs[i] - product[i];
    residual[i] = value;
    local += value * value;
  }
  const double total = blockReduceSum(local);
  if (threadIdx.x == 0) atomicAdd(residualNormSlot, total);
}

void checkRuntimeStatus(cudaError_t status, const char* operation) {
  if (status == cudaSuccess) return;
  std::ostringstream os;
  os << "CUDA PCG failure during " << operation << ": "
     << cudaGetErrorName(status) << " (" << cudaGetErrorString(status) << ")";
  throw std::runtime_error(os.str());
}

}  // namespace

struct PcgSolver::Impl {
  int dimension = 0;
  PcgOptions options;
  bool collectProfile = false;
  bool initialized = false;
  bool warmStartValid = false;
  DeviceArray<double> residual;
  DeviceArray<double> preconditioned;
  DeviceArray<double> direction;
  DeviceArray<double> product;
  DeviceArray<double> slots;
  DeviceArray<int> flags;
  PinnedHostArray<double> hostScalars;
  PinnedHostArray<int> hostFlags;
  cudaEvent_t d2hBegin = nullptr;
  cudaEvent_t d2hEnd = nullptr;
  LinearSolveStats statistics;

  ~Impl() {
    if (d2hEnd) cudaEventDestroy(d2hEnd);
    if (d2hBegin) cudaEventDestroy(d2hBegin);
  }

  void dot(const double* left, const double* right, double* slot,
           cudaStream_t stream) {
    dotKernel<<<gridSize(dimension), kBlockSize, 0, stream>>>(
        left, right, dimension, slot);
    checkRuntimeStatus(cudaGetLastError(), "dot launch");
  }

  struct CheckResult {
    double residualNormSquared = 0.0;
    double rhsNormSquared = 0.0;
    bool breakdown = false;
  };

  CheckResult downloadCheck(cudaStream_t stream) {
    if (collectProfile) {
      checkRuntimeStatus(cudaEventRecord(d2hBegin, stream),
                "convergence D2H begin event");
    }
    checkRuntimeStatus(cudaMemcpyAsync(hostScalars.data(), slots.data() + kRr,
                              sizeof(double), cudaMemcpyDeviceToHost, stream),
              "residual-norm download");
    checkRuntimeStatus(cudaMemcpyAsync(hostScalars.data() + 1,
                              slots.data() + kRhsNorm, sizeof(double),
                              cudaMemcpyDeviceToHost, stream),
              "rhs-norm download");
    checkRuntimeStatus(cudaMemcpyAsync(hostFlags.data(), flags.data(), sizeof(int),
                              cudaMemcpyDeviceToHost, stream),
              "breakdown-flag download");
    if (collectProfile) {
      checkRuntimeStatus(cudaEventRecord(d2hEnd, stream), "convergence D2H end event");
    }
    checkRuntimeStatus(cudaStreamSynchronize(stream), "convergence-check sync");
    ++statistics.pcgHostConvergenceChecks;
    statistics.pcgD2hBytes += 2 * sizeof(double) + sizeof(int);
    if (collectProfile) {
      float elapsedMilliseconds = 0.0f;
      checkRuntimeStatus(cudaEventElapsedTime(&elapsedMilliseconds, d2hBegin, d2hEnd),
                "convergence D2H elapsed time");
      statistics.pcgD2hSeconds += elapsedMilliseconds / 1000.0;
    }
    return {hostScalars.data()[0], hostScalars.data()[1],
            hostFlags.data()[0] != 0};
  }

  void restart(const Preconditioner& preconditioner,
               cudaStream_t stream) {
    preconditioner.apply(residual.data(), preconditioned.data(), stream);
    dot(residual.data(), preconditioned.data(), slots.data() + kRzNew,
        stream);
    initializeRecurrenceKernel<<<1, 1, 0, stream>>>(slots.data());
    checkRuntimeStatus(cudaGetLastError(), "recurrence restart launch");
    checkRuntimeStatus(cudaMemcpyAsync(direction.data(), preconditioned.data(),
                              sizeof(double) * dimension,
                              cudaMemcpyDeviceToDevice, stream),
              "search-direction reset copy");
  }

  void refreshTrueResidual(const LinearOperator& linearOperator,
                           const double* rhs, const double* solution,
                           cudaStream_t stream) {
    checkRuntimeStatus(cudaMemsetAsync(slots.data() + kRr, 0, sizeof(double), stream),
              "true-residual norm clear");
    linearOperator.apply(solution, product.data(), stream);
    trueResidualKernel<<<gridSize(dimension), kBlockSize, 0, stream>>>(
        residual.data(), rhs, product.data(), dimension, slots.data() + kRr);
    checkRuntimeStatus(cudaGetLastError(), "true-residual launch");
  }
};

PcgSolver::PcgSolver() : impl_(std::make_unique<Impl>()) {}
PcgSolver::~PcgSolver() = default;
PcgSolver::PcgSolver(PcgSolver&&) noexcept = default;
PcgSolver& PcgSolver::operator=(PcgSolver&&) noexcept = default;

void PcgSolver::initialize(int dimension, const PcgOptions& options,
                               cudaStream_t stream, bool collectProfile) {
  if (dimension <= 0) {
    throw std::invalid_argument("PcgSolver requires positive dimension");
  }
  if (options.maxIterations < 0 || options.convergenceCheckInterval <= 0 ||
      !std::isfinite(options.relativeTolerance) ||
      options.relativeTolerance <= 0.0) {
    throw std::invalid_argument("PcgSolver has invalid iteration options");
  }
  auto replacement = std::make_unique<Impl>();
  replacement->dimension = dimension;
  replacement->options = options;
  if (replacement->options.maxIterations == 0) {
    replacement->options.maxIterations = std::min(dimension, 250);
  }
  replacement->collectProfile = collectProfile;
  if (collectProfile) {
    checkRuntimeStatus(cudaEventCreate(&replacement->d2hBegin),
              "convergence D2H begin event creation");
    checkRuntimeStatus(cudaEventCreate(&replacement->d2hEnd),
              "convergence D2H end event creation");
  }
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
  replacement->statistics.backend = LinearSolverType::Pcg;
  replacement->statistics.analysisCount = 1;
  replacement->initialized = true;
  impl_ = std::move(replacement);
}

void PcgSolver::solve(const LinearOperator& linearOperator,
                          const Preconditioner& preconditioner,
                          const double* rhs,
                          DeviceArray<double>* solution,
                          cudaStream_t stream) {
  if (!impl_->initialized) {
    throw std::logic_error("PcgSolver is not initialized");
  }
  Impl& state = *impl_;
  if (!rhs || !solution ||
      solution->size() != static_cast<size_t>(state.dimension)) {
    throw std::invalid_argument("PcgSolver storage size mismatch");
  }
  if (linearOperator.dimension() != state.dimension ||
      preconditioner.dimension() != state.dimension) {
    throw std::invalid_argument("PcgSolver operator dimension mismatch");
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
      thetaKernel<<<1, 1, 0, stream>>>(state.slots.data());
      checkRuntimeStatus(cudaGetLastError(), "warm-start scale launch");
      warmStartKernel<<<gridSize(state.dimension), kBlockSize, 0, stream>>>(
          solution->data(), state.residual.data(), rhs, state.product.data(),
          state.dimension, state.slots.data(), state.slots.data() + kRr);
      checkRuntimeStatus(cudaGetLastError(), "warm-start residual launch");
    } else {
      coldStartKernel<<<gridSize(state.dimension), kBlockSize, 0, stream>>>(
          solution->data(), state.residual.data(), rhs, state.dimension,
          state.slots.data() + kRr);
      checkRuntimeStatus(cudaGetLastError(), "cold-start launch");
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
      scalarAlphaKernel<<<1, 1, 0, stream>>>(state.slots.data(),
                                             state.flags.data());
      checkRuntimeStatus(cudaGetLastError(), "alpha launch");
      updateSolutionResidualKernel<<<gridSize(state.dimension), kBlockSize, 0,
                                     stream>>>(
          solution->data(), state.residual.data(), state.direction.data(),
          state.product.data(), state.dimension, state.slots.data(),
          state.slots.data() + kRr);
      checkRuntimeStatus(cudaGetLastError(), "solution/residual update launch");
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
      scalarBetaKernel<<<1, 1, 0, stream>>>(state.slots.data(),
                                            state.flags.data());
      checkRuntimeStatus(cudaGetLastError(), "beta launch");
      updateDirectionKernel<<<gridSize(state.dimension), kBlockSize, 0,
                              stream>>>(
          state.direction.data(), state.preconditioned.data(), state.dimension,
          state.slots.data());
      checkRuntimeStatus(cudaGetLastError(), "direction update launch");
    }

    checkRuntimeStatus(cudaStreamSynchronize(stream), "solve completion");
    ++state.statistics.solveCount;
    state.statistics.pcgIterationsTotal += static_cast<size_t>(iteration);
    state.statistics.lastPcgIterations = static_cast<size_t>(iteration);
    state.statistics.lastPcgConverged = converged;
    state.statistics.lastPcgBreakdown = breakdown;
    if (breakdown) ++state.statistics.pcgBreakdownCount;
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

const LinearSolveStats& PcgSolver::stats() const {
  return impl_->statistics;
}

void PcgSolver::invalidateWarmStart() { impl_->warmStartValid = false; }

}  // namespace gtsam::cuda
