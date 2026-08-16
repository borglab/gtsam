#include <gtsam/linear/cuda/CudaPcgSolver.h>

#include <gtsam/base/cuda/CudaErrors.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace gtsam::cuda {
namespace {

constexpr int kBlockSize = 256;

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

__global__ void ResidualKernel(const double* rhs, const double* product,
                               double* residual, int count) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < count) residual[i] = rhs[i] - product[i];
}

__global__ void UpdateSolutionResidualKernel(double* solution,
                                             double* residual,
                                             const double* direction,
                                             const double* product,
                                             double alpha, int count) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < count) {
    solution[i] += alpha * direction[i];
    residual[i] -= alpha * product[i];
  }
}

__global__ void UpdateDirectionKernel(double* direction,
                                      const double* preconditioned,
                                      double beta, int count) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < count) direction[i] = preconditioned[i] + beta * direction[i];
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
  CudaDeviceArray<double> scalar;
  CudaLinearSolveStats statistics;

  double dot(const double* left, const double* right, cudaStream_t stream) {
    scalar.zero(stream);
    DotKernel<<<GridSize(dimension), kBlockSize, 0, stream>>>(
        left, right, dimension, scalar.data());
    CheckCuda(cudaGetLastError(), "dot launch");
    double result = 0.0;
    CheckCuda(cudaMemcpyAsync(&result, scalar.data(), sizeof(double),
                              cudaMemcpyDeviceToHost, stream),
              "dot download");
    CheckCuda(cudaStreamSynchronize(stream), "dot synchronization");
    return result;
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
  replacement->scalar.resize(1);
  replacement->residual.zero(stream);
  replacement->preconditioned.zero(stream);
  replacement->direction.zero(stream);
  replacement->product.zero(stream);
  replacement->scalar.zero(stream);
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
  const size_t bytes = sizeof(double) * static_cast<size_t>(state.dimension);
  try {
    if (state.warmStartValid && state.options.warmStart) {
      linearOperator.apply(solution->data(), state.product.data(), stream);
      ResidualKernel<<<GridSize(state.dimension), kBlockSize, 0, stream>>>(
          rhs, state.product.data(), state.residual.data(), state.dimension);
      CheckCuda(cudaGetLastError(), "warm residual launch");
    } else {
      solution->zero(stream);
      CheckCuda(cudaMemcpyAsync(state.residual.data(), rhs, bytes,
                                cudaMemcpyDeviceToDevice, stream),
                "initial residual copy");
    }

    const double rhsNormSquared = state.dot(rhs, rhs, stream);
    double residualNormSquared =
        state.dot(state.residual.data(), state.residual.data(), stream);
    const double toleranceSquared = state.options.relativeTolerance *
                                    state.options.relativeTolerance;
    bool converged = rhsNormSquared <= 0.0 ||
                     residualNormSquared <= toleranceSquared * rhsNormSquared;
    bool breakdown = !std::isfinite(residualNormSquared);
    int iteration = 0;

    double rz = 0.0;
    if (!converged && !breakdown) {
      preconditioner.apply(state.residual.data(), state.preconditioned.data(),
                           stream);
      rz = state.dot(state.residual.data(), state.preconditioned.data(), stream);
      breakdown = !std::isfinite(rz) || rz <= 0.0;
      if (!breakdown) {
        CheckCuda(cudaMemcpyAsync(state.direction.data(),
                                  state.preconditioned.data(), bytes,
                                  cudaMemcpyDeviceToDevice, stream),
                  "initial direction copy");
      }
    }

    while (!converged && !breakdown &&
           iteration < state.options.maxIterations) {
      linearOperator.apply(state.direction.data(), state.product.data(),
                           stream);
      const double directionProduct =
          state.dot(state.direction.data(), state.product.data(), stream);
      if (!std::isfinite(directionProduct) || directionProduct <= 0.0) {
        breakdown = true;
        break;
      }
      const double alpha = rz / directionProduct;
      if (!std::isfinite(alpha)) {
        breakdown = true;
        break;
      }
      UpdateSolutionResidualKernel<<<GridSize(state.dimension), kBlockSize, 0,
                                     stream>>>(
          solution->data(), state.residual.data(), state.direction.data(),
          state.product.data(), alpha, state.dimension);
      CheckCuda(cudaGetLastError(), "solution/residual update launch");
      ++iteration;

      residualNormSquared =
          state.dot(state.residual.data(), state.residual.data(), stream);
      if (!std::isfinite(residualNormSquared)) {
        breakdown = true;
        break;
      }
      const bool check =
          iteration % state.options.convergenceCheckInterval == 0 ||
          iteration == state.options.maxIterations;
      if (check && residualNormSquared <= toleranceSquared * rhsNormSquared) {
        linearOperator.apply(solution->data(), state.product.data(), stream);
        ResidualKernel<<<GridSize(state.dimension), kBlockSize, 0, stream>>>(
            rhs, state.product.data(), state.residual.data(), state.dimension);
        CheckCuda(cudaGetLastError(), "verified residual launch");
        residualNormSquared =
            state.dot(state.residual.data(), state.residual.data(), stream);
        converged = std::isfinite(residualNormSquared) &&
                    residualNormSquared <= toleranceSquared * rhsNormSquared;
        if (converged) break;
      }

      preconditioner.apply(state.residual.data(), state.preconditioned.data(),
                           stream);
      const double rzNew =
          state.dot(state.residual.data(), state.preconditioned.data(), stream);
      if (!std::isfinite(rzNew) || rzNew <= 0.0 || rz <= 0.0) {
        breakdown = true;
        break;
      }
      const double beta = rzNew / rz;
      UpdateDirectionKernel<<<GridSize(state.dimension), kBlockSize, 0,
                              stream>>>(
          state.direction.data(), state.preconditioned.data(), beta,
          state.dimension);
      CheckCuda(cudaGetLastError(), "direction update launch");
      rz = rzNew;
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
