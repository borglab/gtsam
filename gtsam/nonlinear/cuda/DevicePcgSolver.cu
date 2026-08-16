#include <gtsam/nonlinear/cuda/DevicePcgSolver.h>

#include <gtsam/linear/cuda/CudaPcgSolver.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace gtsam::cuda {
namespace {

CudaPcgOptions CommonOptions(const DevicePcgOptions& options, int dimension) {
  CudaPcgOptions common;
  common.maxIterations =
      options.maxIterations == 0 ? std::min(dimension, 250)
                                 : options.maxIterations;
  common.relativeTolerance = options.relativeTolerance;
  common.warmStart = options.warmStart;
  common.convergenceCheckInterval = options.convergenceCheckInterval;
  return common;
}

}  // namespace

struct DevicePcgSolver::Impl {
  int columns = 0;
  DevicePcgOptions options;
  bool initialized = false;
  bool collectProfile = false;
  CudaJacobianNormalOperator linearOperator;
  CudaJacobianNormalPreconditioner preconditioner;
  CudaPcgSolver compatibilitySolver;
  DevicePcgSolveStats lastStats;
  DevicePcgProfile profileData;
};

DevicePcgSolver::DevicePcgSolver() : impl_(std::make_unique<Impl>()) {}
DevicePcgSolver::~DevicePcgSolver() = default;
DevicePcgSolver::DevicePcgSolver(DevicePcgSolver&&) noexcept = default;
DevicePcgSolver& DevicePcgSolver::operator=(DevicePcgSolver&&) noexcept =
    default;

void DevicePcgSolver::initialize(
    cusparseHandle_t handle, int rows, int columns, cusparseSpMatDescr_t j,
    cusparseSpMatDescr_t jt, const CudaDeviceArray<int>& jtRowPointers,
    const std::vector<int>& blockOffsets, const DevicePcgOptions& options,
    cudaStream_t stream, bool collectProfile) {
  if (rows <= 0 || columns <= 0) {
    throw std::invalid_argument(
        "DevicePcgSolver requires positive dimensions");
  }
  if (!std::isfinite(options.relativeTolerance) ||
      options.relativeTolerance <= 0.0 || options.maxIterations < 0 ||
      options.convergenceCheckInterval <= 0) {
    throw std::invalid_argument("DevicePcgSolver has invalid PCG options");
  }

  auto state = std::make_unique<Impl>();
  state->columns = columns;
  state->options = options;
  state->collectProfile = collectProfile;
  state->linearOperator.initialize(handle, rows, columns, j, jt, stream);
  state->preconditioner.initialize(columns, jtRowPointers, blockOffsets,
                                   options.preconditioner, stream,
                                   collectProfile);
  state->compatibilitySolver.initialize(
      columns, CommonOptions(options, columns), stream, collectProfile);
  state->initialized = true;
  impl_ = std::move(state);
}

void DevicePcgSolver::buildPreconditioner(
    const CudaDeviceArray<double>& jtValues, cudaStream_t stream) {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  impl_->preconditioner.build(jtValues, stream);
  impl_->compatibilitySolver.invalidateWarmStart();
  impl_->profileData.preconditionerBuild =
      impl_->preconditioner.buildSeconds();
}

void DevicePcgSolver::prepare(
    double lambda, const CudaDeviceArray<double>& dampingDiagonal,
    cudaStream_t stream) {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  impl_->linearOperator.setDamping(lambda, dampingDiagonal, stream);
  impl_->preconditioner.prepare(lambda, dampingDiagonal, stream);
}

const CudaLinearOperator& DevicePcgSolver::linearOperator() const {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  return impl_->linearOperator;
}

const CudaPreconditioner& DevicePcgSolver::preconditioner() const {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  return impl_->preconditioner;
}

void DevicePcgSolver::solve(
    double lambda, const CudaDeviceArray<double>& gradient,
    const CudaDeviceArray<double>& dampingDiagonal,
    CudaDeviceArray<double>* delta, cudaStream_t stream) {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  if (!delta || gradient.size() != static_cast<size_t>(impl_->columns) ||
      delta->size() != static_cast<size_t>(impl_->columns)) {
    throw std::invalid_argument(
        "DevicePcgSolver solve storage sizes do not match the system");
  }
  prepare(lambda, dampingDiagonal, stream);
  impl_->compatibilitySolver.solve(impl_->linearOperator,
                                   impl_->preconditioner, gradient.data(),
                                   delta, stream);
  const CudaLinearSolveStats& after = impl_->compatibilitySolver.stats();
  impl_->lastStats = {};
  impl_->lastStats.iterations = static_cast<int>(after.lastPcgIterations);
  impl_->lastStats.residualNormSquared =
      after.lastPcgResidualNormSquared;
  impl_->lastStats.gradientNormSquared = after.lastPcgRhsNormSquared;
  impl_->lastStats.converged = after.lastPcgConverged;
  impl_->lastStats.breakdown = after.lastPcgBreakdown;
  impl_->profileData.solve = after.solveSeconds;
  impl_->profileData.iterationsTotal = after.pcgIterationsTotal;
  impl_->profileData.solveCount = after.solveCount;
  impl_->profileData.maxIterationHits = after.pcgMaxIterationHits;
}

const DevicePcgSolveStats& DevicePcgSolver::lastSolveStats() const {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  return impl_->lastStats;
}

const DevicePcgProfile& DevicePcgSolver::profile() const {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  return impl_->profileData;
}

}  // namespace gtsam::cuda
