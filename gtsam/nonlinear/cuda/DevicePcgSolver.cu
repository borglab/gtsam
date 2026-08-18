#include <gtsam/nonlinear/cuda/DevicePcgSolver.h>

#include <gtsam/linear/cuda/PcgSolver.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace gtsam::cuda {
namespace {

PcgOptions commonOptions(const DevicePcgOptions& options, int dimension) {
  PcgOptions common;
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
  JacobianNormalOperator linearOperator;
  JacobianNormalPreconditioner preconditioner;
  // The general LM path supplies these producer-owned objects to the shared
  // LinearSolverSession. Allocate a recurrence engine only if a legacy
  // caller explicitly invokes DevicePcgSolver::solve().
  std::unique_ptr<PcgSolver> compatibilitySolver;
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
    cusparseSpMatDescr_t jt, const DeviceArray<int>& jtRowPointers,
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
  state->initialized = true;
  impl_ = std::move(state);
}

void DevicePcgSolver::buildPreconditioner(
    const DeviceArray<double>& jtValues, cudaStream_t stream) {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  impl_->preconditioner.build(jtValues, stream);
  if (impl_->compatibilitySolver) {
    impl_->compatibilitySolver->invalidateWarmStart();
  }
  impl_->profileData.preconditionerBuild =
      impl_->preconditioner.buildSeconds();
}

void DevicePcgSolver::prepare(
    double lambda, const DeviceArray<double>& dampingDiagonal,
    cudaStream_t stream) {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  impl_->linearOperator.setDamping(lambda, dampingDiagonal, stream);
  impl_->preconditioner.prepare(lambda, dampingDiagonal, stream);
}

const LinearOperator& DevicePcgSolver::linearOperator() const {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  return impl_->linearOperator;
}

const Preconditioner& DevicePcgSolver::preconditioner() const {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  return impl_->preconditioner;
}

void DevicePcgSolver::solve(
    double lambda, const DeviceArray<double>& gradient,
    const DeviceArray<double>& dampingDiagonal,
    DeviceArray<double>* delta, cudaStream_t stream) {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  if (!delta || gradient.size() != static_cast<size_t>(impl_->columns) ||
      delta->size() != static_cast<size_t>(impl_->columns)) {
    throw std::invalid_argument(
        "DevicePcgSolver solve storage sizes do not match the system");
  }
  prepare(lambda, dampingDiagonal, stream);
  if (!impl_->compatibilitySolver) {
    impl_->compatibilitySolver = std::make_unique<PcgSolver>();
    impl_->compatibilitySolver->initialize(
        impl_->columns, commonOptions(impl_->options, impl_->columns), stream,
        impl_->collectProfile);
  }
  impl_->compatibilitySolver->solve(impl_->linearOperator,
                                    impl_->preconditioner, gradient.data(),
                                    delta, stream);
  const LinearSolveStats& after = impl_->compatibilitySolver->stats();
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
