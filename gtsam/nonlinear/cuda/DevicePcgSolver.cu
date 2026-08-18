#include <gtsam/nonlinear/cuda/DevicePcgSolver.h>

#include <cmath>
#include <stdexcept>

namespace gtsam::cuda {
struct DevicePcgSolver::Impl {
  bool initialized = false;
  JacobianNormalOperator linearOperator;
  JacobianNormalPreconditioner preconditioner;
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

double DevicePcgSolver::preconditionerBuildSeconds() const {
  if (!impl_->initialized) {
    throw std::logic_error("DevicePcgSolver is not initialized");
  }
  return impl_->preconditioner.buildSeconds();
}

}  // namespace gtsam::cuda
