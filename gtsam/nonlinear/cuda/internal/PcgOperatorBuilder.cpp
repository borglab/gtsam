/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    PcgOperatorBuilder.cpp
 * @brief   Builds the matrix-free normal operator and preconditioner for PCG
 * @author  Ruogu Li
 * @date    Jul 25, 2026
 */

#include <gtsam/nonlinear/cuda/internal/PcgOperatorBuilder.h>

#include <cmath>
#include <stdexcept>

namespace gtsam::cuda {
struct PcgOperatorBuilder::Impl {
  bool initialized = false;
  JacobianNormalOperator linearOperator;
  JacobianNormalPreconditioner preconditioner;
};

PcgOperatorBuilder::PcgOperatorBuilder() : impl_(std::make_unique<Impl>()) {}
PcgOperatorBuilder::~PcgOperatorBuilder() = default;
PcgOperatorBuilder::PcgOperatorBuilder(PcgOperatorBuilder&&) noexcept = default;
PcgOperatorBuilder& PcgOperatorBuilder::operator=(PcgOperatorBuilder&&) noexcept =
    default;

void PcgOperatorBuilder::initialize(
    cusparseHandle_t handle, int rows, int columns, cusparseSpMatDescr_t j,
    cusparseSpMatDescr_t jt, const DeviceArray<int>& jtRowPointers,
    const std::vector<int>& blockOffsets, const DevicePcgOptions& options,
    cudaStream_t stream, bool collectProfile) {
  if (rows <= 0 || columns <= 0) {
    throw std::invalid_argument(
        "PcgOperatorBuilder requires positive dimensions");
  }
  if (!std::isfinite(options.relativeTolerance) ||
      options.relativeTolerance <= 0.0 || options.maxIterations < 0 ||
      options.convergenceCheckInterval <= 0) {
    throw std::invalid_argument("PcgOperatorBuilder has invalid PCG options");
  }

  auto state = std::make_unique<Impl>();
  state->linearOperator.initialize(handle, rows, columns, j, jt, stream);
  state->preconditioner.initialize(columns, jtRowPointers, blockOffsets,
                                   options.preconditioner, stream,
                                   collectProfile);
  state->initialized = true;
  impl_ = std::move(state);
}

void PcgOperatorBuilder::buildPreconditioner(
    const DeviceArray<double>& jtValues, cudaStream_t stream) {
  if (!impl_->initialized) {
    throw std::logic_error("PcgOperatorBuilder is not initialized");
  }
  impl_->preconditioner.build(jtValues, stream);
}

void PcgOperatorBuilder::prepare(
    double lambda, const DeviceArray<double>& dampingDiagonal,
    cudaStream_t stream) {
  if (!impl_->initialized) {
    throw std::logic_error("PcgOperatorBuilder is not initialized");
  }
  impl_->linearOperator.setDamping(lambda, dampingDiagonal, stream);
  impl_->preconditioner.prepare(lambda, dampingDiagonal, stream);
}

const LinearOperator& PcgOperatorBuilder::linearOperator() const {
  if (!impl_->initialized) {
    throw std::logic_error("PcgOperatorBuilder is not initialized");
  }
  return impl_->linearOperator;
}

const Preconditioner& PcgOperatorBuilder::preconditioner() const {
  if (!impl_->initialized) {
    throw std::logic_error("PcgOperatorBuilder is not initialized");
  }
  return impl_->preconditioner;
}

double PcgOperatorBuilder::preconditionerBuildSeconds() const {
  if (!impl_->initialized) {
    throw std::logic_error("PcgOperatorBuilder is not initialized");
  }
  return impl_->preconditioner.buildSeconds();
}

}  // namespace gtsam::cuda
