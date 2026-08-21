/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearSolver.cpp
 * @brief   Shared CUDA linear-solver session and backend selection
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#include <gtsam/linear/cuda/LinearSolver.h>

#include <gtsam/linear/cuda/internal/CudssSpdSolver.h>
#include <gtsam/linear/cuda/internal/DenseCholeskySolver.h>
#include <gtsam/linear/cuda/internal/PcgSolver.h>

#include <memory>
#include <stdexcept>

namespace gtsam::cuda {

struct LinearSolverSession::Impl {
  explicit Impl(const LinearSolverOptions& optionsIn)
      : options(optionsIn) {
    switch (options.backend) {
      case LinearSolverType::DenseCholesky:
        dense = std::make_unique<DenseCholeskySolver>();
        break;
      case LinearSolverType::Cudss:
        cudss = std::make_unique<CudssSpdSolver>();
        break;
      case LinearSolverType::Pcg:
        pcg = std::make_unique<PcgSolver>();
        break;
    }
    localStats.backend = options.backend;
  }

  LinearSolverOptions options;
  std::unique_ptr<DenseCholeskySolver> dense;
  std::unique_ptr<CudssSpdSolver> cudss;
  std::unique_ptr<PcgSolver> pcg;
  LinearSolveStats localStats;
};

LinearSolverSession::LinearSolverSession(
    const LinearSolverOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}
LinearSolverSession::~LinearSolverSession() = default;
LinearSolverSession::LinearSolverSession(
    LinearSolverSession&&) noexcept = default;
LinearSolverSession& LinearSolverSession::operator=(
    LinearSolverSession&&) noexcept = default;

bool LinearSolverSession::supports(LinearSolverType backend,
                                       LinearSystemKind systemKind) {
  switch (backend) {
    case LinearSolverType::DenseCholesky:
      return systemKind == LinearSystemKind::Dense;
    case LinearSolverType::Cudss:
      return systemKind == LinearSystemKind::Sparse;
    case LinearSolverType::Pcg:
      return systemKind == LinearSystemKind::Operator;
  }
  return false;
}

void LinearSolverSession::validate(
    const LinearSolverOptions& options,
    LinearSystemKind systemKind) {
  if (!supports(options.backend, systemKind)) {
    throw std::invalid_argument(
        "CUDA linear solver does not support the requested system kind");
  }
}

void LinearSolverSession::analyze(int denseDimension,
                                      cudaStream_t stream) {
  validate(impl_->options, LinearSystemKind::Dense);
  impl_->dense->analyze(denseDimension, stream);
  ++impl_->localStats.analysisCount;
}

void LinearSolverSession::analyze(
    const DeviceSparseSpdSystem& system,
    DeviceArray<double>* solution, cudaStream_t stream) {
  validate(impl_->options, LinearSystemKind::Sparse);
  impl_->cudss->analyze(system, solution, stream);
}

void LinearSolverSession::analyze(
    const DeviceSparseSpdSystem& system,
    DeviceArray<double>* solution,
    const std::vector<int>& scalarPermutation, cudaStream_t stream) {
  validate(impl_->options, LinearSystemKind::Sparse);
  impl_->cudss->analyze(system, solution, scalarPermutation, stream);
}

void LinearSolverSession::analyze(int operatorDimension,
                                      const PcgOptions& pcgOptions,
                                      cudaStream_t stream,
                                      bool collectProfile) {
  validate(impl_->options, LinearSystemKind::Operator);
  impl_->pcg->initialize(operatorDimension, pcgOptions, stream,
                         collectProfile);
}

void LinearSolverSession::solve(DenseSpdSystemView system,
                                    DeviceArray<double>* solution,
                                    cudaStream_t stream) {
  validate(impl_->options, LinearSystemKind::Dense);
  impl_->dense->solve(system, solution, stream, &impl_->localStats);
}

void LinearSolverSession::solve(const DeviceSparseSpdSystem& system,
                                    DeviceArray<double>* solution,
                                    cudaStream_t stream) {
  validate(impl_->options, LinearSystemKind::Sparse);
  impl_->cudss->solve(system, solution, stream);
}

void LinearSolverSession::solve(
    const LinearOperator& linearOperator,
    const Preconditioner& preconditioner, const double* rhs,
    DeviceArray<double>* solution, cudaStream_t stream) {
  validate(impl_->options, LinearSystemKind::Operator);
  impl_->pcg->solve(linearOperator, preconditioner, rhs, solution, stream);
}

void LinearSolverSession::invalidateWarmStart() {
  if (impl_->pcg) impl_->pcg->invalidateWarmStart();
}

const LinearSolveStats& LinearSolverSession::stats() const {
  switch (impl_->options.backend) {
    case LinearSolverType::DenseCholesky:
      return impl_->localStats;
    case LinearSolverType::Cudss:
      return impl_->cudss->stats();
    case LinearSolverType::Pcg:
      return impl_->pcg->stats();
  }
  throw std::logic_error("CUDA linear session has unknown backend");
}

}  // namespace gtsam::cuda
