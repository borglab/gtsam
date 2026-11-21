/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedBlockTridiagonalDeviceGradients.cpp
 * @brief   GPU-resident differentiable backward for a batch of independent,
 *          converged *chain* nonlinear graphs of identical shape
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <gtsam/nonlinear/cuda/BatchedBlockTridiagonalDeviceGradients.h>

#include <gtsam/base/cuda/Errors.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/cuda/internal/BatchedBlockHessianAssembler.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace gtsam::cuda {

struct BatchedBlockTridiagonalDeviceGradients::Impl {
  std::vector<KeyInfo> keyInfos;
  // adjoint() is logically const but mutates this solver's cached device
  // state across repeated calls -- same rationale BatchedDeviceGradients
  // uses `mutable` for its own solver.
  mutable BatchedBlockTridiagonalCholeskySolver solver;
  DeviceArray<double> factorizedDiagonal;
  DeviceArray<double> factorizedOffDiagonal;
  mutable DeviceArray<double> rhs;
  int batchSize = 0;
  int numBlocks = 0;
  int blockDim = 0;
  int dimension = 0;
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> solutions;
  std::vector<Matrix> jacobians;
  std::vector<std::vector<int>> factorRowCounts;
  // Temporary diagnostic timers isolating the parameterVjp C++/Python
  // boundary crossing from everything else; see parameterGradient().
  mutable double diagPullbackSeconds = 0.0;
  mutable double diagWhitenSeconds = 0.0;
  mutable double diagVjpCallSeconds = 0.0;
  mutable size_t diagVjpCalls = 0;

  explicit Impl(const BatchedDenseSolverOptions& options) : solver(options) {}
  ~Impl() {
    if (diagVjpCalls > 0) {
      fprintf(stderr,
              "[diag-backward] pullbackSeconds=%.6f whitenSeconds=%.6f "
              "vjpCallSeconds=%.6f vjpCalls=%zu\n",
              diagPullbackSeconds, diagWhitenSeconds, diagVjpCallSeconds,
              diagVjpCalls);
    }
  }
};

BatchedBlockTridiagonalDeviceGradients::BatchedBlockTridiagonalDeviceGradients(
    const std::vector<NonlinearFactorGraph>& graphs,
    const std::vector<Values>& solutions, int numBlocks, int blockDim,
    const BatchedDenseSolverOptions& options)
    : impl_(std::make_unique<Impl>(options)) {
  if (graphs.empty() || graphs.size() != solutions.size()) {
    throw std::invalid_argument(
        "BatchedBlockTridiagonalDeviceGradients requires equally-sized, "
        "nonempty graphs and solutions batches");
  }
  if (numBlocks <= 1 || blockDim <= 0) {
    throw std::invalid_argument(
        "BatchedBlockTridiagonalDeviceGradients requires numBlocks > 1 and "
        "blockDim > 0");
  }
  const int batchSize = static_cast<int>(graphs.size());
  const int dimension = numBlocks * blockDim;
  impl_->keyInfos.resize(static_cast<size_t>(batchSize));
  impl_->graphs = graphs;
  impl_->solutions = solutions;
  impl_->jacobians.resize(static_cast<size_t>(batchSize));
  impl_->factorRowCounts.resize(static_cast<size_t>(batchSize));

  std::vector<Matrix> itemA(static_cast<size_t>(batchSize));
  int jacobianRows = -1;
  for (int b = 0; b < batchSize; ++b) {
    const size_t i = static_cast<size_t>(b);
    const auto linearized = graphs[i].linearize(solutions[i]);
    KeyInfo keyInfo(*linearized);
    if (static_cast<int>(keyInfo.numCols()) != dimension) {
      std::ostringstream os;
      os << "BatchedBlockTridiagonalDeviceGradients requires every graph to "
         << "linearize to numBlocks*blockDim=" << dimension << " scalar "
         << "columns; graph " << b << " has " << keyInfo.numCols();
      throw std::invalid_argument(os.str());
    }
    const auto [A, residual] = linearized->jacobian(keyInfo.ordering());
    (void)residual;
    itemA[i] = A;
    if (jacobianRows < 0) {
      jacobianRows = static_cast<int>(A.rows());
    } else if (static_cast<int>(A.rows()) != jacobianRows) {
      throw std::invalid_argument(
          "BatchedBlockTridiagonalDeviceGradients requires every graph to "
          "linearize to the same residual row count for the batched GPU "
          "Hessian assembly");
    }

    impl_->keyInfos[i] = std::move(keyInfo);
    impl_->jacobians[i] = A;

    std::vector<int>& rowCounts = impl_->factorRowCounts[i];
    rowCounts.resize(linearized->size());
    for (size_t f = 0; f < linearized->size(); ++f) {
      const auto& linearFactor = (*linearized)[f];
      if (!linearFactor) {
        rowCounts[f] = 0;
        continue;
      }
      const auto jacobianFactor =
          std::dynamic_pointer_cast<JacobianFactor>(linearFactor);
      if (!jacobianFactor) {
        throw std::invalid_argument(
            "BatchedBlockTridiagonalDeviceGradients requires factor residual "
            "Jacobians from a NoiseModelFactor (graph " + std::to_string(b) +
            ")");
      }
      rowCounts[f] = static_cast<int>(jacobianFactor->rows());
    }
  }

  // Forms diag/offDiag for the whole batch via GPU batched GEMM instead
  // of a per-item, per-block CPU Eigen matmul loop; rhs is unused here
  // (adjoint() supplies its own cotangent RHS at solve time).
  std::vector<double> hostJacobianA(static_cast<size_t>(batchSize) *
                                    static_cast<size_t>(jacobianRows) *
                                    static_cast<size_t>(dimension));
  for (int b = 0; b < batchSize; ++b) {
    Eigen::Map<Matrix>(hostJacobianA.data() +
                          static_cast<size_t>(b) *
                              static_cast<size_t>(jacobianRows) *
                              static_cast<size_t>(dimension),
                      jacobianRows, dimension) = itemA[static_cast<size_t>(b)];
  }
  const std::vector<double> hostJacobianB(
      static_cast<size_t>(batchSize) * static_cast<size_t>(jacobianRows),
      0.0);
  DeviceArray<double> deviceJacobianA;
  DeviceArray<double> deviceJacobianB;
  DeviceArray<double> deviceRhsUnused;
  deviceJacobianA.upload(hostJacobianA);
  deviceJacobianB.upload(hostJacobianB);
  BatchedBlockHessianAssembler assembler(options);
  assembler.assemble(batchSize, numBlocks, blockDim, jacobianRows,
                     &deviceJacobianA, &deviceJacobianB,
                     &impl_->factorizedDiagonal, &impl_->factorizedOffDiagonal,
                     &deviceRhsUnused);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));

  impl_->solver.factorize(batchSize, numBlocks, blockDim,
                         &impl_->factorizedDiagonal,
                         &impl_->factorizedOffDiagonal);
  impl_->batchSize = batchSize;
  impl_->numBlocks = numBlocks;
  impl_->blockDim = blockDim;
  impl_->dimension = dimension;
}

BatchedBlockTridiagonalDeviceGradients::
    ~BatchedBlockTridiagonalDeviceGradients() = default;

std::vector<VectorValues> BatchedBlockTridiagonalDeviceGradients::adjoint(
    const std::vector<VectorValues>& stateCotangents) const {
  if (static_cast<int>(stateCotangents.size()) != impl_->batchSize) {
    throw std::invalid_argument(
        "BatchedBlockTridiagonalDeviceGradients::adjoint requires one "
        "cotangent per graph in the batch");
  }
  const int batchSize = impl_->batchSize;
  const int dimension = impl_->dimension;

  // A flat dimension-length cotangent is already "numBlocks contiguous
  // blockDim-length blocks" in the solver's expected layout, no reshaping.
  std::vector<double> hostRhs(static_cast<size_t>(batchSize) *
                             static_cast<size_t>(dimension));
  for (int b = 0; b < batchSize; ++b) {
    const Vector flat =
        stateCotangents[static_cast<size_t>(b)].vector(
            impl_->keyInfos[static_cast<size_t>(b)].ordering());
    if (flat.size() != dimension) {
      std::ostringstream os;
      os << "BatchedBlockTridiagonalDeviceGradients::adjoint: cotangent " << b
         << " has dimension " << flat.size() << ", expected " << dimension;
      throw std::invalid_argument(os.str());
    }
    std::copy(flat.data(), flat.data() + flat.size(),
             hostRhs.begin() + static_cast<size_t>(b) *
                                    static_cast<size_t>(dimension));
  }

  impl_->rhs.upload(hostRhs);
  impl_->solver.solve(1, &impl_->rhs);

  std::vector<double> hostSolution;
  impl_->rhs.download(&hostSolution);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));

  std::vector<VectorValues> result(static_cast<size_t>(batchSize));
  for (int b = 0; b < batchSize; ++b) {
    Vector flat(dimension);
    std::copy(hostSolution.begin() + static_cast<size_t>(b) *
                                         static_cast<size_t>(dimension),
             hostSolution.begin() + static_cast<size_t>(b + 1) *
                                        static_cast<size_t>(dimension),
             flat.data());
    result[static_cast<size_t>(b)] =
        buildVectorValues(flat, impl_->keyInfos[static_cast<size_t>(b)]);
  }
  return result;
}

std::vector<std::vector<Vector>> BatchedBlockTridiagonalDeviceGradients::pullback(
    const std::vector<VectorValues>& stateCotangents) const {
  const std::vector<VectorValues> lambda = adjoint(stateCotangents);
  const int batchSize = impl_->batchSize;
  std::vector<std::vector<Vector>> result(static_cast<size_t>(batchSize));

  for (int b = 0; b < batchSize; ++b) {
    const size_t i = static_cast<size_t>(b);
    const Vector lambdaFlat = lambda[i].vector(impl_->keyInfos[i].ordering());
    const Vector fullResidual = -(impl_->jacobians[i] * lambdaFlat);
    if (!fullResidual.allFinite()) {
      throw std::runtime_error(
          "BatchedBlockTridiagonalDeviceGradients::pullback: nonfinite "
          "factor residual cotangents for graph " +
          std::to_string(b));
    }

    const std::vector<int>& rowCounts = impl_->factorRowCounts[i];
    std::vector<Vector>& perFactor = result[i];
    perFactor.resize(rowCounts.size());
    int offset = 0;
    for (size_t f = 0; f < rowCounts.size(); ++f) {
      const int rows = rowCounts[f];
      if (rows > 0) {
        perFactor[f] = fullResidual.segment(offset, rows);
        offset += rows;
      }
    }
  }
  return result;
}

std::vector<std::optional<Vector>>
BatchedBlockTridiagonalDeviceGradients::parameterGradient(
    const std::vector<VectorValues>& stateCotangents,
    const ParameterTag& parameter) const {
  auto pullbackStart = std::chrono::steady_clock::now();
  const std::vector<std::vector<Vector>> whitenedCotangents =
      pullback(stateCotangents);
  impl_->diagPullbackSeconds +=
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    pullbackStart)
          .count();
  const int batchSize = impl_->batchSize;
  std::vector<std::optional<Vector>> result(static_cast<size_t>(batchSize));

  for (int b = 0; b < batchSize; ++b) {
    const size_t i = static_cast<size_t>(b);
    const NonlinearFactorGraph& graph = impl_->graphs[i];
    const Values& solution = impl_->solutions[i];
    std::optional<Vector> accumulated;

    for (size_t index = 0; index < graph.size(); ++index) {
      const auto& factor = graph[index];
      if (!factor || !factor->active(solution)) continue;
      const auto& noiseFactor = static_cast<const NoiseModelFactor&>(*factor);

      auto whitenStart = std::chrono::steady_clock::now();
      Vector factorResidualCotangent = whitenedCotangents[i][index];
      if (const auto& noise = noiseFactor.noiseModel()) {
        if (const auto diagonal =
                std::dynamic_pointer_cast<noiseModel::Diagonal>(noise)) {
          diagonal->whitenInPlace(factorResidualCotangent);
        } else {
          const auto gaussian =
              std::static_pointer_cast<noiseModel::Gaussian>(noise);
          factorResidualCotangent =
              gaussian->R().transpose() * factorResidualCotangent;
        }
      }
      if (!factorResidualCotangent.allFinite()) {
        throw std::runtime_error(
            "BatchedBlockTridiagonalDeviceGradients::parameterGradient: "
            "nonfinite unwhitened factor residual cotangent (graph " +
            std::to_string(b) + ", factor " + std::to_string(index) + ")");
      }
      impl_->diagWhitenSeconds +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        whitenStart)
              .count();

      auto vjpStart = std::chrono::steady_clock::now();
      const auto contribution =
          noiseFactor.parameterVjp(solution, parameter, factorResidualCotangent);
      impl_->diagVjpCallSeconds +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        vjpStart)
              .count();
      ++impl_->diagVjpCalls;
      if (!contribution) continue;
      if (contribution->size() == 0 || !contribution->allFinite() ||
          (accumulated && accumulated->size() != contribution->size())) {
        throw std::runtime_error(
            "BatchedBlockTridiagonalDeviceGradients::parameterGradient: "
            "invalid parameter VJP dimension or value (graph " +
            std::to_string(b) + ", factor " + std::to_string(index) + ")");
      }
      if (accumulated) {
        *accumulated += *contribution;
      } else {
        accumulated = *contribution;
      }
    }

    if (accumulated && !accumulated->allFinite()) {
      throw std::runtime_error(
          "BatchedBlockTridiagonalDeviceGradients::parameterGradient: "
          "nonfinite accumulated parameter gradient for graph " +
          std::to_string(b));
    }
    result[i] = accumulated;
  }
  return result;
}

int BatchedBlockTridiagonalDeviceGradients::batchSize() const {
  return impl_->batchSize;
}
int BatchedBlockTridiagonalDeviceGradients::dimension() const {
  return impl_->dimension;
}
const KeyInfo& BatchedBlockTridiagonalDeviceGradients::keyInfo(
    size_t index) const {
  return impl_->keyInfos.at(index);
}

}  // namespace gtsam::cuda
