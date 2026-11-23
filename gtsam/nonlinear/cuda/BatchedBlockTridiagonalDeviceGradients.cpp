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
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace gtsam::cuda {
namespace {

// Rejects noise models that would make graph.linearize() silently whiten
// with a robust reweighting, or make the static_pointer_cast<Gaussian>
// below invalid (noiseModel::Robust does not derive from Gaussian).
void checkGaussianNoise(const SharedNoiseModel& model, size_t rows,
                        int graphIndex, size_t factorIndex) {
  if (!model) return;
  const auto gaussian = std::dynamic_pointer_cast<noiseModel::Gaussian>(model);
  if (!gaussian || model->isConstrained() || model->dim() != rows) {
    throw std::invalid_argument(
        "BatchedBlockTridiagonalDeviceGradients requires factor residual "
        "noise models to be dimension-matched, unconstrained Gaussian "
        "(graph " + std::to_string(graphIndex) + ", factor " +
        std::to_string(factorIndex) + ")");
  }
  if (const auto diagonal =
          std::dynamic_pointer_cast<noiseModel::Diagonal>(model)) {
    if (!diagonal->sigmasRef().allFinite() ||
        (diagonal->sigmasRef().array() <= 0).any()) {
      throw std::invalid_argument(
          "BatchedBlockTridiagonalDeviceGradients requires finite, positive "
          "noise sigmas (graph " + std::to_string(graphIndex) + ", factor " +
          std::to_string(factorIndex) + ")");
    }
  } else {
    const Matrix whitening = gaussian->R();
    if (!whitening.allFinite() || (whitening.diagonal().array() == 0).any()) {
      throw std::invalid_argument(
          "BatchedBlockTridiagonalDeviceGradients requires a finite, "
          "positive-definite noise covariance (graph " +
          std::to_string(graphIndex) + ", factor " +
          std::to_string(factorIndex) + ")");
    }
  }
}

// Omitted known key means zero, unknown key is an error, matching
// GaussianBayesTree::backSubstituteTranspose's own semantics.
Vector flattenCotangent(const VectorValues& cotangent, const KeyInfo& keyInfo,
                       int dimension, int graphIndex) {
  Vector flat = Vector::Zero(dimension);
  for (const auto& entry : keyInfo) {
    if (!cotangent.exists(entry.first)) continue;
    const Vector& value = cotangent.at(entry.first);
    if (static_cast<size_t>(value.size()) != entry.second.dim ||
        !value.allFinite()) {
      throw std::invalid_argument(
          "BatchedBlockTridiagonalDeviceGradients::adjoint: cotangent " +
          std::to_string(graphIndex) +
          " has an invalid or nonfinite value at a known key");
    }
    flat.segment(static_cast<Eigen::Index>(entry.second.start),
                static_cast<Eigen::Index>(entry.second.dim)) = value;
  }
  for (auto it = cotangent.begin(); it != cotangent.end(); ++it) {
    if (!keyInfo.count(it->first)) {
      throw std::invalid_argument(
          "BatchedBlockTridiagonalDeviceGradients::adjoint: cotangent " +
          std::to_string(graphIndex) + " has a key unknown to this graph");
    }
  }
  return flat;
}

}  // namespace

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
  // Re-asserted before adjoint()'s own device uploads/downloads:
  // cudaSetDevice is thread-global and must stay current per call.
  int deviceId = -1;

  explicit Impl(const BatchedDenseSolverOptions& options)
      : solver(options), deviceId(options.deviceId) {}
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
      const auto* noiseFactor =
          dynamic_cast<const NoiseModelFactor*>(graphs[i][f].get());
      if (!noiseFactor) {
        throw std::invalid_argument(
            "BatchedBlockTridiagonalDeviceGradients requires factor residual "
            "Jacobians from a NoiseModelFactor (graph " + std::to_string(b) +
            ", factor " + std::to_string(f) + ")");
      }
      checkGaussianNoise(noiseFactor->noiseModel(), jacobianFactor->rows(), b,
                        f);

      // Each factor must touch at most two adjacent blocks: the assembler
      // below only ever forms diag[t]/offDiag[t]=At^T At+1, so wider
      // coupling would silently drop that factor's contribution.
      int minBlock = std::numeric_limits<int>::max();
      int maxBlock = std::numeric_limits<int>::min();
      // Uses impl_->keyInfos[i], not the local `keyInfo`: it was already
      // std::move()'d into impl_->keyInfos[i] above, leaving the local
      // empty.
      for (const Key& key : jacobianFactor->keys()) {
        const auto entryIt = impl_->keyInfos[i].find(key);
        const size_t colOffset = entryIt->second.start;
        const size_t colWidth = entryIt->second.dim;
        const int firstBlock = static_cast<int>(colOffset / blockDim);
        const int lastBlock =
            static_cast<int>((colOffset + colWidth - 1) / blockDim);
        if (firstBlock != lastBlock) {
          throw std::invalid_argument(
              "BatchedBlockTridiagonalDeviceGradients requires every key to "
              "lie within a single numBlocks*blockDim group; factor " +
              std::to_string(f) + " (graph " + std::to_string(b) +
              ") has a key straddling a group boundary at column " +
              std::to_string(colOffset));
        }
        minBlock = std::min(minBlock, firstBlock);
        maxBlock = std::max(maxBlock, firstBlock);
      }
      if (!jacobianFactor->keys().empty() && maxBlock - minBlock > 1) {
        throw std::invalid_argument(
            "BatchedBlockTridiagonalDeviceGradients requires every factor "
            "to touch at most two adjacent blocks (chain structure); "
            "factor " + std::to_string(f) + " (graph " + std::to_string(b) +
            ") spans blocks " + std::to_string(minBlock) + ".." +
            std::to_string(maxBlock) + " -- BatchedBlockHessianAssembler "
            "only forms diagonal and immediate off-diagonal Hessian blocks "
            "and would silently drop this factor's contribution to "
            "non-adjacent blocks");
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
  if (impl_->deviceId >= 0) {
    GTSAM_CUDA_CHECK(cudaSetDevice(impl_->deviceId));
  }
  const int batchSize = impl_->batchSize;
  const int dimension = impl_->dimension;

  // A flat dimension-length cotangent is already "numBlocks contiguous
  // blockDim-length blocks" in the solver's expected layout, no reshaping.
  std::vector<double> hostRhs(static_cast<size_t>(batchSize) *
                             static_cast<size_t>(dimension));
  for (int b = 0; b < batchSize; ++b) {
    const Vector flat =
        flattenCotangent(stateCotangents[static_cast<size_t>(b)],
                        impl_->keyInfos[static_cast<size_t>(b)], dimension, b);
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
  const std::vector<std::vector<Vector>> whitenedCotangents =
      pullback(stateCotangents);
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

      const auto contribution =
          noiseFactor.parameterVjp(solution, parameter, factorResidualCotangent);
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

std::vector<VectorValues> BatchedBlockTridiagonalDeviceGradients::pushforward(
    const std::vector<std::vector<Vector>>& factorResidualDirections) const {
  const int batchSize = impl_->batchSize;
  if (static_cast<int>(factorResidualDirections.size()) != batchSize) {
    throw std::invalid_argument(
        "BatchedBlockTridiagonalDeviceGradients::pushforward requires one "
        "factor residual direction per graph in the batch");
  }
  std::vector<VectorValues> rhsBatch(static_cast<size_t>(batchSize));
  for (int b = 0; b < batchSize; ++b) {
    const size_t i = static_cast<size_t>(b);
    const std::vector<int>& rowCounts = impl_->factorRowCounts[i];
    if (factorResidualDirections[i].size() != rowCounts.size()) {
      throw std::invalid_argument(
          "BatchedBlockTridiagonalDeviceGradients::pushforward: graph " +
          std::to_string(b) +
          " expected one factor residual direction block per factor slot");
    }
    Vector flat = Vector::Zero(impl_->jacobians[i].rows());
    int offset = 0;
    for (size_t f = 0; f < rowCounts.size(); ++f) {
      const int rows = rowCounts[f];
      const Vector& block = factorResidualDirections[i][f];
      if (block.size() != rows || !block.allFinite()) {
        throw std::invalid_argument(
            "BatchedBlockTridiagonalDeviceGradients::pushforward: graph " +
            std::to_string(b) + " factor " + std::to_string(f) +
            " has an invalid or nonfinite factor residual direction");
      }
      if (rows > 0) {
        flat.segment(offset, rows) = block;
        offset += rows;
      }
    }
    const Vector rhsFlat = -(impl_->jacobians[i].transpose() * flat);
    rhsBatch[i] = buildVectorValues(rhsFlat, impl_->keyInfos[i]);
  }
  return adjoint(rhsBatch);
}

std::vector<std::optional<Vector>>
BatchedBlockTridiagonalDeviceGradients::noiseGradient(
    const std::vector<VectorValues>& stateCotangents,
    const SharedNoiseModel model) const {
  const auto diagonal = std::dynamic_pointer_cast<noiseModel::Diagonal>(model);
  if (!diagonal || model->isUnit() || model->isConstrained()) {
    throw std::invalid_argument(
        "BatchedBlockTridiagonalDeviceGradients::noiseGradient supports "
        "Diagonal and Isotropic models only");
  }
  const bool isotropic =
      static_cast<bool>(std::dynamic_pointer_cast<noiseModel::Isotropic>(model));
  const std::vector<std::vector<Vector>> whitenedCotangents =
      pullback(stateCotangents);
  const Vector& precisions = diagonal->precisions();
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
      if (noiseFactor.noiseModel().get() != model.get()) continue;
      const Vector residual = noiseFactor.unwhitenedError(solution);
      const Vector& cotangent = whitenedCotangents[i][index];
      Vector contribution =
          -2.0 * cotangent.array() * residual.array() * precisions.array();
      if (isotropic) contribution = Vector::Constant(1, contribution.sum());
      if (accumulated) {
        *accumulated += contribution;
      } else {
        accumulated = contribution;
      }
    }
    if (accumulated && !accumulated->allFinite()) {
      throw std::runtime_error(
          "BatchedBlockTridiagonalDeviceGradients::noiseGradient: nonfinite "
          "noise-model gradient for graph " +
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
