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
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <algorithm>
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

  explicit Impl(const BatchedDenseSolverOptions& options) : solver(options) {}
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

  const size_t blockElems =
      static_cast<size_t>(blockDim) * static_cast<size_t>(blockDim);
  std::vector<double> hostDiag(static_cast<size_t>(batchSize) * numBlocks *
                              blockElems);
  std::vector<double> hostOffDiag(static_cast<size_t>(batchSize) *
                                 (numBlocks - 1) * blockElems);

  // Same OpenMP/GIL-release precondition as BatchedDeviceGradients's
  // identical loop -- see that file and
  // python/gtsam/specializations/cuda_nonlinear.h.
  std::vector<bool> dimensionMismatch(static_cast<size_t>(batchSize), false);
  std::vector<int> mismatchedDims(static_cast<size_t>(batchSize), 0);
  std::vector<bool> notJacobianFactor(static_cast<size_t>(batchSize), false);
#pragma omp parallel for num_threads(batchSize) schedule(static)
  for (int b = 0; b < batchSize; ++b) {
    const size_t i = static_cast<size_t>(b);
    const auto linearized = graphs[i].linearize(solutions[i]);
    KeyInfo keyInfo(*linearized);
    if (static_cast<int>(keyInfo.numCols()) != dimension) {
      dimensionMismatch[i] = true;
      mismatchedDims[i] = static_cast<int>(keyInfo.numCols());
      continue;
    }
    const auto [A, residual] = linearized->jacobian(keyInfo.ordering());
    (void)residual;

    const size_t diagItemOffset =
        i * static_cast<size_t>(numBlocks) * blockElems;
    const size_t offDiagItemOffset =
        i * static_cast<size_t>(numBlocks - 1) * blockElems;
    // Copy each block into contiguous memory first, since A.block(...)
    // is a strided view and multiplying through it directly is slow.
    std::vector<Matrix> blockCopies(static_cast<size_t>(numBlocks));
    for (int t = 0; t < numBlocks; ++t) {
      blockCopies[static_cast<size_t>(t)] =
          A.block(0, t * blockDim, A.rows(), blockDim);
    }
    for (int t = 0; t < numBlocks; ++t) {
      const Matrix& At = blockCopies[static_cast<size_t>(t)];
      const Matrix Htt = At.transpose() * At;
      std::copy(Htt.data(), Htt.data() + Htt.size(),
               hostDiag.begin() + diagItemOffset +
                   static_cast<size_t>(t) * blockElems);
      if (t + 1 < numBlocks) {
        const Matrix& At1 = blockCopies[static_cast<size_t>(t + 1)];
        const Matrix Htt1 = At.transpose() * At1;
        std::copy(Htt1.data(), Htt1.data() + Htt1.size(),
                 hostOffDiag.begin() + offDiagItemOffset +
                     static_cast<size_t>(t) * blockElems);
      }
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
        notJacobianFactor[i] = true;
        break;
      }
      rowCounts[f] = static_cast<int>(jacobianFactor->rows());
    }
  }
  for (int b = 0; b < batchSize; ++b) {
    const size_t i = static_cast<size_t>(b);
    if (dimensionMismatch[i]) {
      std::ostringstream os;
      os << "BatchedBlockTridiagonalDeviceGradients requires every graph to "
         << "linearize to numBlocks*blockDim=" << dimension << " scalar "
         << "columns; graph " << b << " has " << mismatchedDims[i];
      throw std::invalid_argument(os.str());
    }
    if (notJacobianFactor[i]) {
      throw std::invalid_argument(
          "BatchedBlockTridiagonalDeviceGradients requires factor residual "
          "Jacobians from a NoiseModelFactor (graph " + std::to_string(b) +
          ")");
    }
  }

  impl_->factorizedDiagonal.upload(hostDiag);
  impl_->factorizedOffDiagonal.upload(hostOffDiag);
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
