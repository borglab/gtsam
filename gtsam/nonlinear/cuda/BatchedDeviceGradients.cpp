/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedDeviceGradients.cpp
 * @brief   GPU-resident differentiable backward for a batch of independent,
 *          converged nonlinear graphs of identical scalar dimension
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <gtsam/nonlinear/cuda/BatchedDeviceGradients.h>

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

struct BatchedDeviceGradients::Impl {
  std::vector<KeyInfo> keyInfos;
  // adjoint() is logically const but mutates this solver's device state.
  mutable BatchedDenseCholeskySolver solver;
  DeviceArray<double> factorized;
  // Reused across adjoint() calls to avoid a fresh DeviceArray each time.
  mutable DeviceArray<double> rhs;
  int batchSize = 0;
  int dimension = 0;
  // Retained for pullback()/parameterGradient(): original graphs/
  // solutions plus each item's dense whitened Jacobian A.
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> solutions;
  std::vector<Matrix> jacobians;
  // Per graph, per factor index: row count within jacobians[b], or 0 for
  // a null/inactive factor (mirrors Gradients' zero-row placeholder).
  std::vector<std::vector<int>> factorRowCounts;

  explicit Impl(const BatchedDenseSolverOptions& options) : solver(options) {}
};

BatchedDeviceGradients::BatchedDeviceGradients(
    const std::vector<NonlinearFactorGraph>& graphs,
    const std::vector<Values>& solutions,
    const BatchedDenseSolverOptions& options)
    : impl_(std::make_unique<Impl>(options)) {
  if (graphs.empty() || graphs.size() != solutions.size()) {
    throw std::invalid_argument(
        "BatchedDeviceGradients requires equally-sized, nonempty graphs and "
        "solutions batches");
  }
  const int batchSize = static_cast<int>(graphs.size());
  impl_->keyInfos.resize(static_cast<size_t>(batchSize));
  impl_->graphs = graphs;
  impl_->solutions = solutions;
  impl_->jacobians.resize(static_cast<size_t>(batchSize));
  impl_->factorRowCounts.resize(static_cast<size_t>(batchSize));

  std::vector<double> hostH;  // batchSize blocks of dimension*dimension, col-major
  int dimension = 0;
  {
    // Probe dimension from item 0 serially before sizing shared storage.
    const auto linearized0 = graphs[0].linearize(solutions[0]);
    const KeyInfo keyInfo0(*linearized0);
    dimension = static_cast<int>(keyInfo0.numCols());
    if (dimension <= 0) {
      throw std::invalid_argument(
          "BatchedDeviceGradients requires a positive scalar dimension");
    }
    hostH.resize(static_cast<size_t>(batchSize) *
                static_cast<size_t>(dimension) *
                static_cast<size_t>(dimension));
  }

  // Each item is independent; safe to run linearize() (and any Python
  // CustomFactor callback) from OpenMP threads only because this
  // constructor is hand-bound with py::call_guard<gil_scoped_release>()
  // in cuda_nonlinear.h. Exceptions can't cross the parallel region, so a
  // dimension mismatch is recorded per item and thrown once after.
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
    (void)residual;  // adjoint() only needs H; callers supply their own cotangents.

    const Matrix H = A.transpose() * A;
    const size_t offset =
        i * static_cast<size_t>(dimension) * static_cast<size_t>(dimension);
    std::copy(H.data(), H.data() + H.size(), hostH.begin() + offset);
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
      // NoiseModelFactor::linearize() always returns a JacobianFactor (or
      // null for an inactive factor); Gradients itself requires the same
      // and rejects HessianFactor-only factors.
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
      os << "BatchedDeviceGradients requires every graph to linearize to "
         << "the same scalar dimension; graph 0 has " << dimension
         << ", graph " << b << " has " << mismatchedDims[i];
      throw std::invalid_argument(os.str());
    }
    if (notJacobianFactor[i]) {
      throw std::invalid_argument(
          "BatchedDeviceGradients requires factor residual Jacobians from a "
          "NoiseModelFactor (graph " + std::to_string(b) + ")");
    }
  }

  impl_->factorized.upload(hostH);
  impl_->solver.factorize(batchSize, dimension, &impl_->factorized);
  impl_->batchSize = batchSize;
  impl_->dimension = dimension;
}

BatchedDeviceGradients::~BatchedDeviceGradients() = default;

std::vector<VectorValues> BatchedDeviceGradients::adjoint(
    const std::vector<VectorValues>& stateCotangents) const {
  if (static_cast<int>(stateCotangents.size()) != impl_->batchSize) {
    throw std::invalid_argument(
        "BatchedDeviceGradients::adjoint requires one cotangent per graph in "
        "the batch");
  }
  const int batchSize = impl_->batchSize;
  const int dimension = impl_->dimension;

  std::vector<double> hostRhs(static_cast<size_t>(batchSize) *
                             static_cast<size_t>(dimension));
  for (int b = 0; b < batchSize; ++b) {
    const Vector flat =
        stateCotangents[static_cast<size_t>(b)].vector(
            impl_->keyInfos[static_cast<size_t>(b)].ordering());
    if (flat.size() != dimension) {
      std::ostringstream os;
      os << "BatchedDeviceGradients::adjoint: cotangent " << b
         << " has dimension " << flat.size() << ", expected " << dimension;
      throw std::invalid_argument(os.str());
    }
    std::copy(flat.data(), flat.data() + flat.size(),
             hostRhs.begin() + static_cast<size_t>(b) *
                                    static_cast<size_t>(dimension));
  }

  // Reuses impl_->rhs (see its declaration): upload() only re-copies these
  // bytes once the batch's size is established on the first call.
  impl_->rhs.upload(hostRhs);
  impl_->solver.solve(1, &impl_->rhs);

  std::vector<double> hostSolution;
  impl_->rhs.download(&hostSolution);
  // download() does not synchronize (see DeviceArray's documented
  // contract); this is the one wait that has to happen before hostSolution
  // below is safe to read.
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

std::vector<std::vector<Vector>> BatchedDeviceGradients::pullback(
    const std::vector<VectorValues>& stateCotangents) const {
  const std::vector<VectorValues> lambda = adjoint(stateCotangents);
  const int batchSize = impl_->batchSize;
  std::vector<std::vector<Vector>> result(static_cast<size_t>(batchSize));

  for (int b = 0; b < batchSize; ++b) {
    const size_t i = static_cast<size_t>(b);
    const Vector lambdaFlat =
        lambda[i].vector(impl_->keyInfos[i].ordering());
    const Vector fullResidual = -(impl_->jacobians[i] * lambdaFlat);
    if (!fullResidual.allFinite()) {
      throw std::runtime_error(
          "BatchedDeviceGradients::pullback: nonfinite factor residual "
          "cotangents for graph " +
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
      // rows == 0 (null/inactive factor): leave the default-constructed
      // empty Vector, mirroring gtsam::Gradients::pullback's zero-row slots.
    }
  }
  return result;
}

std::vector<std::optional<Vector>> BatchedDeviceGradients::parameterGradient(
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
      // The cotangent conversion uses W transpose, not the inverse
      // whitening -- matches gtsam::Gradients::parameterGradient exactly.
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
            "BatchedDeviceGradients::parameterGradient: nonfinite "
            "unwhitened factor residual cotangent (graph " +
            std::to_string(b) + ", factor " + std::to_string(index) + ")");
      }

      const auto contribution =
          noiseFactor.parameterVjp(solution, parameter, factorResidualCotangent);
      if (!contribution) continue;
      if (contribution->size() == 0 || !contribution->allFinite() ||
          (accumulated && accumulated->size() != contribution->size())) {
        throw std::runtime_error(
            "BatchedDeviceGradients::parameterGradient: invalid parameter "
            "VJP dimension or value (graph " +
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
          "BatchedDeviceGradients::parameterGradient: nonfinite accumulated "
          "parameter gradient for graph " +
          std::to_string(b));
    }
    result[i] = accumulated;
  }
  return result;
}

int BatchedDeviceGradients::batchSize() const { return impl_->batchSize; }
int BatchedDeviceGradients::dimension() const { return impl_->dimension; }
const KeyInfo& BatchedDeviceGradients::keyInfo(size_t index) const {
  return impl_->keyInfos.at(index);
}

}  // namespace gtsam::cuda
