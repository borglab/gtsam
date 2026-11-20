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
 * @date    Nov 20, 2026
 */

#include <gtsam/nonlinear/cuda/BatchedDeviceGradients.h>

#include <gtsam/base/cuda/Errors.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace gtsam::cuda {

struct BatchedDeviceGradients::Impl {
  std::vector<KeyInfo> keyInfos;
  // adjoint() is logically const (it answers a query, it does not change
  // what graph this batch represents), but the solve reuses and therefore
  // mutates this solver's internal cuSOLVER/device state -- the same
  // rationale CudssSpdSolver::Impl uses `mutable` for its cached state.
  mutable BatchedDenseCholeskySolver solver;
  DeviceArray<double> factorized;
  // Reused across every adjoint() call (one per tag) instead of a fresh
  // DeviceArray each time: a prior version allocated a new rhs buffer per
  // call, and that repeated cudaMalloc/cudaFree -- not the actual solve --
  // was the dominant cost of this class at DFG-TAMP's problem size.
  mutable DeviceArray<double> rhs;
  int batchSize = 0;
  int dimension = 0;

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

  std::vector<double> hostH;  // batchSize blocks of dimension*dimension, col-major
  int dimension = 0;
  {
    // Determine dimension from item 0 before sizing the shared hostH/
    // keyInfos storage the parallel loop below writes into by index --
    // doing this probe serially first keeps that loop free of any
    // resize/reserve race.
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

  // Each item reads only its own graphs[b]/solutions[b] and writes only its
  // own hostH block/keyInfos[b] -- the B independent graphs are
  // embarrassingly parallel, same as BatchedDenseLevenbergMarquardt's
  // linearize/H_base loop. Exceptions cannot cross an OpenMP parallel-region
  // boundary, so a dimension mismatch is recorded per item and thrown once,
  // after the loop, instead of thrown in place.
  std::vector<bool> dimensionMismatch(static_cast<size_t>(batchSize), false);
  std::vector<int> mismatchedDims(static_cast<size_t>(batchSize), 0);
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

int BatchedDeviceGradients::batchSize() const { return impl_->batchSize; }
int BatchedDeviceGradients::dimension() const { return impl_->dimension; }
const KeyInfo& BatchedDeviceGradients::keyInfo(size_t index) const {
  return impl_->keyInfos.at(index);
}

}  // namespace gtsam::cuda
