/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeBatchedBlockTridiagonalLM.cpp
 * @brief   Benchmark BatchedBlockTridiagonalLevenbergMarquardtOptimizer
 *          against CPU at DFG-TAMP's real window-graph shape (T=8 x
 *          192-dim = 1536 columns). One key per timestep, chained by
 *          adjacent-only factors, mirroring build_window_graph's
 *          structure.
 *
 * An earlier version of this benchmark also compared against a dense
 * batched optimizer (treating each item's full Hessian as one dense
 * matrix, ignoring the chain sparsity); it measured 81.6s/step at this
 * shape versus CPU's ~22s/step. That dense path has since been removed
 * from the library entirely, since GTSAM is a sparse factor-graph library
 * and the block-tridiagonal path above is both the architecturally
 * appropriate and the empirically faster choice for this chain structure.
 * @date    Nov 20, 2026
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/cuda/BatchedBlockTridiagonalLevenbergMarquardt.h>
#include <gtsam/slam/PriorFactor.h>

#include <chrono>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;
using Clock = std::chrono::steady_clock;

namespace {

constexpr int kNumBlocks = 8;    // T timesteps, matches the reported run.
constexpr int kBlockDim = 192;   // 3 slots x 64 dims, matches the reported run.
constexpr int kBatchSize = 8;    // B window graphs per training step.

/// The x2^2 term gives real curvature so repeated LM iterations do
/// genuinely different linearizations.
class NonlinearCouplingFactor : public NoiseModelFactorN<Vector, Vector> {
 public:
  using Base = NoiseModelFactorN<Vector, Vector>;

  NonlinearCouplingFactor(Key key1, Key key2, const Vector& offset,
                          const SharedNoiseModel& model)
      : Base(model, key1, key2), offset_(offset) {}

  Vector evaluateError(const Vector& x1, const Vector& x2,
                       OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    const Vector coupled = x2 + 0.1 * x2.array().square().matrix();
    if (H1) *H1 = Matrix::Identity(x1.size(), x1.size());
    if (H2) {
      *H2 = -(Matrix::Identity(x2.size(), x2.size()) +
             (0.2 * x2.array()).matrix().asDiagonal().toDenseMatrix());
    }
    return x1 - coupled - offset_;
  }

 private:
  Vector offset_;
};

NonlinearFactorGraph buildChainGraph(std::mt19937* rng) {
  std::normal_distribution<double> noise(0.0, 1.0);
  NonlinearFactorGraph graph;
  const auto noiseModel = noiseModel::Isotropic::Sigma(kBlockDim, 0.1);

  for (int t = 0; t < kNumBlocks; ++t) {
    Vector anchor(kBlockDim);
    for (int d = 0; d < kBlockDim; ++d) anchor(d) = noise(*rng);
    graph.emplace_shared<PriorFactor<Vector>>(Symbol('x', t), anchor,
                                              noiseModel);
  }
  for (int t = 0; t + 1 < kNumBlocks; ++t) {
    Vector offset(kBlockDim);
    for (int d = 0; d < kBlockDim; ++d) offset(d) = 0.05 * noise(*rng);
    graph.emplace_shared<NonlinearCouplingFactor>(
        Symbol('x', t), Symbol('x', t + 1), offset, noiseModel);
  }
  return graph;
}

Values buildInitialValues(std::mt19937* rng) {
  std::normal_distribution<double> noise(0.0, 1.0);
  Values values;
  for (int t = 0; t < kNumBlocks; ++t) {
    Vector x(kBlockDim);
    for (int d = 0; d < kBlockDim; ++d) x(d) = noise(*rng);
    values.insert(Symbol('x', t), x);
  }
  return values;
}

LevenbergMarquardtParams realToleranceParams() {
  LevenbergMarquardtParams params;
  params.maxIterations = 30;
  return params;
}

double elapsedSeconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

int main(int argc, char** argv) {
  int repeats = 3;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--repeats" && i + 1 < argc) {
      repeats = std::atoi(argv[++i]);
    }
  }

  std::cout << "Chain shape: numBlocks=" << kNumBlocks
            << " blockDim=" << kBlockDim
            << " totalColumns=" << kNumBlocks * kBlockDim
            << " batchSize=" << kBatchSize << " repeats=" << repeats
            << std::endl;

  std::mt19937 rng(12345);
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> initialValues;
  for (int b = 0; b < kBatchSize; ++b) {
    graphs.push_back(buildChainGraph(&rng));
    initialValues.push_back(buildInitialValues(&rng));
  }
  const LevenbergMarquardtParams params = realToleranceParams();

  // CPU: one LevenbergMarquardtOptimizer per graph, looped -- exactly how
  // JointWindowTrainer's current CPU path calls GTSAM today.
  {
    double total = 0.0;
    for (int r = 0; r < repeats; ++r) {
      const auto start = Clock::now();
      for (int b = 0; b < kBatchSize; ++b) {
        LevenbergMarquardtOptimizer(graphs[static_cast<size_t>(b)],
                                    initialValues[static_cast<size_t>(b)],
                                    params)
            .optimize();
      }
      total += elapsedSeconds(start);
    }
    std::cout << "CPU (per-graph LevenbergMarquardtOptimizer): mean_step="
              << (total / repeats) << "s" << std::endl;
  }

  // GPU, block-tridiagonal batched (exploits the chain sparsity instead of
  // forming a dense n x n Hessian; see the file docstring for why there is
  // no dense-batched comparison here any more).
  {
    double total = 0.0;
    size_t totalAttempts = 0;
    double solverSeconds = 0.0;
    size_t factorizeCalls = 0, solveCalls = 0;
    for (int r = 0; r < repeats; ++r) {
      const auto start = Clock::now();
      BatchedBlockTridiagonalLevenbergMarquardtOptimizer optimizer(
          graphs, initialValues, kNumBlocks, kBlockDim, params);
      optimizer.optimize();
      total += elapsedSeconds(start);
      for (const auto& item : optimizer.results()) totalAttempts += item.lambdaAttempts;
      solverSeconds += optimizer.linearSolveStats().factorizationSeconds +
                       optimizer.linearSolveStats().solveSeconds;
      factorizeCalls += optimizer.linearSolveStats().factorizationCount;
      solveCalls += optimizer.linearSolveStats().solveCount;
    }
    std::cout << "GPU block-tridiagonal batched "
                 "(BatchedBlockTridiagonalLevenbergMarquardt): mean_step="
              << (total / repeats) << "s totalLambdaAttempts=" << totalAttempts
              << " solverSeconds=" << (solverSeconds / repeats)
              << " factorizeCalls=" << factorizeCalls
              << " solveCalls=" << solveCalls << std::endl;
  }

  return 0;
}
