/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file timeNonlinearOptimizerCache.cpp
 * @brief Compare validated cache reuse, unchecked reuse, and fresh elimination.
 */

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/symbolic/IndexedJunctionTree.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace gtsam;

/// Reproduce the old cache policy for fixed-structure benchmark inputs only.
class UncheckedCacheOptimizer : public GaussNewtonOptimizer {
  mutable std::optional<IndexedJunctionTree> tree_;

 public:
  using GaussNewtonOptimizer::GaussNewtonOptimizer;

  /// Reuse the first tree without validating subsequent symbolic inputs.
  VectorValues solve(const GaussianFactorGraph& graph,
                     const NonlinearOptimizerParams& params) const override {
    if (!tree_) {
      tree_ = graph.buildIndexedJunctionTree(*params.ordering);
    }
    return graph.eliminateMultifrontal(*tree_, params.getEliminationFunction())
        ->optimize();
  }
};

/// Interleave alternatives, reversing their order to limit timing drift.
std::vector<double> medianTimes(
    size_t repetitions, const std::vector<std::function<void()>>& operations) {
  using Clock = std::chrono::steady_clock;
  for (const auto& operation : operations) operation();
  std::vector<std::vector<double>> samples(operations.size());
  for (size_t sample = 0; sample < 7; ++sample) {
    for (size_t j = 0; j < operations.size(); ++j) {
      const size_t index = sample % 2 ? operations.size() - 1 - j : j;
      const auto start = Clock::now();
      for (size_t i = 0; i < repetitions; ++i) operations[index]();
      samples[index].push_back(
          std::chrono::duration<double, std::micro>(Clock::now() - start)
              .count() /
          repetitions);
    }
  }
  std::vector<double> medians;
  for (auto& times : samples) {
    std::sort(times.begin(), times.end());
    medians.push_back(times[times.size() / 2]);
  }
  return medians;
}

/// Benchmark a fixed graph with scalar or six-dimensional variable blocks.
void benchmarkLinear(size_t variables, size_t dimension, size_t repetitions) {
  GaussianFactorGraph graph;
  const Matrix identity = Matrix::Identity(dimension, dimension);
  const Vector measurement = Vector::Ones(dimension);
  for (Key key = 0; key < variables; ++key) {
    graph.emplace_shared<JacobianFactor>(key, identity, measurement);
    if (key > 0) {
      graph.emplace_shared<JacobianFactor>(key - 1, -identity, key, identity,
                                           measurement);
    }
  }
  NonlinearOptimizerParams params;
  params.setOrdering(Ordering::Colamd(graph));
  GaussNewtonOptimizer optimizer(NonlinearFactorGraph{}, Values{});
  UncheckedCacheOptimizer uncheckedOptimizer(NonlinearFactorGraph{}, Values{});
  double checksum = 0.0;
  const auto times = medianTimes(repetitions, {
      [&] { checksum += optimizer.solve(graph, params).at(0)(0); },
      [&] { checksum += uncheckedOptimizer.solve(graph, params).at(0)(0); },
      [&] { checksum += graph.optimize(*params.ordering).at(0)(0); }});
  std::cout << "linear," << variables << ',' << dimension << ',' << times[0]
            << ',' << times[1] << ',' << times[2] << ',' << checksum << '\n';
}

/// Include linearization, error evaluation, and retraction in the measurement.
void benchmarkNonlinear(size_t variables, size_t repetitions) {
  NonlinearFactorGraph graph;
  Values initial;
  const Vector3 measurement{1.0, 0.5, -0.25};
  const auto model = noiseModel::Unit::Create(3);
  for (Key key = 0; key < variables; ++key) {
    initial.insert(key, Vector3::Zero().eval());
    graph.addPrior(key, measurement, model);
    if (key > 0) {
      graph.emplace_shared<BetweenFactor<Vector3>>(key - 1, key, measurement,
                                                   model);
    }
  }
  GaussNewtonOptimizer optimizer(graph, initial);
  UncheckedCacheOptimizer uncheckedOptimizer(graph, initial);
  const auto times = medianTimes(repetitions, {
      [&] { optimizer.iterate(); }, [&] { uncheckedOptimizer.iterate(); }});
  std::cout << "nonlinear," << variables << ",3," << times[0] << ',' << times[1] << ",,"
            << optimizer.error() << '\n';
}

/// Usage: timeNonlinearOptimizerCache [variables=1000] [repetitions=30].
int main(int argc, char** argv) {
  const size_t variables = argc > 1 ? std::stoul(argv[1]) : 1000;
  const size_t repetitions = argc > 2 ? std::stoul(argv[2]) : 30;
  if (variables == 0 || repetitions == 0) {
    std::cerr << "variables and repetitions must be positive\n";
    return 1;
  }
  std::cout << std::fixed << std::setprecision(3)
            << "case,variables,dimension,cached_us,unchecked_us,rebuilt_us,checksum\n";
  benchmarkLinear(variables, 1, repetitions);
  benchmarkLinear(variables, 6, repetitions);
  benchmarkNonlinear(variables, repetitions);
  return 0;
}
