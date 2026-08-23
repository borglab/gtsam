/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file timeHybridInference.cpp
 * @brief Benchmark sequential and multifrontal hybrid inference.
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <Eigen/Sparse>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"

namespace {

using gtsam::DiscreteKey;
using gtsam::DiscreteKeys;
using gtsam::GaussianFactor;
using gtsam::HybridBayesNet;
using gtsam::HybridBayesTree;
using gtsam::HybridGaussianFactor;
using gtsam::HybridGaussianFactorGraph;
using gtsam::HybridValues;
using gtsam::JacobianFactor;
using gtsam::Matrix2;
using gtsam::Ordering;
using gtsam::TableFactor;
using gtsam::Vector2;
using gtsam::noiseModel::Isotropic;
using gtsam::symbol_shorthand::A;
using gtsam::symbol_shorthand::X;
using gtsam::timing::BenchmarkMetric;
using gtsam::timing::MedianPolicy;
using gtsam::timing::TimingSummary;

volatile double timingChecksum = 0.0;

struct BenchmarkOptions {
  size_t numberObjects = 3;
  size_t warmups = 1;
  size_t repetitions = 10;
  size_t iterations = 10;
  std::optional<std::string> outputPath;
};

struct InferenceTimings {
  std::vector<double> sequentialCreate;
  std::vector<double> sequentialOptimize;
  std::vector<double> multifrontalCreate;
  std::vector<double> multifrontalOptimize;
};

DiscreteKeys makeAssociationKeys(size_t numberObjects) {
  DiscreteKeys keys;
  keys.reserve(numberObjects);
  for (size_t i = 0; i < numberObjects; ++i) {
    keys.emplace_back(A(i), numberObjects);
  }
  return keys;
}

// Construct a sparse one-to-one association constraint using only core APIs.
TableFactor makeOneToOneConstraint(size_t numberObjects) {
  const DiscreteKeys keys = makeAssociationKeys(numberObjects);
  uint64_t cardinality = 1;
  uint64_t validAssignments = 1;
  for (size_t i = 0; i < numberObjects; ++i) {
    cardinality *= numberObjects;
    validAssignments *= i + 1;
  }

  Eigen::SparseVector<double> values(cardinality);
  values.reserve(validAssignments);
  std::vector<bool> used(numberObjects, false);
  const auto addAssignments = [&](const auto& add, size_t depth,
                                  uint64_t index) -> void {
    if (depth == numberObjects) {
      values.insert(index) = 1.0;
      return;
    }
    for (size_t value = 0; value < numberObjects; ++value) {
      if (used[value]) continue;
      used[value] = true;
      add(add, depth + 1, index * numberObjects + value);
      used[value] = false;
    }
  };
  addAssignments(addAssignments, 0, 0);
  return TableFactor(keys, values);
}

std::vector<Vector2> makePredictions(size_t numberObjects) {
  constexpr double kPi = 3.14159265358979323846;
  std::vector<Vector2> predictions;
  predictions.reserve(numberObjects);
  for (size_t i = 0; i < numberObjects; ++i) {
    const double angle =
        2.0 * kPi * static_cast<double>(i) / static_cast<double>(numberObjects);
    predictions.emplace_back(4.0 * std::cos(angle), 4.0 * std::sin(angle));
  }
  return predictions;
}

HybridGaussianFactorGraph makeGraph(size_t numberObjects) {
  const std::vector<Vector2> predictions = makePredictions(numberObjects);
  std::vector<Vector2> measurements;
  measurements.reserve(numberObjects);
  for (size_t i = 0; i < numberObjects; ++i) {
    const Vector2 offset{0.05 * static_cast<double>(i + 1),
                         -0.03 * static_cast<double>(i + 1)};
    measurements.push_back(predictions[(i + 1) % numberObjects] + offset);
  }

  const auto model = Isotropic::Sigma(2, 1.5);
  HybridGaussianFactorGraph graph;
  for (size_t i = 0; i < numberObjects; ++i) {
    graph.emplace_shared<JacobianFactor>(X(i), Matrix2::Identity(),
                                         predictions[i], model);

    std::vector<GaussianFactor::shared_ptr> components;
    components.reserve(numberObjects);
    for (const Vector2& measurement : measurements) {
      components.emplace_back(std::make_shared<JacobianFactor>(
          X(i), Matrix2::Identity(), measurement, model));
    }
    graph.emplace_shared<HybridGaussianFactor>(DiscreteKey(A(i), numberObjects),
                                               components);
  }
  graph.push_back(
      std::make_shared<TableFactor>(makeOneToOneConstraint(numberObjects)));
  return graph;
}

Ordering makeOrdering(size_t numberObjects) {
  Ordering ordering;
  ordering.reserve(2 * numberObjects);
  for (size_t i = 0; i < numberObjects; ++i) ordering.push_back(X(i));
  for (size_t i = 0; i < numberObjects; ++i) ordering.push_back(A(i));
  return ordering;
}

template <class Callable>
double timeBatchMicroseconds(Callable&& callable, size_t iterations) {
  const double seconds = gtsam::timing::measureSeconds([&] {
    for (size_t i = 0; i < iterations; ++i) callable();
  });
  return seconds * 1e6 / static_cast<double>(iterations);
}

InferenceTimings measureInference(const HybridGaussianFactorGraph& graph,
                                  const Ordering& ordering,
                                  const HybridBayesNet::shared_ptr& bayesNet,
                                  const HybridBayesTree::shared_ptr& bayesTree,
                                  const BenchmarkOptions& options) {
  const auto sequentialCreate = [&] {
    const HybridBayesNet::shared_ptr result =
        graph.eliminateSequential(ordering);
    timingChecksum += static_cast<double>(result->size());
  };
  const auto sequentialOptimize = [&] {
    const HybridValues result = bayesNet->optimize();
    timingChecksum += static_cast<double>(result.discrete().at(A(0)));
  };
  const auto multifrontalCreate = [&] {
    const HybridBayesTree::shared_ptr result =
        graph.eliminateMultifrontal(ordering);
    timingChecksum += static_cast<double>(result->size());
  };
  const auto multifrontalOptimize = [&] {
    const HybridValues result = bayesTree->optimize();
    timingChecksum += static_cast<double>(result.discrete().at(A(0)));
  };

  for (size_t i = 0; i < options.warmups; ++i) {
    sequentialCreate();
    sequentialOptimize();
    multifrontalCreate();
    multifrontalOptimize();
  }

  InferenceTimings timings;
  timings.sequentialCreate.reserve(options.repetitions);
  timings.sequentialOptimize.reserve(options.repetitions);
  timings.multifrontalCreate.reserve(options.repetitions);
  timings.multifrontalOptimize.reserve(options.repetitions);
  for (size_t repetition = 0; repetition < options.repetitions; ++repetition) {
    // Rotate all four operations to reduce systematic thermal bias.
    for (size_t offset = 0; offset < 4; ++offset) {
      switch ((repetition + offset) % 4) {
        case 0:
          timings.sequentialCreate.push_back(
              timeBatchMicroseconds(sequentialCreate, options.iterations));
          break;
        case 1:
          timings.sequentialOptimize.push_back(
              timeBatchMicroseconds(sequentialOptimize, options.iterations));
          break;
        case 2:
          timings.multifrontalCreate.push_back(
              timeBatchMicroseconds(multifrontalCreate, options.iterations));
          break;
        case 3:
          timings.multifrontalOptimize.push_back(
              timeBatchMicroseconds(multifrontalOptimize, options.iterations));
          break;
      }
    }
  }
  return timings;
}

TimingSummary summarize(const std::vector<double>& samples) {
  return gtsam::timing::summarizeSamples(samples, MedianPolicy::kUpperMiddle);
}

void printUsage() {
  std::cout << "Usage: timeHybridInference [--objects N] [--warmup N] "
               "[--repeats N] [--iterations N] [--output FILE]\n";
}

}  // namespace

int main(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  if (arguments.helpRequested()) {
    printUsage();
    return 0;
  }

  const std::string outputPath = arguments.stringValue("--output", "");
  const BenchmarkOptions options{
      arguments.sizeValue("--objects", 3),
      arguments.sizeValue("--warmup", 1),
      arguments.sizeValue("--repeats", 10),
      arguments.sizeValue("--iterations", 10),
      outputPath.empty() ? std::optional<std::string>() : outputPath,
  };
  arguments.validateAllConsumed();
  if (options.numberObjects < 2 || options.numberObjects > 7 ||
      options.repetitions == 0 || options.iterations == 0) {
    throw std::invalid_argument(
        "require 2 <= objects <= 7, repeats >= 1, and iterations >= 1");
  }

  const HybridGaussianFactorGraph graph = makeGraph(options.numberObjects);
  const Ordering ordering = makeOrdering(options.numberObjects);
  const HybridBayesNet::shared_ptr bayesNet =
      graph.eliminateSequential(ordering);
  const HybridBayesTree::shared_ptr bayesTree =
      graph.eliminateMultifrontal(ordering);
  const HybridValues sequential = bayesNet->optimize();
  const HybridValues multifrontal = bayesTree->optimize();
  if (!sequential.equals(multifrontal, 1e-9)) {
    throw std::runtime_error(
        "sequential and multifrontal hybrid solutions disagree");
  }

  const InferenceTimings samples =
      measureInference(graph, ordering, bayesNet, bayesTree, options);
  const double sequentialCreate = summarize(samples.sequentialCreate).median;
  const double sequentialOptimize =
      summarize(samples.sequentialOptimize).median;
  const double multifrontalCreate =
      summarize(samples.multifrontalCreate).median;
  const double multifrontalOptimize =
      summarize(samples.multifrontalOptimize).median;

  const TableFactor constraint = makeOneToOneConstraint(options.numberObjects);
  std::cout << "Hybrid 2D one-to-one association with " << options.numberObjects
            << " objects and " << constraint.nrValues()
            << " feasible assignments\n"
            << std::fixed << std::setprecision(2)
            << "sequential_create_us: " << sequentialCreate << '\n'
            << "sequential_optimize_us: " << sequentialOptimize << '\n'
            << "multifrontal_create_us: " << multifrontalCreate << '\n'
            << "multifrontal_optimize_us: " << multifrontalOptimize << '\n';

  if (options.outputPath) {
    gtsam::timing::writeBenchmarkActionMetrics(
        *options.outputPath,
        {BenchmarkMetric{"sequential_create", "microseconds", sequentialCreate},
         BenchmarkMetric{"sequential_optimize", "microseconds",
                         sequentialOptimize},
         BenchmarkMetric{"multifrontal_create", "microseconds",
                         multifrontalCreate},
         BenchmarkMetric{"multifrontal_optimize", "microseconds",
                         multifrontalOptimize}});
  }
  std::cerr << "timing checksum: " << timingChecksum << '\n';
  return 0;
}
