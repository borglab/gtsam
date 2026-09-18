/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeMultifrontalSolver.cpp
 * @brief   Compare MultifrontalSolver against standard elimination
 * @author  Frank Dellaert
 * @date    December 2025
 */

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <tests/smallExample.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"
#include "timeSFMBAL.h"

using namespace std;
using namespace gtsam;
using namespace example;
namespace bal = gtsam::timing::bal;

namespace {
// Thresholds chosen empirically for these timing experiments: merging small
// frontal cliques tends to improve performance without materially affecting
// numerical behavior for the tested problems.
MultifrontalSolver::Parameters makeDefaultParams() {
  MultifrontalSolver::Parameters params;
  params.mergeDimCap = 32;
  // params.reportStream = &std::cout;
  return params;
}

struct Options {
  bool bal135Only = false;
  size_t balDataset = 0;
  size_t samples = 1;
  size_t iterations = 2;
  size_t threads = 0;
};

size_t parsePositiveSize(const char* option, const char* value) {
  size_t consumed = 0;
  const std::string text(value);
  const unsigned long long parsed = std::stoull(text, &consumed);
  if (consumed != text.size() || parsed == 0) {
    throw std::invalid_argument(std::string(option) +
                                " requires a positive integer");
  }
  return static_cast<size_t>(parsed);
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string option(argv[i]);
    if (option == "--bal135-only") {
      options.bal135Only = true;
    } else if (option == "--bal-dataset" || option == "--samples" ||
               option == "--iterations" || option == "--threads") {
      if (++i >= argc) {
        throw std::invalid_argument(option + " requires a value");
      }
      const size_t value = parsePositiveSize(option.c_str(), argv[i]);
      if (option == "--bal-dataset") {
        if (value != 16 && value != 88 && value != 135) {
          throw std::invalid_argument(
              "--bal-dataset requires one of 16, 88, or 135");
        }
        options.balDataset = value;
      } else if (option == "--samples")
        options.samples = value;
      else if (option == "--iterations")
        options.iterations = value;
      else
        options.threads = value;
    } else if (option == "--help") {
      cout << "Usage: timeMultifrontalSolver [--bal135-only] "
              "[--bal-dataset 16|88|135] "
              "[--samples N] [--iterations N] [--threads N]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  if (options.bal135Only && options.balDataset != 0 &&
      options.balDataset != 135) {
    throw std::invalid_argument("--bal135-only conflicts with --bal-dataset");
  }
  return options;
}

}  // namespace

/// Run standard GTSAM multifrontal elimination and optimization.
static void runStandardSolver(const GaussianFactorGraph& smoother,
                              const Ordering& ordering, size_t iterations) {
  for (size_t i = 0; i < iterations; ++i) {
    GaussianBayesTree bayesTree = *smoother.eliminateMultifrontal(ordering);
    VectorValues solution = bayesTree.optimize();
    (void)solution;
  }
}

/// Run new MultifrontalSolver elimination and optimization.
static void runMultifrontalSolver(MultifrontalSolver& solver,
                                  const GaussianFactorGraph& graph,
                                  size_t iterations) {
  for (size_t i = 0; i < iterations; ++i) {
    solver.eliminateInPlace(graph);
    const VectorValues& solution = solver.updateSolution();
    (void)solution;
  }
}

namespace {
const std::vector<std::string> balDatasets = bal::standardDatasets();
const string& bal16 = balDatasets[0];
const string& bal88 = balDatasets[1];
const string& bal135 = balDatasets[2];
}  // namespace

const string& sampledBalDataset(size_t cameraCount) {
  switch (cameraCount) {
    case 16:
      return bal16;
    case 88:
      return bal88;
    case 135:
      return bal135;
    default:
      throw std::invalid_argument("unsupported BAL camera count");
  }
}

void runSampledBALBenchmark(MultifrontalSolver::Parameters params,
                            size_t cameraCount, size_t samples,
                            size_t iterations) {
  const string& filename = sampledBalDataset(cameraCount);
  cout << "\nBAL-" << cameraCount << " sampled benchmark: " << filename
       << " (samples=" << samples << ", iterations=" << iterations
       << ", threads=" << params.numThreads << ")" << std::endl;

  const SfmData db = bal::loadDataset(filename);
  const bal::BalBenchmarkConfig config;
  const NonlinearFactorGraph graph = bal::buildGeneralSfmGraph(db, config, 0.1);
  const Values initial = bal::buildGeneralSfmInitial(db);
  const GaussianFactorGraph linear = *graph.linearize(initial);
  const Ordering ordering = bal::createSchurOrdering(db, false);

  MultifrontalSolver solver(linear, ordering, params);
  runMultifrontalSolver(solver, linear, 1);  // Untimed warmup.
  std::vector<double> timings;
  timings.reserve(samples);
  for (size_t sample = 0; sample < samples; ++sample) {
    const double seconds = gtsam::timing::measureSeconds(
        [&] { runMultifrontalSolver(solver, linear, iterations); });
    timings.push_back(seconds);
    cout << "  Sample " << sample + 1 << ": " << seconds << " s" << std::endl;
  }
  std::sort(timings.begin(), timings.end());
  const size_t middle = timings.size() / 2;
  const double median = timings.size() % 2 == 0
                            ? 0.5 * (timings[middle - 1] + timings[middle])
                            : timings[middle];
  cout << "  Median: " << median << " s" << std::endl;
}

void runBALBenchmark(MultifrontalSolver::Parameters params) {
  const size_t bal_iterations = 2;
  for (const auto& filename : {bal16, bal88, bal135}) {
    cout << "\nProcessing BAL file: " << filename << std::endl;
    const SfmData db = bal::loadDataset(filename);
    const bal::BalBenchmarkConfig config;
    const NonlinearFactorGraph graph =
        bal::buildGeneralSfmGraph(db, config, 0.1);
    const Values initial = bal::buildGeneralSfmInitial(db);
    const GaussianFactorGraph linear = *graph.linearize(initial);

    const Ordering ordering = bal::createSchurOrdering(db, false);
    cout << "\nBAL Benchmark (" << filename << ", iterations=" << bal_iterations
         << "):" << std::endl;

    MultifrontalSolver solver(linear, ordering, params);
    solver.eliminateInPlace(linear);  // Warm up cache.
    const double imperativeSeconds = gtsam::timing::measureSeconds(
        [&] { runMultifrontalSolver(solver, linear, bal_iterations); });
    cout << "  MultifrontalSolver: " << imperativeSeconds << " s" << std::endl;

    const double standardSeconds = gtsam::timing::measureSeconds(
        [&] { runStandardSolver(linear, ordering, bal_iterations); });
    cout << "  Standard GTSAM:     " << standardSeconds << " s" << std::endl;

    cout << "  Speedup:            " << standardSeconds / imperativeSeconds
         << "x" << std::endl;
  }
}

void runChainBenchmark(MultifrontalSolver::Parameters params) {
  const std::vector<size_t> T_values = {10, 50, 100, 500, 1000, 5000};
  const size_t iterations = 500;

  for (size_t T : T_values) {
    cout << "\nBenchmark (T=" << T << ", iterations=" << iterations
         << "):" << std::endl;
    GaussianFactorGraph smoother = createSmoother(T);
    const Ordering ordering = Ordering::Metis(smoother);

    double imperativeSeconds = 0.0;
    imperativeSeconds = gtsam::timing::measureSeconds([&] {
      MultifrontalSolver solver(smoother, ordering, params);
      runMultifrontalSolver(solver, smoother, iterations);
    });
    cout << "\nTiming results:\n";
    cout << "  MultifrontalSolver: " << imperativeSeconds << " s" << std::endl;

    const double standardSeconds = gtsam::timing::measureSeconds(
        [&] { runStandardSolver(smoother, ordering, iterations); });
    cout << "  Standard GTSAM:     " << standardSeconds << " s" << std::endl;

    cout << "  Speedup:            " << standardSeconds / imperativeSeconds
         << "x" << std::endl;
  }
}

void runChain5000(MultifrontalSolver::Parameters params) {
  const size_t iterations = 5000;

  const size_t T = 5000;
  cout << "\nBenchmark (T=" << T << ", iterations=" << iterations
       << "):" << std::endl;
  GaussianFactorGraph smoother = createSmoother(T);
  const Ordering ordering = Ordering::Metis(smoother);

  const double imperativeSeconds = gtsam::timing::measureSeconds([&] {
    MultifrontalSolver solver(smoother, ordering, params);
    runMultifrontalSolver(solver, smoother, iterations);
  });
  cout << "\nTiming results:\n";
  cout << "  MultifrontalSolver: " << imperativeSeconds << " s" << std::endl;
}

void tuneMergingBAL(MultifrontalSolver::Parameters params) {
  const size_t iterations = 2;
  const std::vector<std::string> balFiles = {bal16, bal88, bal135};
  cout << "\nTune leaf scheduling aggregation (BAL, iterations=" << iterations
       << ")" << std::endl;

  const std::vector<size_t> sweep = {0, 64, 128, 256, 512, 1024, 2048};
  std::vector<std::vector<double>> results(
      sweep.size(), std::vector<double>(balFiles.size(), 0.0));

  for (size_t fileIndex = 0; fileIndex < balFiles.size(); ++fileIndex) {
    const std::string& filename = balFiles[fileIndex];
    cout << "\n  BAL file: " << filename << std::endl;
    const SfmData db = bal::loadDataset(filename);
    const bal::BalBenchmarkConfig config;
    const NonlinearFactorGraph graph =
        bal::buildGeneralSfmGraph(db, config, 0.1);
    const Values initial = bal::buildGeneralSfmInitial(db);
    const GaussianFactorGraph linear = *graph.linearize(initial);
    const Ordering ordering = bal::createSchurOrdering(db, false);

    for (size_t i = 0; i < sweep.size(); ++i) {
      const size_t parameter = sweep[i];
      params.leafAggregationProblemSize = parameter;

      MultifrontalSolver solver(linear, ordering, params);
      solver.eliminateInPlace(linear);  // Warm up cache.
      const double imperativeSeconds = gtsam::timing::measureSeconds(
          [&] { runMultifrontalSolver(solver, linear, iterations); });
      results[i][fileIndex] = imperativeSeconds;
      cout << "  leafAggregationProblemSize=" << parameter << " -> "
           << imperativeSeconds << " s\n"
           << std::endl;
    }
  }

  cout << "\n| LeafAggregationProblemSize | BAL16 | BAL88 | BAL135 |\n";
  cout << "| --- | --- | --- | --- |\n";
  for (size_t i = 0; i < sweep.size(); ++i) {
    cout << "| " << sweep[i];
    for (size_t fileIndex = 0; fileIndex < balFiles.size(); ++fileIndex) {
      cout << " | " << results[i][fileIndex];
    }
    cout << " |\n";
  }
}

void tuneMergeChain(MultifrontalSolver::Parameters params) {
  const size_t iterations = 100;
  const size_t T = 5000;
  cout << "\nTune mergeDimCap (chain T=" << T << ", iterations=" << iterations
       << ")" << std::endl;

  GaussianFactorGraph smoother = createSmoother(T);
  const Ordering ordering = Ordering::Metis(smoother);

  const std::vector<size_t> sweep = {0, 16, 32, 64, 128, 256, 512};
  std::vector<std::pair<size_t, double>> results;
  for (size_t parameter : sweep) {
    params.mergeDimCap = parameter;
    MultifrontalSolver solver(smoother, ordering, params);

    const double imperativeSeconds = gtsam::timing::measureSeconds(
        [&] { runMultifrontalSolver(solver, smoother, iterations); });
    results.emplace_back(parameter, imperativeSeconds);
    cout << "  mergeDimCap=" << parameter << " -> " << imperativeSeconds
         << " s\n"
         << std::endl;
  }

  cout << "\n| Phase | Cap | Seconds |\n";
  cout << "| --- | --- | --- |\n";
  for (const auto& result : results) {
    cout << "| mergeDimCap | " << result.first << " | " << result.second
         << " |\n";
  }
}

int main(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);
  auto params = makeDefaultParams();
  if (options.threads > 0) params.numThreads = options.threads;
  cout << "Merging dim parameter " << params.mergeDimCap << std::endl;

  const size_t sampledDataset = options.bal135Only ? 135 : options.balDataset;
  if (sampledDataset != 0) {
    runSampledBALBenchmark(params, sampledDataset, options.samples,
                           options.iterations);
    return 0;
  }
  runBALBenchmark(params);
  runChainBenchmark(params);
  // runChain5000(params);
  // tuneMergingBAL(params);
  // tuneMergeChain(params);
  return 0;
}
