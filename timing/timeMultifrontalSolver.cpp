/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
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

#include <chrono>
#include <iostream>

#include "timeSFMBAL.h"

using namespace std;
using namespace gtsam;
using namespace example;

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
const std::string bal135 = findExampleDataFile("dubrovnik-135-90642-pre.txt");
const string bal16 = findExampleDataFile("dubrovnik-16-22106-pre");
const string bal88 = findExampleDataFile("dubrovnik-88-64298-pre");
}  // namespace

void runBAL135Benchmark(MultifrontalSolver::Parameters params) {
  const size_t iterations = 1;
  cout << "\nSingle MFS test: " << bal135 << " (iterations=" << iterations
       << ")" << std::endl;

  const SfmData db = SfmData::FromBalFile(bal135);
  const NonlinearFactorGraph graph = buildGeneralSfmGraph(db, 0.1);
  const Values initial = buildGeneralSfmInitial(db);
  const GaussianFactorGraph linear = *graph.linearize(initial);
  const Ordering ordering = createSchurOrdering(db, false);

  MultifrontalSolver solver(linear, ordering, params);
  auto start = std::chrono::high_resolution_clock::now();
  runMultifrontalSolver(solver, linear, iterations);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> t_imperative = end - start;
  cout << "  MultifrontalSolver: " << t_imperative.count() << " s" << std::endl;
  tictoc_print();
}

void runBALBenchmark(MultifrontalSolver::Parameters params) {
  const size_t bal_iterations = 2;
  for (const auto& filename : {bal16, bal88, bal135}) {
    cout << "\nProcessing BAL file: " << filename << std::endl;
    const SfmData db = SfmData::FromBalFile(filename);
    const NonlinearFactorGraph graph = buildGeneralSfmGraph(db, 0.1);
    const Values initial = buildGeneralSfmInitial(db);
    const GaussianFactorGraph linear = *graph.linearize(initial);

    const Ordering ordering = createSchurOrdering(db, false);
    cout << "\nBAL Benchmark (" << filename << ", iterations=" << bal_iterations
         << "):" << std::endl;

    MultifrontalSolver solver(linear, ordering, params);
    solver.eliminateInPlace(linear);  // Warm up cache.
    auto start = std::chrono::high_resolution_clock::now();
    runMultifrontalSolver(solver, linear, bal_iterations);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_imperative = end - start;
    cout << "  MultifrontalSolver: " << t_imperative.count() << " s"
         << std::endl;

    start = std::chrono::high_resolution_clock::now();
    runStandardSolver(linear, ordering, bal_iterations);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_standard = end - start;
    cout << "  Standard GTSAM:     " << t_standard.count() << " s" << std::endl;

    cout << "  Speedup:            "
         << t_standard.count() / t_imperative.count() << "x" << std::endl;
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

    auto start = std::chrono::high_resolution_clock::now();
    MultifrontalSolver solver(smoother, ordering, params);
    runMultifrontalSolver(solver, smoother, iterations);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_imperative = end - start;
    cout << "\nTiming results:\n";
    cout << "  MultifrontalSolver: " << t_imperative.count() << " s"
         << std::endl;

    start = std::chrono::high_resolution_clock::now();
    runStandardSolver(smoother, ordering, iterations);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_standard = end - start;
    cout << "  Standard GTSAM:     " << t_standard.count() << " s" << std::endl;

    cout << "  Speedup:            "
         << t_standard.count() / t_imperative.count() << "x" << std::endl;
  }
}

void runChain5000(MultifrontalSolver::Parameters params) {
  const size_t iterations = 5000;

  const size_t T = 5000;
  cout << "\nBenchmark (T=" << T << ", iterations=" << iterations
       << "):" << std::endl;
  GaussianFactorGraph smoother = createSmoother(T);
  const Ordering ordering = Ordering::Metis(smoother);

  auto start = std::chrono::high_resolution_clock::now();
  MultifrontalSolver solver(smoother, ordering, params);
  runMultifrontalSolver(solver, smoother, iterations);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> t_imperative = end - start;
  cout << "\nTiming results:\n";
  cout << "  MultifrontalSolver: " << t_imperative.count() << " s" << std::endl;
}

void tuneMergingBAL(MultifrontalSolver::Parameters params) {
  const size_t iterations = 2;
  const std::vector<std::string> balFiles = {bal16, bal88, bal135};
  cout << "\nTune leaf merging (BAL, iterations=" << iterations << ")"
       << std::endl;

  const std::vector<size_t> sweep = {0, 64, 128, 256, 512, 1024, 2048};
  std::vector<std::vector<double>> results(
      sweep.size(), std::vector<double>(balFiles.size(), 0.0));

  for (size_t fileIndex = 0; fileIndex < balFiles.size(); ++fileIndex) {
    const std::string& filename = balFiles[fileIndex];
    cout << "\n  BAL file: " << filename << std::endl;
    const SfmData db = SfmData::FromBalFile(filename);
    const NonlinearFactorGraph graph = buildGeneralSfmGraph(db, 0.1);
    const Values initial = buildGeneralSfmInitial(db);
    const GaussianFactorGraph linear = *graph.linearize(initial);
    const Ordering ordering = createSchurOrdering(db, false);

    for (size_t i = 0; i < sweep.size(); ++i) {
      const size_t parameter = sweep[i];
      params.leafMergeDimCap = parameter;

      MultifrontalSolver solver(linear, ordering, params);
      solver.eliminateInPlace(linear);  // Warm up cache.
      auto start = std::chrono::high_resolution_clock::now();
      runMultifrontalSolver(solver, linear, iterations);
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> t_imperative = end - start;
      results[i][fileIndex] = t_imperative.count();
      cout << "  leafMergeDimCap=" << parameter << " -> "
           << t_imperative.count() << " s\n"
           << std::endl;
    }
  }

  cout << "\n| LeafMergeDimCap | BAL16 | BAL88 | BAL135 |\n";
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

    auto start = std::chrono::high_resolution_clock::now();
    runMultifrontalSolver(solver, smoother, iterations);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_imperative = end - start;
    results.emplace_back(parameter, t_imperative.count());
    cout << "  mergeDimCap=" << parameter << " -> " << t_imperative.count()
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

int main() {
  auto params = makeDefaultParams();
  cout << "Merging dim parameter " << params.mergeDimCap << std::endl;

  // runBAL135Benchmark(params);
  runBALBenchmark(params);
  runChainBenchmark(params);
  // runChain5000(params);
  // tuneMergingBAL(params);
  // tuneMergeChain(params);
  return 0;
}
