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
// Threshold chosen empirically for these timing experiments: merging small
// frontal cliques tends to improve performance without materially affecting
// numerical behavior for the tested problems.
constexpr size_t kMergeDimCap = 16;
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
                                  const GaussianFactorGraph& smoother,
                                  size_t iterations) {
  for (size_t i = 0; i < iterations; ++i) {
    solver.load(smoother);
    solver.eliminateInPlace();
    const VectorValues& solution = solver.updateSolution();
    (void)solution;
  }
}

int main() {
  cout << "Merging dim cap " << kMergeDimCap << std::endl;

  {
    const size_t bal_iterations = 5;
    const string bal16 = findExampleDataFile("dubrovnik-16-22106-pre");
    const string bal88 = findExampleDataFile("dubrovnik-88-64298-pre");
    for (const auto& filename : {bal16, bal88}) {
      cout << "\nProcessing BAL file: " << filename << std::endl;
      const SfmData db = SfmData::FromBalFile(filename);
      const NonlinearFactorGraph graph = buildGeneralSfmGraph(db, 0.1);
      const Values initial = buildGeneralSfmInitial(db);
      const GaussianFactorGraph linear = *graph.linearize(initial);

      const std::vector<std::pair<string, Ordering>> orderings = {
          {"Burn", createSchurOrdering(db, false)},
          {"Metis", Ordering::Metis(linear)},
          {"Schur", createSchurOrdering(db, false)},
          {"Colamd", Ordering::Colamd(linear)},
      };

      for (const auto& [label, ordering] : orderings) {
        cout << "\nBAL Benchmark (" << label
             << ", iterations=" << bal_iterations << "):" << std::endl;

        MultifrontalSolver solver(linear, ordering, kMergeDimCap, nullptr);
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
        cout << "  Standard GTSAM:     " << t_standard.count() << " s"
             << std::endl;

        cout << "  Speedup:            "
             << t_standard.count() / t_imperative.count() << "x" << std::endl;
      }
    }
  }

  const std::vector<size_t> T_values = {10, 50, 100, 500, 1000, 5000};
  const size_t iterations = 1000;

  for (size_t T : T_values) {
    cout << "\nBenchmark (T=" << T << ", iterations=" << iterations
         << "):" << std::endl;
    GaussianFactorGraph smoother = createSmoother(T);
    const Ordering ordering = Ordering::Metis(smoother);

    MultifrontalSolver solver(smoother, ordering, kMergeDimCap, &std::cout);
    auto start = std::chrono::high_resolution_clock::now();
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
  return 0;
}
