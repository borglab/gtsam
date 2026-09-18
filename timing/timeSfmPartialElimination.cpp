/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeSfmPartialElimination.cpp
 * @brief Compare the public CPU SFM elimination-mode and solver matrix.
 */

#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/sfm/SfmLevenbergMarquardt.h>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "internal/SfmBalBenchmark.h"
#include "internal/TimingUtils.h"

namespace {
using namespace gtsam;
namespace bal = gtsam::timing::bal;

enum class IterativeBackend { None, PCG, Subgraph };

struct SolverChoice {
  const char* name;
  SfmEliminationMode mode;
  NonlinearOptimizerParams::LinearSolverType type;
  IterativeBackend iterativeBackend = IterativeBackend::None;
};

struct Result {
  std::string solver;
  std::vector<double> samples;
  double medianSeconds = 0.0;
  double minimumSeconds = 0.0;
  double maximumSeconds = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t iterations = 0;
  size_t innerIterations = 0;
  bool exceededTimeCap = false;
  std::string failure;
};

Result run(const SolverChoice& choice, const NonlinearFactorGraph& graph,
           const Values& initial,
           const bal::BalBenchmarkConfig& benchmarkConfig,
           const Ordering& schurOrdering, const Ordering& reducedOrdering,
           size_t repetitions, double maximumSeconds) {
  SfmLevenbergMarquardtParams params;
  static_cast<LevenbergMarquardtParams&>(params) =
      bal::makeLevenbergMarquardtParams(benchmarkConfig, nullptr, "SILENT");
  params.setEliminationMode(choice.mode);
  params.setLinearSolver(choice.type);
  params.setOrdering(choice.mode == SfmEliminationMode::Full ? schurOrdering
                                                             : reducedOrdering);
  if (choice.type == NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    params.multifrontalParams.qrMode = MultifrontalParameters::QRMode::Allow;
  } else if (choice.iterativeBackend == IterativeBackend::PCG) {
    auto pcg = std::make_shared<PCGSolverParameters>(
        std::make_shared<BlockJacobiPreconditionerParameters>());
    pcg->minIterations = 0;
    pcg->maxIterations = 500;
    pcg->reset = 501;
    pcg->epsilon_abs = 0.0;
    pcg->epsilon_rel = 1e-3;
    pcg->parallel = true;
    pcg->numThreads = 0;
    params.iterativeParams = pcg;
  } else if (choice.iterativeBackend == IterativeBackend::Subgraph) {
    auto subgraph = std::make_shared<SubgraphSolverParameters>();
    subgraph->minIterations = 0;
    subgraph->maxIterations = 500;
    subgraph->reset = 501;
    subgraph->epsilon_abs = 0.0;
    subgraph->epsilon_rel = 1e-3;
    params.iterativeParams = subgraph;
  }

  Result result;
  result.solver = choice.name;
  result.initialError = graph.error(initial);
  try {
    result.samples.reserve(repetitions);
    for (size_t repetition = 0; repetition < repetitions; ++repetition) {
      SfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
      result.samples.push_back(
          gtsam::timing::measureSeconds([&] { optimizer.optimize(); }));
      result.finalError = optimizer.error();
      result.iterations = optimizer.iterations();
      result.innerIterations = optimizer.getInnerIterations();
      if (result.samples.back() >= maximumSeconds) {
        result.exceededTimeCap = true;
        break;
      }
    }
    if (!result.exceededTimeCap) {
      const gtsam::timing::TimingSummary summary =
          gtsam::timing::summarizeSamples(
              result.samples, gtsam::timing::MedianPolicy::kAverageMiddle);
      result.medianSeconds = summary.median;
      result.minimumSeconds = summary.minimum;
      result.maximumSeconds = summary.maximum;
    }
  } catch (const std::exception& error) {
    result.failure = error.what();
  }
  return result;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    gtsam::timing::Arguments arguments(argc, argv);
    if (arguments.helpRequested()) {
      std::cout << "Usage: timeSfmPartialElimination [--repetitions N] "
                   "[--max-seconds S] [--subgraph-only|--cholmod-only] "
                   "[--configuration NAME] [BALfile]\n";
      return 0;
    }
    const size_t repetitions = arguments.sizeValue("--repetitions", 1);
    const double maximumSeconds = arguments.doubleValue("--max-seconds", 300.0);
    const bool subgraphOnly = arguments.flag("--subgraph-only");
    const std::optional<std::string> configuration =
        arguments.optionalString("--configuration");
    const bool cholmodOnly = arguments.flag("--cholmod-only");
    if (repetitions == 0) {
      throw std::invalid_argument("--repetitions must be at least one");
    }
    if (maximumSeconds <= 0.0) {
      throw std::invalid_argument("--max-seconds must be positive");
    }
    if (subgraphOnly && cholmodOnly) {
      throw std::invalid_argument(
          "--subgraph-only and --cholmod-only are mutually exclusive");
    }
    if (configuration && (subgraphOnly || cholmodOnly)) {
      throw std::invalid_argument(
          "--configuration cannot be combined with a solver-only flag");
    }
    const std::vector<std::string> positionals = arguments.positionals();
    arguments.validateAllConsumed();
    if (positionals.size() > 1) {
      throw std::invalid_argument(
          "timeSfmPartialElimination accepts at most one BAL file");
    }

    const std::string filename =
        positionals.empty() ? bal::defaultDataset() : positionals.front();
    bal::BalBenchmarkConfig config;
    config.useSchur = true;
    const SfmData data = bal::loadDataset(filename);
    const NonlinearFactorGraph graph =
        bal::buildBatchSfmGraph(data, config, false, 0);
    const Values initial = bal::buildGeneralSfmInitial(data);
    const Ordering reducedOrdering =
        SfmLevenbergMarquardtOptimizer::CreateReducedOrdering(graph, initial);
    const Ordering schurOrdering =
        SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(graph,
                                                            reducedOrdering);

    const std::vector<SolverChoice> choices{
        {"Full/MultifrontalSolver", SfmEliminationMode::Full,
         NonlinearOptimizerParams::MULTIFRONTAL_SOLVER},
        {"Full/MultifrontalCholesky", SfmEliminationMode::Full,
         NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY},
        {"Full/MultifrontalQR", SfmEliminationMode::Full,
         NonlinearOptimizerParams::MULTIFRONTAL_QR},
        {"Full/SequentialCholesky", SfmEliminationMode::Full,
         NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY},
        {"Full/SequentialQR", SfmEliminationMode::Full,
         NonlinearOptimizerParams::SEQUENTIAL_QR},
        {"Full/PCG", SfmEliminationMode::Full,
         NonlinearOptimizerParams::Iterative, IterativeBackend::PCG},
        {"Full/SubgraphSolver", SfmEliminationMode::Full,
         NonlinearOptimizerParams::Iterative, IterativeBackend::Subgraph},
        {"Full/CHOLMOD", SfmEliminationMode::Full,
         NonlinearOptimizerParams::CHOLMOD},
        {"Schur/MultifrontalSolver", SfmEliminationMode::Schur,
         NonlinearOptimizerParams::MULTIFRONTAL_SOLVER},
        {"Schur/MultifrontalCholesky", SfmEliminationMode::Schur,
         NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY},
        {"Schur/MultifrontalQR", SfmEliminationMode::Schur,
         NonlinearOptimizerParams::MULTIFRONTAL_QR},
        {"Schur/SequentialCholesky", SfmEliminationMode::Schur,
         NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY},
        {"Schur/SequentialQR", SfmEliminationMode::Schur,
         NonlinearOptimizerParams::SEQUENTIAL_QR},
        {"Schur/PCG", SfmEliminationMode::Schur,
         NonlinearOptimizerParams::Iterative, IterativeBackend::PCG},
        {"Schur/SubgraphSolver", SfmEliminationMode::Schur,
         NonlinearOptimizerParams::Iterative, IterativeBackend::Subgraph},
        {"Schur/CHOLMOD", SfmEliminationMode::Schur,
         NonlinearOptimizerParams::CHOLMOD},
    };

    std::vector<Result> results;
    for (const SolverChoice& choice : choices) {
      if (subgraphOnly &&
          choice.iterativeBackend != IterativeBackend::Subgraph) {
        continue;
      }
      if (cholmodOnly && choice.type != NonlinearOptimizerParams::CHOLMOD) {
        continue;
      }
      if (configuration && choice.name != *configuration) continue;
      results.push_back(run(choice, graph, initial, config, schurOrdering,
                            reducedOrdering, repetitions, maximumSeconds));
    }
    if (results.empty()) {
      throw std::invalid_argument("No timing configuration matched");
    }

    std::cout << std::fixed << std::setprecision(6) << "Public CPU SFM timing: "
              << std::filesystem::path(filename).filename().string() << "\n\n"
              << "Repetitions: " << repetitions
              << " (optimizer construction and ordering excluded)\n"
              << "Per-run reporting cap: " << maximumSeconds << " s\n"
              << "Shared ordering: natural points, then METIS cameras\n\n"
              << "| Configuration | Median s | Min s | Max s | Initial error | "
                 "Final error | LM iterations | Inner iterations | Status |\n"
              << "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | "
                 "--- |\n";
    for (const Result& result : results) {
      std::cout << "| " << result.solver << " | ";
      if (result.exceededTimeCap || !result.failure.empty()) {
        std::cout << "- | - | - | ";
      } else {
        std::cout << result.medianSeconds << " | " << result.minimumSeconds
                  << " | " << result.maximumSeconds << " | ";
      }
      std::cout << result.initialError << " | " << result.finalError << " | "
                << result.iterations << " | " << result.innerIterations
                << " | ";
      if (result.exceededTimeCap) {
        std::cout << "exceeded " << maximumSeconds << " s cap";
      } else {
        std::cout << (result.failure.empty() ? "ok" : result.failure);
      }
      std::cout << " |\n";
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
