/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeSFMBALsmart.cpp
 * @brief Compare direct and PCG solvers for structureless BAL smart factors.
 */

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/SmartFactorParams.h>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/SfmBalBenchmark.h"
#include "internal/SfmPcgBenchmark.h"
#include "internal/TimingUtils.h"

namespace {

using namespace gtsam;
namespace bal = gtsam::timing::bal;

std::string usage() {
  return "Usage: timeSFMBALsmart "
         "[--cholesky-only | --pcg-only | --hessian-pcg-only | "
         "--implicit-schur-pcg-only] "
         "[--benchmark-action-json FILE] [BALfile]";
}

struct RunOptions {
  bal::BalBenchmarkConfig config;
  bool choleskyOnly = false;
  bool pcgOnly = false;
  bool hessianPcgOnly = false;
  bool implicitSchurPcgOnly = false;
  bool help = false;
  std::optional<std::string> benchmarkActionJson;
  std::string filename;
};

struct SmartTimingResult {
  std::string linearization;
  std::string solver;
  double graphBuildSeconds = 0.0;
  double errorEvaluationSeconds = 0.0;
  double optimizeSeconds = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t lmIterations = 0;
  size_t lmInnerIterations = 0;
  size_t linearSolves = 0;
  size_t pcgIterations = 0;
  size_t nonConvergedLinearSolves = 0;
  double operatorSetupSeconds = 0.0;
  double preconditionerSetupSeconds = 0.0;
  double linearSolveSeconds = 0.0;
};

RunOptions parseOptions(int argc, char* argv[]) {
  gtsam::timing::Arguments arguments(argc, argv);
  RunOptions options;
  options.choleskyOnly = arguments.flag("--cholesky-only");
  options.pcgOnly = arguments.flag("--pcg-only");
  options.hessianPcgOnly = arguments.flag("--hessian-pcg-only");
  options.implicitSchurPcgOnly =
      arguments.flag("--implicit-schur-pcg-only");
  options.benchmarkActionJson =
      arguments.optionalString("--benchmark-action-json");
  options.help = arguments.helpRequested();
  const std::vector<std::string> filenames = arguments.positionals();
  arguments.validateAllConsumed();

  if (options.help) return options;
  const size_t exclusiveModes = static_cast<size_t>(options.choleskyOnly) +
                                static_cast<size_t>(options.pcgOnly) +
                                static_cast<size_t>(options.hessianPcgOnly) +
                                static_cast<size_t>(
                                    options.implicitSchurPcgOnly);
  if (exclusiveModes > 1) {
    throw std::runtime_error(usage());
  }
  if (filenames.size() > 1) throw std::runtime_error(usage());
  options.filename = filenames.empty() ? bal::defaultDataset()
                                       : filenames.front();
  return options;
}

LevenbergMarquardtParams makeParameters(
    const RunOptions& options, const Ordering& cameraOrdering,
    NonlinearOptimizerParams::LinearSolverType solverType) {
  LevenbergMarquardtParams parameters = bal::makeLevenbergMarquardtParams(
      options.config, &cameraOrdering, "SILENT");
  parameters.linearSolverType = solverType;
  return parameters;
}

SmartTimingResult runSmartCase(const SfmData& data, const RunOptions& options,
                               LinearizationMode linearizationMode,
                               const std::string& linearization,
                               bool usePcg) {
  SmartTimingResult result;
  result.linearization = linearization;
  result.solver = usePcg ? "ParallelPCG(BlockJacobi)"
                         : "MultifrontalCholesky";

  NonlinearFactorGraph graph;
  const SmartProjectionParams smartParameters(linearizationMode);
  result.graphBuildSeconds = gtsam::timing::measureSeconds([&] {
    graph = bal::buildSmartSfmGraph(data, options.config, smartParameters);
  });
  const Values initial = bal::buildSmartSfmInitial(data);
  result.errorEvaluationSeconds = gtsam::timing::measureSeconds(
      [&] { result.initialError = graph.error(initial); });

  const Ordering cameraOrdering = bal::createCameraOrdering(data);
  if (usePcg) {
    const LevenbergMarquardtParams parameters = makeParameters(
        options, cameraOrdering, NonlinearOptimizerParams::Iterative);
    const bal::PcgOptimizationResult pcg =
        bal::runParallelPcgOptimization(graph, initial, parameters);
    result.optimizeSeconds = pcg.elapsedSeconds;
    result.finalError = pcg.finalError;
    result.lmIterations = pcg.lmIterations;
    result.lmInnerIterations = pcg.lmInnerIterations;
    result.linearSolves = pcg.linearSolves;
    result.pcgIterations = pcg.pcgIterations;
    result.nonConvergedLinearSolves = pcg.nonConvergedLinearSolves;
    result.operatorSetupSeconds = pcg.operatorSetupSeconds;
    result.preconditionerSetupSeconds = pcg.preconditionerSetupSeconds;
    result.linearSolveSeconds = pcg.solveSeconds;
  } else {
    const LevenbergMarquardtParams parameters = makeParameters(
        options, cameraOrdering,
        NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY);
    LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
    result.optimizeSeconds =
        gtsam::timing::measureSeconds([&] { optimizer.optimize(); });
    result.finalError = optimizer.error();
    result.lmIterations = optimizer.iterations();
    result.lmInnerIterations =
        static_cast<size_t>(optimizer.getInnerIterations());
  }
  return result;
}

void printResults(const std::string& filename,
                  const std::vector<SmartTimingResult>& results) {
  std::cout << "\nStructureless smart-factor BAL comparison for: " << filename
            << "\n"
            << std::fixed << std::setprecision(6)
            << "\n| Linearization | Solver | Graph build s | Error eval s "
               "| Optimize s | Initial error | Final error | LM iterations | "
               "LM inner iterations | Linear solves | Avg. CG iterations | "
               "Non-converged CG solves | Operator setup s | "
               "Preconditioner setup s | PCG solve s |\n"
            << "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | "
               "---: | ---: | ---: | ---: | ---: | ---: | ---: |\n";
  for (const SmartTimingResult& result : results) {
    const double averagePcgIterations =
        result.linearSolves == 0
            ? 0.0
            : static_cast<double>(result.pcgIterations) /
                  static_cast<double>(result.linearSolves);
    std::cout << "| " << result.linearization << " | " << result.solver
              << " | " << result.graphBuildSeconds << " | "
              << result.errorEvaluationSeconds << " | "
              << result.optimizeSeconds << " | " << result.initialError
              << " | " << result.finalError << " | " << result.lmIterations
              << " | "
              << result.lmInnerIterations << " | ";
    if (result.linearSolves == 0) {
      std::cout << "n/a | n/a | n/a | n/a | n/a | n/a |\n";
    } else {
      std::cout << result.linearSolves << " | " << averagePcgIterations
                << " | " << result.nonConvergedLinearSolves << " | "
                << result.operatorSetupSeconds << " | "
                << result.preconditionerSetupSeconds << " | "
                << result.linearSolveSeconds << " |\n";
    }
  }
}

void writeBenchmarkActionJson(
    const std::string& filename,
    const std::vector<SmartTimingResult>& results,
    const std::string& outputPath) {
  const std::string dataset =
      std::filesystem::path(filename).filename().string();
  std::vector<gtsam::timing::BenchmarkMetric> metrics;
  for (const SmartTimingResult& result : results) {
    const std::string prefix = "timeSFMBALsmart/" + dataset + "/" +
                               result.linearization + "/" + result.solver +
                               "/";
    metrics.push_back({prefix + "graphBuild", "s", result.graphBuildSeconds});
    metrics.push_back(
        {prefix + "errorEvaluation", "s", result.errorEvaluationSeconds});
    metrics.push_back({prefix + "optimize", "s", result.optimizeSeconds});
    metrics.push_back(
        {prefix + "lmIterations", "count",
         static_cast<double>(result.lmIterations)});
    if (result.linearSolves != 0) {
      metrics.push_back(
          {prefix + "linearSolves", "count",
           static_cast<double>(result.linearSolves)});
      metrics.push_back(
          {prefix + "averagePcgIterations", "count",
           static_cast<double>(result.pcgIterations) /
               static_cast<double>(result.linearSolves)});
    }
  }
  gtsam::timing::writeBenchmarkActionMetrics(outputPath, metrics);
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const RunOptions options = parseOptions(argc, argv);
    if (options.help) {
      std::cout << usage() << '\n';
      return 0;
    }

    const SfmData data = bal::loadDataset(options.filename);
    std::vector<SmartTimingResult> results;
    const bool defaultMode = !options.choleskyOnly && !options.pcgOnly &&
                             !options.hessianPcgOnly &&
                             !options.implicitSchurPcgOnly;
    if (defaultMode || options.choleskyOnly) {
      results.push_back(runSmartCase(data, options, HESSIAN, "HESSIAN", false));
    }
    if (defaultMode || options.pcgOnly || options.hessianPcgOnly) {
      results.push_back(runSmartCase(data, options, HESSIAN, "HESSIAN", true));
    }
    if (defaultMode || options.pcgOnly || options.implicitSchurPcgOnly) {
      results.push_back(runSmartCase(data, options, IMPLICIT_SCHUR,
                                     "IMPLICIT_SCHUR", true));
    }

    printResults(options.filename, results);
    if (options.benchmarkActionJson) {
      writeBenchmarkActionJson(options.filename, results,
                               *options.benchmarkActionJson);
    }
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
