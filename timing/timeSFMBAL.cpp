/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMBAL.cpp
 * @brief   time SFM with BAL file,  conventional GeneralSFMFactor
 * @author  Frank Dellaert
 * @date    June 6, 2015
 */

#include "timeSFMBAL.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/symbolic/IndexedJunctionTree.h>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>

#include "internal/SfmPcgBenchmark.h"
#include "internal/SfmCholmodBenchmark.h"
#include "internal/TimingUtils.h"

namespace {
using namespace gtsam;
using symbol_shorthand::P;
namespace bal = gtsam::timing::bal;

std::string usage() {
  return "Usage: timeSFMBAL [--colamd] [--profile] [--camera-batch] "
         "[--cholesky-only] [--profile-point-cholesky] "
         "[--end-to-end-pcg] "
         "[--point-batch-schur-cholmod-only] "
         "[--batch-chunk-size N] "
         "[--benchmark-action-json FILE] [BALfile]";
}

struct TimingRow {
  std::string dataset;
  double regularCholesky = 0.0;
  double regularQr = 0.0;
  double regularSolver = 0.0;
  double batchCholesky = 0.0;
  double batchQr = 0.0;
  double batchSolver = 0.0;
  double cameraFirstCholesky = 0.0;
  double cameraFirstQr = 0.0;
  double cameraFirstSolver = 0.0;
  double cameraBatchCholesky = 0.0;
  double cameraBatchQr = 0.0;
  double cameraBatchSolver = 0.0;
};

struct RunOptions {
  bal::BalBenchmarkConfig config;
  bool profile = false;
  bool benchmarkActionJson = false;
  bool cameraBatch = false;
  bool choleskyOnly = false;
  bool profilePointCholesky = false;
  size_t batchChunkSize = 0;
  std::string benchmarkActionJsonPath;
  std::vector<std::string> filenames;
  bool endToEndPcg = false;
  bool pointBatchSchurCholmodOnly = false;
  bool help = false;
};

struct PointCholeskyProfileRow {
  std::string dataset;
  std::string variant;
  size_t iterations = 0;
  size_t innerIterations = 0;
  size_t nonlinearFactors = 0;
  size_t linearFactors = 0;
  size_t jacobianFactors = 0;
  size_t batchFactors = 0;
  size_t linearRows = 0;
  size_t batchDenseScalarEntries = 0;
  size_t batchStoredScalarEntriesEstimate = 0;
  double initialError = 0.0;
  double finalError = 0.0;
  double linearize = 0.0;
  double hessianDiagonal = 0.0;
  double damp = 0.0;
  double symbolic = 0.0;
  double eliminate = 0.0;
  double backSubstitute = 0.0;
  double deltaError = 0.0;
  double retractAndError = 0.0;

  double solveTotal() const { return symbolic + eliminate + backSubstitute; }

  double total() const {
    return linearize + hessianDiagonal + damp + solveTotal() + deltaError +
           retractAndError;
  }
};

void writeBenchmarkActionJson(const std::vector<TimingRow>& rows,
                              const std::string& outputPath,
                              bool includeCameraBatch, bool choleskyOnly) {
  std::vector<gtsam::timing::BenchmarkMetric> metrics;
  const auto append = [&](const std::string& name, double value) {
    metrics.push_back({name, "s", value});
  };
  for (const auto& row : rows) {
    const std::string prefix = "timeSFMBAL/" + row.dataset + "/";
    append(prefix + "MultifrontalCholesky", row.regularCholesky);
    if (!choleskyOnly) {
      append(prefix + "MultifrontalQR", row.regularQr);
      append(prefix + "MultifrontalSolver", row.regularSolver);
    }
    append(prefix + "BatchFactor/MultifrontalCholesky", row.batchCholesky);
    if (!choleskyOnly) {
      append(prefix + "BatchFactor/MultifrontalQR", row.batchQr);
      append(prefix + "BatchFactor/MultifrontalSolver", row.batchSolver);
    }
    if (!includeCameraBatch) continue;
    append(prefix + "CameraFirst/MultifrontalCholesky",
           row.cameraFirstCholesky);
    if (!choleskyOnly) {
      append(prefix + "CameraFirst/MultifrontalQR", row.cameraFirstQr);
      append(prefix + "CameraFirst/MultifrontalSolver", row.cameraFirstSolver);
    }
    append(prefix + "CameraBatchFactor/MultifrontalCholesky",
           row.cameraBatchCholesky);
    if (!choleskyOnly) {
      append(prefix + "CameraBatchFactor/MultifrontalQR", row.cameraBatchQr);
      append(prefix + "CameraBatchFactor/MultifrontalSolver",
             row.cameraBatchSolver);
    }
  }
  gtsam::timing::writeBenchmarkActionMetrics(outputPath, metrics);
}

void writePointCholeskyProfileJson(
    const std::vector<PointCholeskyProfileRow>& rows,
    const std::string& outputPath) {
  std::vector<gtsam::timing::BenchmarkMetric> metrics;
  const auto append = [&](const std::string& name, const std::string& unit,
                          double value) {
    metrics.push_back({name, unit, value});
  };
  for (const auto& row : rows) {
    const std::string prefix = "timeSFMBAL/" + row.dataset +
                               "/PointFirstCholeskyProfile/" + row.variant +
                               "/";
    append(prefix + "linearize", "s", row.linearize);
    append(prefix + "hessianDiagonal", "s", row.hessianDiagonal);
    append(prefix + "damp", "s", row.damp);
    append(prefix + "symbolic", "s", row.symbolic);
    append(prefix + "eliminate", "s", row.eliminate);
    append(prefix + "backSubstitute", "s", row.backSubstitute);
    append(prefix + "deltaError", "s", row.deltaError);
    append(prefix + "retractAndError", "s", row.retractAndError);
    append(prefix + "total", "s", row.total());
    append(prefix + "iterations", "count", static_cast<double>(row.iterations));
    append(prefix + "innerIterations", "count",
           static_cast<double>(row.innerIterations));
    append(prefix + "linearFactors", "count",
           static_cast<double>(row.linearFactors));
    append(prefix + "batchDenseScalarEntries", "count",
           static_cast<double>(row.batchDenseScalarEntries));
    append(prefix + "batchStoredScalarEntriesEstimate", "count",
           static_cast<double>(row.batchStoredScalarEntriesEstimate));
  }
  gtsam::timing::writeBenchmarkActionMetrics(outputPath, metrics);
}
RunOptions parseBalFiles(int argc, char* argv[]) {
  gtsam::timing::Arguments arguments(argc, argv);
  RunOptions options;
  options.config.useSchur = !arguments.flag("--colamd");
  options.profile = arguments.flag("--profile");
  options.cameraBatch = arguments.flag("--camera-batch");
  options.choleskyOnly = arguments.flag("--cholesky-only");
  options.profilePointCholesky = arguments.flag("--profile-point-cholesky");
  options.endToEndPcg = arguments.flag("--end-to-end-pcg");
  options.pointBatchSchurCholmodOnly =
      arguments.flag("--point-batch-schur-cholmod-only");
  options.batchChunkSize = arguments.sizeValue("--batch-chunk-size", 0);
  const auto jsonPath = arguments.optionalString("--benchmark-action-json");
  options.benchmarkActionJson = jsonPath.has_value();
  options.benchmarkActionJsonPath = jsonPath.value_or("");
  options.help = arguments.helpRequested();
  options.filenames = arguments.positionals();
  arguments.validateAllConsumed();

  if (options.help) return options;
  if (options.filenames.size() > 1) throw std::runtime_error(usage());
  const bool hasFilename = !options.filenames.empty();
  if (options.profile && hasFilename) throw std::runtime_error(usage());
  if (options.profile &&
      (options.benchmarkActionJson || options.cameraBatch ||
       options.choleskyOnly || options.profilePointCholesky)) {
    throw std::runtime_error(usage());
  }
  if (options.profilePointCholesky && options.cameraBatch) {
    throw std::runtime_error(
        "--profile-point-cholesky profiles the point-first path; do not "
        "combine it with --camera-batch.");
  }
  if (options.cameraBatch && !options.config.useSchur) {
    throw std::runtime_error(
        "--camera-batch uses camera-first ordering; do not combine it with "
        "--colamd.");
  }
  if (options.endToEndPcg &&
      (options.profile || options.benchmarkActionJson || options.cameraBatch ||
       options.choleskyOnly || options.profilePointCholesky ||
       options.batchChunkSize != 0)) {
    throw std::runtime_error(
        "--end-to-end-pcg is a standalone comparison; combine it only with "
        "--colamd or a BAL file.");
  }
  if (options.pointBatchSchurCholmodOnly &&
      (options.profile || options.endToEndPcg ||
       options.benchmarkActionJson || options.cameraBatch ||
       options.choleskyOnly || options.profilePointCholesky ||
       options.batchChunkSize != 0)) {
    throw std::runtime_error(
        "--point-batch-schur-cholmod-only is a standalone comparison; "
        "combine it only with --colamd or a BAL file.");
  }
  if (options.pointBatchSchurCholmodOnly &&
      !bal::cholmodBackendAvailable()) {
    throw std::runtime_error(
        "--point-batch-schur-cholmod-only requires a build with CHOLMOD");
  }

  if (hasFilename) return options;
  if (options.profile) {
    options.filenames = {bal::profileDataset()};
  } else if (options.profilePointCholesky || options.benchmarkActionJson ||
             options.endToEndPcg || options.pointBatchSchurCholmodOnly) {
    options.filenames = {bal::defaultDataset()};
  } else {
    options.filenames = bal::standardDatasets();
  }
  return options;
}
double runSolver(const NonlinearFactorGraph& graph, const Values& initial,
                 const Ordering& ordering,
                 NonlinearOptimizerParams::LinearSolverType solverType,
                 const std::string& label,
                 const bal::BalBenchmarkConfig& config) {
  LevenbergMarquardtParams params =
      bal::makeLevenbergMarquardtParams(config, &ordering);
  params.linearSolverType = solverType;
  if (solverType == NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    params.multifrontalParams.qrMode = MultifrontalParameters::QRMode::Allow;
  }
  LevenbergMarquardtOptimizer lm(graph, initial, params);
  const double elapsed = gtsam::timing::measureSeconds([&] { lm.optimize(); });
  std::cout << "  " << label << ": " << elapsed << " s\n";
  return elapsed;
}

double speedup(double baseline, double candidate) {
  return candidate > 0.0 ? (baseline / candidate) : 0.0;
}

void collectLinearStats(const GaussianFactorGraph& linear,
                        PointCholeskyProfileRow* row) {
  row->linearFactors = linear.size();
  for (const auto& factor : linear) {
    if (!factor) continue;
    if (const auto batch =
            std::dynamic_pointer_cast<BatchJacobianFactorBase>(factor)) {
      row->batchFactors++;
      const size_t rows = batch->rows();
      row->linearRows += rows;

      size_t denseColumns = 1;  // RHS column.
      for (auto it = batch->begin(); it != batch->end(); ++it) {
        denseColumns += static_cast<size_t>(batch->getDim(it));
      }
      row->batchDenseScalarEntries += rows * denseColumns;

      // SFM factors have 2 residual rows and touch one 9D camera plus one 3D
      // point per measurement. This estimates the compact row-sparse payload.
      constexpr size_t kSfmStoredScalarsPerMeasurement = 2 * (9 + 3 + 1);
      row->batchStoredScalarEntriesEstimate +=
          (rows / 2) * kSfmStoredScalarsPerMeasurement;
    } else if (const auto jacobian =
                   std::dynamic_pointer_cast<JacobianFactor>(factor)) {
      row->jacobianFactors++;
      row->linearRows += jacobian->rows();
    }
  }
}

PointCholeskyProfileRow profilePointFirstCholeskyVariant(
    const std::string& dataset, const std::string& variant,
    const NonlinearFactorGraph& graph, const Values& initial,
    const Ordering& ordering, const bal::BalBenchmarkConfig& config) {
  PointCholeskyProfileRow row;
  row.dataset = dataset;
  row.variant = variant;
  row.nonlinearFactors = graph.size();

  LevenbergMarquardtParams params =
      bal::makeLevenbergMarquardtParams(config, &ordering, "SILENT");
  params.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;

  Values values = initial;
  double currentError = graph.error(values);
  row.initialError = currentError;
  row.finalError = currentError;
  double lambda = params.lambdaInitial;
  double currentFactor = params.lambdaFactor;
  std::optional<IndexedJunctionTree> indexedJunctionTree;

  for (size_t iteration = 0; iteration < params.maxIterations; ++iteration) {
    GaussianFactorGraph::shared_ptr linear;
    row.linearize += gtsam::timing::measureSeconds(
        [&] { linear = graph.linearize(values); });
    if (iteration == 0) collectLinearStats(*linear, &row);

    VectorValues sqrtHessianDiagonal;
    row.hessianDiagonal += gtsam::timing::measureSeconds([&] {
      sqrtHessianDiagonal = linear->hessianDiagonal();
      for (auto& [key, value] : sqrtHessianDiagonal) {
        value = value.cwiseMax(params.dampingParams.minDiagonal)
                    .cwiseMin(params.dampingParams.maxDiagonal)
                    .cwiseSqrt();
      }
    });

    bool stepSuccessful = false;
    bool stopSearchingLambda = false;
    double acceptedError = currentError;
    Values acceptedValues;

    while (!stepSuccessful && !stopSearchingLambda) {
      row.innerIterations++;
      GaussianFactorGraph damped;
      row.damp += gtsam::timing::measureSeconds([&] {
        damped = *linear;
        damped.reserve(damped.size() + sqrtHessianDiagonal.size());
        const double sigma = 1.0 / std::sqrt(lambda);
        for (const auto& keyVector : sqrtHessianDiagonal) {
          const Key key = keyVector.first;
          const Vector& diagonal = keyVector.second;
          const size_t dim = static_cast<size_t>(diagonal.size());
          Matrix A = Eigen::DiagonalMatrix<double, Eigen::Dynamic>(diagonal);
          Vector b = Vector::Zero(dim);
          damped.emplace_shared<JacobianFactor>(
              key, A, b, noiseModel::Isotropic::Sigma(dim, sigma));
        }
      });

      if (!indexedJunctionTree) {
        row.symbolic += gtsam::timing::measureSeconds([&] {
          indexedJunctionTree.emplace(
              damped.buildIndexedJunctionTree(ordering));
        });
      }

      std::shared_ptr<GaussianBayesTree> bayesTree;
      row.eliminate += gtsam::timing::measureSeconds([&] {
        bayesTree = damped.eliminateMultifrontal(*indexedJunctionTree,
                                                 EliminatePreferCholesky);
      });

      VectorValues delta;
      row.backSubstitute +=
          gtsam::timing::measureSeconds([&] { delta = bayesTree->optimize(); });

      double linearizedCostChange = 0.0;
      row.deltaError += gtsam::timing::measureSeconds([&] {
        double oldLinearizedError = 0.0, newLinearizedError = 0.0;
        linearizedCostChange =
            linear->deltaError(delta, &oldLinearizedError, &newLinearizedError);
      });

      double newError = std::numeric_limits<double>::infinity();
      Values newValues;
      row.retractAndError += gtsam::timing::measureSeconds([&] {
        newValues = values.retract(delta);
        newError = graph.error(newValues);
      });

      const double costChange = currentError - newError;
      double modelFidelity = 0.0;
      if (linearizedCostChange >= 0.0 &&
          linearizedCostChange >
              std::numeric_limits<double>::epsilon() * currentError) {
        modelFidelity = costChange / linearizedCostChange;
        stepSuccessful = modelFidelity > params.minModelFidelity;
      }

      const double minAbsoluteTolerance =
          params.relativeErrorTol * currentError;
      stopSearchingLambda = std::abs(costChange) < minAbsoluteTolerance;

      if (stepSuccessful) {
        if (params.useFixedLambdaFactor) {
          lambda /= currentFactor;
        } else {
          lambda *=
              std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * modelFidelity - 1.0, 3));
          currentFactor = 2.0 * currentFactor;
        }
        lambda = std::max(params.lambdaLowerBound, lambda);
        acceptedValues = std::move(newValues);
        acceptedError = newError;
      } else if (!stopSearchingLambda) {
        lambda *= currentFactor;
        if (!params.useFixedLambdaFactor) currentFactor *= 2.0;
        if (lambda >= params.lambdaUpperBound) {
          stopSearchingLambda = true;
        }
      }
    }

    row.iterations++;
    const double previousError = currentError;
    if (stepSuccessful) {
      values = std::move(acceptedValues);
      currentError = acceptedError;
      row.finalError = currentError;
    }

    if (std::abs(previousError - currentError) <
        params.relativeErrorTol * previousError) {
      break;
    }
    if (!stepSuccessful) break;
  }

  return row;
}

std::vector<PointCholeskyProfileRow> profilePointFirstCholesky(
    const std::string& filename, size_t batchChunkSize,
    const bal::BalBenchmarkConfig& config) {
  const std::string dataset =
      std::filesystem::path(filename).filename().string();
  std::cout << "\nProfiling point-first Cholesky for BAL file: " << filename
            << std::endl;
  const SfmData db = bal::loadDataset(filename);
  const Values initial = bal::buildGeneralSfmInitial(db);
  const Ordering ordering = bal::createSchurOrdering(db, false);

  const NonlinearFactorGraph regularGraph =
      bal::buildGeneralSfmGraph(db, config);
  const NonlinearFactorGraph batchGraph =
      bal::buildBatchSfmGraph(db, config, false, batchChunkSize);

  std::vector<PointCholeskyProfileRow> rows;
  rows.push_back(profilePointFirstCholeskyVariant(
      dataset, "Regular", regularGraph, initial, ordering, config));
  rows.push_back(profilePointFirstCholeskyVariant(
      dataset, "PointBatch", batchGraph, initial, ordering, config));

  std::cout
      << "\n| Dataset | Variant | Nonlinear factors | Linear factors | "
         "Jacobian factors | Batch factors | Rows | Iterations | Inner its | "
         "Linearize s | Hessian diagonal s | Damping s | Symbolic s | "
         "Eliminate s | Back-substitute s | Delta error s | "
         "Retract+error s | Total s |\n";
  std::cout
      << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | "
         "--- | --- | --- | --- | --- | --- | --- |\n";
  std::cout << std::fixed << std::setprecision(6);
  for (const auto& row : rows) {
    std::cout << "| " << row.dataset << " | " << row.variant << " | "
              << row.nonlinearFactors << " | " << row.linearFactors << " | "
              << row.jacobianFactors << " | " << row.batchFactors << " | "
              << row.linearRows << " | " << row.iterations << " | "
              << row.innerIterations << " | " << row.linearize << " | "
              << row.hessianDiagonal << " | " << row.damp << " | "
              << row.symbolic << " | " << row.eliminate << " | "
              << row.backSubstitute << " | " << row.deltaError << " | "
              << row.retractAndError << " | " << row.total() << " |\n";
  }

  std::cout << "\n| Dataset | Variant | Batch dense scalar entries | "
               "Estimated stored scalar entries | Dense/stored ratio |\n";
  std::cout << "| --- | --- | --- | --- | --- |\n";
  for (const auto& row : rows) {
    const double ratio =
        row.batchStoredScalarEntriesEstimate > 0
            ? static_cast<double>(row.batchDenseScalarEntries) /
                  static_cast<double>(row.batchStoredScalarEntriesEstimate)
            : 0.0;
    std::cout << "| " << row.dataset << " | " << row.variant << " | "
              << row.batchDenseScalarEntries << " | "
              << row.batchStoredScalarEntriesEstimate << " | " << ratio
              << "x |\n";
  }

  return rows;
}
}  // namespace

int main(int argc, char* argv[]) {
  const auto options = parseBalFiles(argc, argv);
  if (options.help) {
    std::cout << usage() << '\n';
    return 0;
  }

  if (options.endToEndPcg) {
    bal::runEndToEndPcgComparison(options.filenames, options.config);
    return 0;
  }

  if (options.pointBatchSchurCholmodOnly) {
    for (const auto& filename : options.filenames) {
      const SfmData data = bal::loadDataset(filename);
      NonlinearFactorGraph graph;
      const double graphBuildSeconds = gtsam::timing::measureSeconds([&] {
        graph = bal::buildBatchSfmGraph(data, options.config, false, 0);
      });
      const Values initial = bal::buildGeneralSfmInitial(data);
      const Ordering ordering = bal::createSchurOrdering(data, false);
      LevenbergMarquardtParams parameters =
          bal::makeLevenbergMarquardtParams(options.config, &ordering,
                                            "SILENT");
      const bal::SparseSchurOptimizationResult result =
          bal::runPointBatchSchurCholmodOptimization(graph, initial,
                                                      parameters);
      std::cout << std::fixed << std::setprecision(6)
                << "\n| Dataset | Solver | Graph build s | Optimize s | "
                   "Initial error | Final error | LM iterations | "
                   "LM inner iterations | Linear solves | Assembly s | "
                   "Factor + solve s | Back-substitute s |\n"
                << "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | "
                   "---: | ---: | ---: | ---: |\n"
                << "| " << std::filesystem::path(filename).filename().string()
                << " | PointBatch/SparseSchur/CHOLMOD | "
                << graphBuildSeconds << " | " << result.elapsedSeconds << " | "
                << result.initialError << " | " << result.finalError << " | "
                << result.lmIterations << " | " << result.lmInnerIterations
                << " | " << result.linearSolves << " | "
                << result.assemblySeconds << " | "
                << result.factorAndSolveSeconds << " | "
                << result.backSubstituteSeconds << " |\n";
    }
    return 0;
  }

  if (options.profilePointCholesky) {
    std::vector<PointCholeskyProfileRow> profileRows;
    for (const auto& filename : options.filenames) {
      std::vector<PointCholeskyProfileRow> rows = profilePointFirstCholesky(
          filename, options.batchChunkSize, options.config);
      profileRows.insert(profileRows.end(), rows.begin(), rows.end());
    }
    if (options.benchmarkActionJson) {
      writePointCholeskyProfileJson(profileRows,
                                    options.benchmarkActionJsonPath);
      std::cout << "\nWrote point-first Cholesky profile JSON to "
                << options.benchmarkActionJsonPath << "\n";
    }
    return 0;
  }

  std::vector<TimingRow> rows;

  for (const auto& filename : options.filenames) {
    const std::string dataset =
        std::filesystem::path(filename).filename().string();
    std::cout << "\nProcessing BAL file: " << filename << std::endl;
    const SfmData db = bal::loadDataset(filename);

    NonlinearFactorGraph graph = bal::buildGeneralSfmGraph(db, options.config);
    Values initial = bal::buildGeneralSfmInitial(db);

    Ordering ordering;
    Ordering cameraFirstOrdering;
    if (options.config.useSchur) {
      ordering = bal::createSchurOrdering(db, false);
      if (options.cameraBatch) {
        cameraFirstOrdering = bal::createCameraFirstOrdering(db);
      }
    }

    double newTime = 0.0;
    if (!options.choleskyOnly) {
      newTime = runSolver(graph, initial, ordering,
                          NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                          "MultifrontalSolver", options.config);
    }
    double legacyTime = 0.0;
    double legacyQrTime = 0.0;
    double batchLegacyTime = 0.0;
    double batchQrTime = 0.0;
    double batchTime = 0.0;
    double cameraFirstLegacyTime = 0.0;
    double cameraFirstQrTime = 0.0;
    double cameraFirstSolverTime = 0.0;
    double cameraBatchLegacyTime = 0.0;
    double cameraBatchQrTime = 0.0;
    double cameraBatchSolverTime = 0.0;
    if (!options.profile) {
      legacyTime = runSolver(graph, initial, ordering,
                             NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                             "MultifrontalCholesky", options.config);
      if (!options.choleskyOnly) {
        legacyQrTime = runSolver(graph, initial, ordering,
                                 NonlinearOptimizerParams::MULTIFRONTAL_QR,
                                 "MultifrontalQR", options.config);
      }
      const NonlinearFactorGraph batchGraph = bal::buildBatchSfmGraph(
          db, options.config, false, options.batchChunkSize);
      batchLegacyTime =
          runSolver(batchGraph, initial, ordering,
                    NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                    "BatchFactor/MultifrontalCholesky", options.config);
      if (!options.choleskyOnly) {
        batchQrTime = runSolver(batchGraph, initial, ordering,
                                NonlinearOptimizerParams::MULTIFRONTAL_QR,
                                "BatchFactor/MultifrontalQR", options.config);
        batchTime = runSolver(batchGraph, initial, ordering,
                              NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                              "BatchFactor/MultifrontalSolver", options.config);
      }

      if (options.cameraBatch) {
        const NonlinearFactorGraph cameraBatchGraph =
            bal::buildCameraBatchSfmGraph(db, options.config);
        cameraBatchLegacyTime =
            runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                      NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                      "CameraBatchFactor/MultifrontalCholesky", options.config);
        if (!options.choleskyOnly) {
          cameraBatchQrTime =
              runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_QR,
                        "CameraBatchFactor/MultifrontalQR", options.config);
          cameraBatchSolverTime =
              runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                        "CameraBatchFactor/MultifrontalSolver", options.config);
        }

        cameraFirstLegacyTime =
            runSolver(graph, initial, cameraFirstOrdering,
                      NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                      "CameraFirst/MultifrontalCholesky", options.config);
        if (!options.choleskyOnly) {
          cameraFirstQrTime =
              runSolver(graph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_QR,
                        "CameraFirst/MultifrontalQR", options.config);
          cameraFirstSolverTime =
              runSolver(graph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                        "CameraFirst/MultifrontalSolver", options.config);
        }
      }
    }

    if (!options.profile) {
      rows.push_back({dataset, legacyTime, legacyQrTime, newTime,
                      batchLegacyTime, batchQrTime, batchTime,
                      cameraFirstLegacyTime, cameraFirstQrTime,
                      cameraFirstSolverTime, cameraBatchLegacyTime,
                      cameraBatchQrTime, cameraBatchSolverTime});
    }
  }
  if (!options.profile) {
    std::cout << std::fixed << std::setprecision(3);
    if (options.choleskyOnly) {
      std::cout << "\n| Dataset | Regular Cholesky s | "
                   "Batch Cholesky s | Cholesky Speedup |\n";
      std::cout << "| --- | --- | --- | --- |\n";
      for (const auto& row : rows) {
        std::cout << "| " << row.dataset << " | " << row.regularCholesky
                  << " | " << row.batchCholesky << " | "
                  << speedup(row.regularCholesky, row.batchCholesky) << "x |\n";
      }
    } else {
      std::cout << "\n| Dataset | Regular Cholesky s | Regular QR s | "
                   "Regular Solver s | Batch Cholesky s | Batch QR s | "
                   "Batch Solver s | Batch Solver Speedup |\n";
      std::cout << "| --- | --- | --- | --- | --- | --- | --- | --- |\n";
      for (const auto& row : rows) {
        std::cout << "| " << row.dataset << " | " << row.regularCholesky
                  << " | " << row.regularQr << " | " << row.regularSolver
                  << " | " << row.batchCholesky << " | " << row.batchQr << " | "
                  << row.batchSolver << " | "
                  << speedup(row.regularSolver, row.batchSolver) << "x |\n";
      }
    }

    if (options.cameraBatch) {
      if (options.choleskyOnly) {
        std::cout << "\n| Dataset | Camera-first Cholesky s | "
                     "Camera-batch Cholesky s | Cholesky Speedup |\n";
        std::cout << "| --- | --- | --- | --- |\n";
        for (const auto& row : rows) {
          std::cout << "| " << row.dataset << " | " << row.cameraFirstCholesky
                    << " | " << row.cameraBatchCholesky << " | "
                    << speedup(row.cameraFirstCholesky, row.cameraBatchCholesky)
                    << "x |\n";
        }
      } else {
        std::cout
            << "\n| Dataset | Camera-first Cholesky s | Camera-first QR s | "
               "Camera-first Solver s | Camera-batch Cholesky s | "
               "Camera-batch QR s | Camera-batch Solver s | Cholesky Speedup | "
               "QR Speedup | Solver Speedup |\n";
        std::cout << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | "
                     "--- |\n";
        for (const auto& row : rows) {
          std::cout << "| " << row.dataset << " | " << row.cameraFirstCholesky
                    << " | " << row.cameraFirstQr << " | "
                    << row.cameraFirstSolver << " | " << row.cameraBatchCholesky
                    << " | " << row.cameraBatchQr << " | "
                    << row.cameraBatchSolver << " | "
                    << speedup(row.cameraFirstCholesky, row.cameraBatchCholesky)
                    << "x | " << speedup(row.cameraFirstQr, row.cameraBatchQr)
                    << "x | "
                    << speedup(row.cameraFirstSolver, row.cameraBatchSolver)
                    << "x |\n";
        }
      }
    }
  }

  if (options.benchmarkActionJson) {
    if (rows.empty()) {
      throw std::runtime_error("No benchmark rows found to write.");
    }
    writeBenchmarkActionJson(rows, options.benchmarkActionJsonPath,
                             options.cameraBatch, options.choleskyOnly);
    std::cout << "\nWrote benchmark-action JSON to "
              << options.benchmarkActionJsonPath << "\n";
  }
  return 0;
}
