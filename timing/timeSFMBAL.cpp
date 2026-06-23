/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
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

#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/BatchFactor.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>

namespace {
constexpr const char* kDefaultBenchmarkDataset = "dubrovnik-16-22106-pre";
constexpr const char* kProfileDataset = "dubrovnik-135-90642-pre";

using Camera = PinholeCamera<Cal3Bundler>;
using SfmFactor = GeneralSFMFactor<Camera, Point3>;

std::string usage() {
  return "Usage: timeSFMBAL [--colamd] [--profile] [--camera-batch] "
         "[--cholesky-only] [--profile-point-cholesky] "
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
  bool profile = false;
  bool benchmarkActionJson = false;
  bool cameraBatch = false;
  bool choleskyOnly = false;
  bool profilePointCholesky = false;
  std::string benchmarkActionJsonPath;
  std::vector<std::string> filenames;
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

std::string escapeJson(std::string value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += c;
        break;
    }
  }
  return escaped;
}

void writeBenchmarkActionJson(const std::vector<TimingRow>& rows,
                              const std::string& outputPath,
                              bool includeCameraBatch, bool choleskyOnly) {
  std::ofstream out(outputPath);
  if (!out) {
    throw runtime_error("Unable to open benchmark JSON output file: " +
                        outputPath);
  }

  out << "[\n";
  bool first = true;
  const auto appendEntry = [&](const std::string& name, const double value) {
    if (!first) out << ",\n";
    first = false;
    out << "  {\n";
    out << "    \"name\": \"" << escapeJson(name) << "\",\n";
    out << "    \"unit\": \"s\",\n";
    out << "    \"value\": " << std::fixed << std::setprecision(9) << value
        << "\n";
    out << "  }";
  };

  for (const auto& row : rows) {
    appendEntry("timeSFMBAL/" + row.dataset + "/MultifrontalCholesky",
                row.regularCholesky);
    if (!choleskyOnly) {
      appendEntry("timeSFMBAL/" + row.dataset + "/MultifrontalQR",
                  row.regularQr);
      appendEntry("timeSFMBAL/" + row.dataset + "/MultifrontalSolver",
                  row.regularSolver);
    }
    appendEntry(
        "timeSFMBAL/" + row.dataset + "/BatchFactor/MultifrontalCholesky",
        row.batchCholesky);
    if (!choleskyOnly) {
      appendEntry("timeSFMBAL/" + row.dataset + "/BatchFactor/MultifrontalQR",
                  row.batchQr);
      appendEntry(
          "timeSFMBAL/" + row.dataset + "/BatchFactor/MultifrontalSolver",
          row.batchSolver);
    }
    if (!includeCameraBatch) continue;
    appendEntry("timeSFMBAL/" + row.dataset +
                    "/CameraFirst/MultifrontalCholesky",
                row.cameraFirstCholesky);
    if (!choleskyOnly) {
      appendEntry("timeSFMBAL/" + row.dataset + "/CameraFirst/MultifrontalQR",
                  row.cameraFirstQr);
      appendEntry(
          "timeSFMBAL/" + row.dataset + "/CameraFirst/MultifrontalSolver",
          row.cameraFirstSolver);
    }
    appendEntry("timeSFMBAL/" + row.dataset +
                    "/CameraBatchFactor/MultifrontalCholesky",
                row.cameraBatchCholesky);
    if (!choleskyOnly) {
      appendEntry("timeSFMBAL/" + row.dataset +
                      "/CameraBatchFactor/MultifrontalQR",
                  row.cameraBatchQr);
      appendEntry("timeSFMBAL/" + row.dataset +
                      "/CameraBatchFactor/MultifrontalSolver",
                  row.cameraBatchSolver);
    }
  }
  out << "\n]\n";
}

void writePointCholeskyProfileJson(
    const std::vector<PointCholeskyProfileRow>& rows,
    const std::string& outputPath) {
  std::ofstream out(outputPath);
  if (!out) {
    throw runtime_error("Unable to open profile JSON output file: " +
                        outputPath);
  }

  out << "[\n";
  bool first = true;
  const auto appendEntry = [&](const std::string& name, const std::string& unit,
                               const double value) {
    if (!first) out << ",\n";
    first = false;
    out << "  {\n";
    out << "    \"name\": \"" << escapeJson(name) << "\",\n";
    out << "    \"unit\": \"" << escapeJson(unit) << "\",\n";
    out << "    \"value\": " << std::fixed << std::setprecision(9) << value
        << "\n";
    out << "  }";
  };

  for (const auto& row : rows) {
    const std::string prefix = "timeSFMBAL/" + row.dataset +
                               "/PointFirstCholeskyProfile/" + row.variant +
                               "/";
    appendEntry(prefix + "linearize", "s", row.linearize);
    appendEntry(prefix + "hessianDiagonal", "s", row.hessianDiagonal);
    appendEntry(prefix + "damp", "s", row.damp);
    appendEntry(prefix + "symbolic", "s", row.symbolic);
    appendEntry(prefix + "eliminate", "s", row.eliminate);
    appendEntry(prefix + "backSubstitute", "s", row.backSubstitute);
    appendEntry(prefix + "deltaError", "s", row.deltaError);
    appendEntry(prefix + "retractAndError", "s", row.retractAndError);
    appendEntry(prefix + "total", "s", row.total());
    appendEntry(prefix + "iterations", "count",
                static_cast<double>(row.iterations));
    appendEntry(prefix + "innerIterations", "count",
                static_cast<double>(row.innerIterations));
    appendEntry(prefix + "linearFactors", "count",
                static_cast<double>(row.linearFactors));
    appendEntry(prefix + "batchDenseScalarEntries", "count",
                static_cast<double>(row.batchDenseScalarEntries));
    appendEntry(prefix + "batchStoredScalarEntriesEstimate", "count",
                static_cast<double>(row.batchStoredScalarEntriesEstimate));
  }
  out << "\n]\n";
}

RunOptions parseBalFiles(int argc, char* argv[]) {
  std::string filename;
  bool profile = false;
  bool benchmarkActionJson = false;
  bool cameraBatch = false;
  bool choleskyOnly = false;
  bool profilePointCholesky = false;
  bool colamd = false;
  std::string benchmarkActionJsonPath;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--colamd") == 0) {
      gUseSchur = false;
      colamd = true;
      continue;
    }
    if (strcmp(argv[i], "--profile") == 0) {
      profile = true;
      continue;
    }
    if (strcmp(argv[i], "--camera-batch") == 0) {
      cameraBatch = true;
      continue;
    }
    if (strcmp(argv[i], "--cholesky-only") == 0) {
      choleskyOnly = true;
      continue;
    }
    if (strcmp(argv[i], "--profile-point-cholesky") == 0) {
      profilePointCholesky = true;
      continue;
    }
    if (strcmp(argv[i], "--benchmark-action-json") == 0) {
      if (++i >= argc || argv[i][0] == '-') {
        throw runtime_error(usage());
      }
      benchmarkActionJson = true;
      benchmarkActionJsonPath = argv[i];
      continue;
    }
    if (argv[i][0] == '-') {
      throw runtime_error(usage());
    }
    if (!filename.empty()) {
      throw runtime_error(usage());
    }
    filename = argv[i];
  }

  if (profile && !filename.empty()) {
    throw runtime_error(usage());
  }
  if (profile && (benchmarkActionJson || cameraBatch || choleskyOnly ||
                  profilePointCholesky)) {
    throw runtime_error(usage());
  }
  if (profilePointCholesky && cameraBatch) {
    throw runtime_error("--profile-point-cholesky profiles the point-first "
                        "path; do not combine it with --camera-batch.");
  }
  if (cameraBatch && colamd) {
    throw runtime_error("--camera-batch uses camera-first ordering; do not "
                        "combine it with --colamd.");
  }

  if (!filename.empty()) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            choleskyOnly,
            profilePointCholesky,
            benchmarkActionJsonPath,
            {filename}};
  }

  if (profile) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            choleskyOnly,
            profilePointCholesky,
            benchmarkActionJsonPath,
            {findExampleDataFile(kProfileDataset)}};
  }

  if (profilePointCholesky) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            choleskyOnly,
            profilePointCholesky,
            benchmarkActionJsonPath,
            {findExampleDataFile(kDefaultBenchmarkDataset)}};
  }

  if (benchmarkActionJson) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            choleskyOnly,
            profilePointCholesky,
            benchmarkActionJsonPath,
            {findExampleDataFile(kDefaultBenchmarkDataset)}};
  }

  return {profile,
          benchmarkActionJson,
          cameraBatch,
          choleskyOnly,
          profilePointCholesky,
          benchmarkActionJsonPath,
          {
              findExampleDataFile("dubrovnik-16-22106-pre"),
              findExampleDataFile("dubrovnik-88-64298-pre"),
              findExampleDataFile("dubrovnik-135-90642-pre"),
          }};
}

double runSolver(const NonlinearFactorGraph& graph, const Values& initial,
                 const Ordering& ordering,
                 NonlinearOptimizerParams::LinearSolverType solverType,
                 const std::string& label) {
  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.setVerbosityLM("SUMMARY");
  params.setRelativeErrorTol(0.01);
  params.linearSolverType = solverType;
  if (solverType == NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    params.multifrontalParams.qrMode = MultifrontalParameters::QRMode::Allow;
  }
  if (gUseSchur) {
    params.setOrdering(ordering);
  }

  auto start = std::chrono::high_resolution_clock::now();
  LevenbergMarquardtOptimizer lm(graph, initial, params);
  lm.optimize();
  auto end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed = end - start;
  std::cout << "  " << label << ": " << elapsed.count() << " s\n";
  return elapsed.count();
}

NonlinearFactorGraph buildBatchSfmGraph(const SfmData& db,
                                        bool useHessianFactor) {
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.numberTracks(); j++) {
    const auto& measurementsForTrack = db.tracks[j].measurements;
    if (measurementsForTrack.size() < 2) continue;

    std::map<Key, Point2> measurements;
    for (const SfmMeasurement& measurement : measurementsForTrack) {
      measurements[C(measurement.first)] = measurement.second;
    }

    auto batch = std::make_shared<BatchFactor<SfmFactor, 2>>(measurements, P(j),
                                                             gNoiseModel);
    batch->setUseHessianFactor(useHessianFactor);
    graph.add(batch);
  }
  return graph;
}

NonlinearFactorGraph buildCameraBatchSfmGraph(const SfmData& db) {
  NonlinearFactorGraph graph;
  std::vector<std::map<Key, Point2>> measurementsByCamera(db.numberCameras());

  for (size_t j = 0; j < db.numberTracks(); j++) {
    const auto& measurementsForTrack = db.tracks[j].measurements;
    if (measurementsForTrack.size() < 2) continue;

    for (const SfmMeasurement& measurement : measurementsForTrack) {
      measurementsByCamera[measurement.first][P(j)] = measurement.second;
    }
  }

  for (size_t i = 0; i < measurementsByCamera.size(); ++i) {
    const auto& measurements = measurementsByCamera[i];
    if (measurements.empty()) continue;

    graph.add(std::make_shared<BatchFactor<SfmFactor, 2>>(C(i), measurements,
                                                          gNoiseModel));
  }
  return graph;
}

Ordering createCameraFirstOrdering(const SfmData& db) {
  Ordering ordering;
  for (size_t i = 0; i < db.numberCameras(); i++) ordering.push_back(C(i));
  for (size_t j = 0; j < db.numberTracks(); j++) ordering.push_back(P(j));
  return ordering;
}

double speedup(double baseline, double candidate) {
  return candidate > 0.0 ? (baseline / candidate) : 0.0;
}

template <typename Callable>
double measureSeconds(Callable&& callable) {
  const auto start = std::chrono::high_resolution_clock::now();
  callable();
  const auto end = std::chrono::high_resolution_clock::now();
  return std::chrono::duration<double>(end - start).count();
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
    const Ordering& ordering) {
  PointCholeskyProfileRow row;
  row.dataset = dataset;
  row.variant = variant;
  row.nonlinearFactors = graph.size();

  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.setVerbosityLM("SILENT");
  params.setRelativeErrorTol(0.01);
  params.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;
  params.setOrdering(ordering);

  Values values = initial;
  double currentError = graph.error(values);
  row.initialError = currentError;
  row.finalError = currentError;
  double lambda = params.lambdaInitial;
  double currentFactor = params.lambdaFactor;
  std::optional<IndexedJunctionTree> indexedJunctionTree;

  for (size_t iteration = 0; iteration < params.maxIterations; ++iteration) {
    GaussianFactorGraph::shared_ptr linear;
    row.linearize += measureSeconds([&] { linear = graph.linearize(values); });
    if (iteration == 0) collectLinearStats(*linear, &row);

    VectorValues sqrtHessianDiagonal;
    row.hessianDiagonal += measureSeconds([&] {
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
      row.damp += measureSeconds([&] {
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
        row.symbolic += measureSeconds([&] {
          indexedJunctionTree.emplace(damped.buildIndexedJunctionTree(ordering));
        });
      }

      std::shared_ptr<GaussianBayesTree> bayesTree;
      row.eliminate += measureSeconds([&] {
        bayesTree = damped.eliminateMultifrontal(*indexedJunctionTree,
                                                 EliminatePreferCholesky);
      });

      VectorValues delta;
      row.backSubstitute +=
          measureSeconds([&] { delta = bayesTree->optimize(); });

      double linearizedCostChange = 0.0;
      row.deltaError += measureSeconds([&] {
        double oldLinearizedError = 0.0, newLinearizedError = 0.0;
        linearizedCostChange =
            linear->deltaError(delta, &oldLinearizedError, &newLinearizedError);
      });

      double newError = std::numeric_limits<double>::infinity();
      Values newValues;
      row.retractAndError += measureSeconds([&] {
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
          lambda *= std::max(1.0 / 3.0,
                             1.0 - std::pow(2.0 * modelFidelity - 1.0, 3));
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
    const std::string& filename) {
  const std::string dataset = std::filesystem::path(filename).filename().string();
  std::cout << "\nProfiling point-first Cholesky for BAL file: " << filename
            << std::endl;
  const SfmData db = SfmData::FromBalFile(filename);
  const Values initial = buildGeneralSfmInitial(db);
  const Ordering ordering = createSchurOrdering(db, false);

  const NonlinearFactorGraph regularGraph = buildGeneralSfmGraph(db);
  const NonlinearFactorGraph batchGraph = buildBatchSfmGraph(db, false);

  std::vector<PointCholeskyProfileRow> rows;
  rows.push_back(profilePointFirstCholeskyVariant(
      dataset, "Regular", regularGraph, initial, ordering));
  rows.push_back(profilePointFirstCholeskyVariant(
      dataset, "PointBatch", batchGraph, initial, ordering));

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

  std::cout
      << "\n| Dataset | Variant | Batch dense scalar entries | "
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

  if (options.profilePointCholesky) {
    std::vector<PointCholeskyProfileRow> profileRows;
    for (const auto& filename : options.filenames) {
      std::vector<PointCholeskyProfileRow> rows =
          profilePointFirstCholesky(filename);
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
    const SfmData db = SfmData::FromBalFile(filename);

    NonlinearFactorGraph graph = buildGeneralSfmGraph(db);
    Values initial = buildGeneralSfmInitial(db);

    Ordering ordering;
    Ordering cameraFirstOrdering;
    if (gUseSchur) {
      ordering = createSchurOrdering(db, false);
      if (options.cameraBatch) {
        cameraFirstOrdering = createCameraFirstOrdering(db);
      }
    }

    double newTime = 0.0;
    if (!options.choleskyOnly) {
      newTime = runSolver(graph, initial, ordering,
                          NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                          "MultifrontalSolver");
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
                             "MultifrontalCholesky");
      if (!options.choleskyOnly) {
        legacyQrTime = runSolver(graph, initial, ordering,
                                 NonlinearOptimizerParams::MULTIFRONTAL_QR,
                                 "MultifrontalQR");
      }
      const NonlinearFactorGraph batchGraph = buildBatchSfmGraph(db, false);
      batchLegacyTime =
          runSolver(batchGraph, initial, ordering,
                    NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                    "BatchFactor/MultifrontalCholesky");
      if (!options.choleskyOnly) {
        batchQrTime = runSolver(batchGraph, initial, ordering,
                                NonlinearOptimizerParams::MULTIFRONTAL_QR,
                                "BatchFactor/MultifrontalQR");
        batchTime = runSolver(batchGraph, initial, ordering,
                              NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                              "BatchFactor/MultifrontalSolver");
      }

      if (options.cameraBatch) {
        const NonlinearFactorGraph cameraBatchGraph =
            buildCameraBatchSfmGraph(db);
        cameraBatchLegacyTime =
            runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                      NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                      "CameraBatchFactor/MultifrontalCholesky");
        if (!options.choleskyOnly) {
          cameraBatchQrTime =
              runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_QR,
                        "CameraBatchFactor/MultifrontalQR");
          cameraBatchSolverTime =
              runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                        "CameraBatchFactor/MultifrontalSolver");
        }

        cameraFirstLegacyTime =
            runSolver(graph, initial, cameraFirstOrdering,
                      NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                      "CameraFirst/MultifrontalCholesky");
        if (!options.choleskyOnly) {
          cameraFirstQrTime =
              runSolver(graph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_QR,
                        "CameraFirst/MultifrontalQR");
          cameraFirstSolverTime =
              runSolver(graph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                        "CameraFirst/MultifrontalSolver");
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
                  << speedup(row.regularCholesky, row.batchCholesky)
                  << "x |\n";
      }
    } else {
      std::cout << "\n| Dataset | Regular Cholesky s | Regular QR s | "
                   "Regular Solver s | Batch Cholesky s | Batch QR s | "
                   "Batch Solver s | Batch Solver Speedup |\n";
      std::cout << "| --- | --- | --- | --- | --- | --- | --- | --- |\n";
      for (const auto& row : rows) {
        std::cout << "| " << row.dataset << " | " << row.regularCholesky
                  << " | " << row.regularQr << " | " << row.regularSolver
                  << " | " << row.batchCholesky << " | " << row.batchQr
                  << " | " << row.batchSolver << " | "
                  << speedup(row.regularSolver, row.batchSolver) << "x |\n";
      }
    }

    if (options.cameraBatch) {
      if (options.choleskyOnly) {
        std::cout << "\n| Dataset | Camera-first Cholesky s | "
                     "Camera-batch Cholesky s | Cholesky Speedup |\n";
        std::cout << "| --- | --- | --- | --- |\n";
        for (const auto& row : rows) {
          std::cout << "| " << row.dataset << " | "
                    << row.cameraFirstCholesky << " | "
                    << row.cameraBatchCholesky << " | "
                    << speedup(row.cameraFirstCholesky,
                               row.cameraBatchCholesky)
                    << "x |\n";
        }
      } else {
        std::cout
            << "\n| Dataset | Camera-first Cholesky s | Camera-first QR s | "
               "Camera-first Solver s | Camera-batch Cholesky s | "
               "Camera-batch QR s | Camera-batch Solver s | Cholesky Speedup | "
               "QR Speedup | Solver Speedup |\n";
        std::cout
            << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n";
        for (const auto& row : rows) {
          std::cout << "| " << row.dataset << " | " << row.cameraFirstCholesky
                    << " | " << row.cameraFirstQr << " | "
                    << row.cameraFirstSolver << " | "
                    << row.cameraBatchCholesky << " | " << row.cameraBatchQr
                    << " | " << row.cameraBatchSolver << " | "
                    << speedup(row.cameraFirstCholesky,
                               row.cameraBatchCholesky)
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
      throw runtime_error("No benchmark rows found to write.");
    }
    writeBenchmarkActionJson(rows, options.benchmarkActionJsonPath,
                             options.cameraBatch, options.choleskyOnly);
    std::cout << "\nWrote benchmark-action JSON to "
              << options.benchmarkActionJsonPath << "\n";
  }
  return 0;
}
