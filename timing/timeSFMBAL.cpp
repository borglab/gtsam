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

#include <gtsam/nonlinear/BatchFactor.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>

namespace {
constexpr const char* kDefaultBenchmarkDataset = "dubrovnik-16-22106-pre";
constexpr const char* kProfileDataset = "dubrovnik-135-90642-pre";

using Camera = PinholeCamera<Cal3Bundler>;
using SfmFactor = GeneralSFMFactor<Camera, Point3>;

std::string usage() {
  return "Usage: timeSFMBAL [--colamd] [--profile] [--camera-batch] "
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
  std::string benchmarkActionJsonPath;
  std::vector<std::string> filenames;
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
                              bool includeCameraBatch) {
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
    appendEntry("timeSFMBAL/" + row.dataset + "/MultifrontalQR", row.regularQr);
    appendEntry("timeSFMBAL/" + row.dataset + "/MultifrontalSolver",
                row.regularSolver);
    appendEntry(
        "timeSFMBAL/" + row.dataset + "/BatchFactor/MultifrontalCholesky",
        row.batchCholesky);
    appendEntry("timeSFMBAL/" + row.dataset + "/BatchFactor/MultifrontalQR",
                row.batchQr);
    appendEntry("timeSFMBAL/" + row.dataset + "/BatchFactor/MultifrontalSolver",
                row.batchSolver);
    if (!includeCameraBatch) continue;
    appendEntry("timeSFMBAL/" + row.dataset +
                    "/CameraFirst/MultifrontalCholesky",
                row.cameraFirstCholesky);
    appendEntry("timeSFMBAL/" + row.dataset + "/CameraFirst/MultifrontalQR",
                row.cameraFirstQr);
    appendEntry("timeSFMBAL/" + row.dataset + "/CameraFirst/MultifrontalSolver",
                row.cameraFirstSolver);
    appendEntry("timeSFMBAL/" + row.dataset +
                    "/CameraBatchFactor/MultifrontalCholesky",
                row.cameraBatchCholesky);
    appendEntry("timeSFMBAL/" + row.dataset +
                    "/CameraBatchFactor/MultifrontalQR",
                row.cameraBatchQr);
    appendEntry("timeSFMBAL/" + row.dataset +
                    "/CameraBatchFactor/MultifrontalSolver",
                row.cameraBatchSolver);
  }
  out << "\n]\n";
}

RunOptions parseBalFiles(int argc, char* argv[]) {
  std::string filename;
  bool profile = false;
  bool benchmarkActionJson = false;
  bool cameraBatch = false;
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
  if (profile && (benchmarkActionJson || cameraBatch)) {
    throw runtime_error(usage());
  }
  if (cameraBatch && colamd) {
    throw runtime_error("--camera-batch uses camera-first ordering; do not "
                        "combine it with --colamd.");
  }

  if (!filename.empty()) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            benchmarkActionJsonPath,
            {filename}};
  }

  if (profile) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            benchmarkActionJsonPath,
            {findExampleDataFile(kProfileDataset)}};
  }

  if (benchmarkActionJson) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            benchmarkActionJsonPath,
            {findExampleDataFile(kDefaultBenchmarkDataset)}};
  }

  return {profile,
          benchmarkActionJson,
          cameraBatch,
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
}  // namespace

int main(int argc, char* argv[]) {
  const auto options = parseBalFiles(argc, argv);
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

    const double newTime = runSolver(
        graph, initial, ordering, NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
        "MultifrontalSolver");
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
      legacyQrTime = runSolver(graph, initial, ordering,
                               NonlinearOptimizerParams::MULTIFRONTAL_QR,
                               "MultifrontalQR");
      const NonlinearFactorGraph batchGraph = buildBatchSfmGraph(db, false);
      batchLegacyTime =
          runSolver(batchGraph, initial, ordering,
                    NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                    "BatchFactor/MultifrontalCholesky");
      batchQrTime = runSolver(batchGraph, initial, ordering,
                              NonlinearOptimizerParams::MULTIFRONTAL_QR,
                              "BatchFactor/MultifrontalQR");
      batchTime = runSolver(batchGraph, initial, ordering,
                            NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                            "BatchFactor/MultifrontalSolver");

      if (options.cameraBatch) {
        const NonlinearFactorGraph cameraBatchGraph =
            buildCameraBatchSfmGraph(db);
        cameraBatchSolverTime =
            runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                      NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                      "CameraBatchFactor/MultifrontalSolver");
        cameraBatchLegacyTime =
            runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                      NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                      "CameraBatchFactor/MultifrontalCholesky");
        cameraBatchQrTime =
            runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                      NonlinearOptimizerParams::MULTIFRONTAL_QR,
                      "CameraBatchFactor/MultifrontalQR");

        cameraFirstLegacyTime =
            runSolver(graph, initial, cameraFirstOrdering,
                      NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                      "CameraFirst/MultifrontalCholesky");
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

    if (!options.profile) {
      rows.push_back({dataset, legacyTime, legacyQrTime, newTime,
                      batchLegacyTime, batchQrTime, batchTime,
                      cameraFirstLegacyTime, cameraFirstQrTime,
                      cameraFirstSolverTime, cameraBatchLegacyTime,
                      cameraBatchQrTime, cameraBatchSolverTime});
    }
  }
  if (!options.profile) {
    std::cout << "\n| Dataset | Regular Cholesky s | Regular QR s | "
                 "Regular Solver s | Batch Cholesky s | Batch QR s | "
                 "Batch Solver s | Batch Solver Speedup |\n";
    std::cout << "| --- | --- | --- | --- | --- | --- | --- | --- |\n";
    std::cout << std::fixed << std::setprecision(3);
    for (const auto& row : rows) {
      std::cout << "| " << row.dataset << " | " << row.regularCholesky << " | "
                << row.regularQr << " | " << row.regularSolver << " | "
                << row.batchCholesky << " | " << row.batchQr << " | "
                << row.batchSolver << " | "
                << speedup(row.regularSolver, row.batchSolver) << "x |\n";
    }

    if (options.cameraBatch) {
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

  if (options.benchmarkActionJson) {
    if (rows.empty()) {
      throw runtime_error("No benchmark rows found to write.");
    }
    writeBenchmarkActionJson(rows, options.benchmarkActionJsonPath,
                             options.cameraBatch);
    std::cout << "\nWrote benchmark-action JSON to "
              << options.benchmarkActionJsonPath << "\n";
  }
  return 0;
}
