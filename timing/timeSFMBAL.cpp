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
  return "Usage: timeSFMBAL [--colamd] [--profile] "
         "[--benchmark-action-json FILE] [BALfile]";
}

struct TimingRow {
  std::string dataset;
  double regularCholesky = 0.0;
  double regularSolver = 0.0;
  double batchSolver = 0.0;
  double batchHessianSolver = 0.0;
};

struct RunOptions {
  bool profile = false;
  bool benchmarkActionJson = false;
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
                              const std::string& outputPath) {
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
    appendEntry("timeSFMBAL/" + row.dataset + "/MultifrontalSolver",
                row.regularSolver);
    appendEntry("timeSFMBAL/" + row.dataset + "/BatchFactor/MultifrontalSolver",
                row.batchSolver);
    appendEntry(
        "timeSFMBAL/" + row.dataset + "/BatchFactorHessian/MultifrontalSolver",
        row.batchHessianSolver);
  }
  out << "\n]\n";
}

RunOptions parseBalFiles(int argc, char* argv[]) {
  std::string filename;
  bool profile = false;
  bool benchmarkActionJson = false;
  std::string benchmarkActionJsonPath;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--colamd") == 0) {
      gUseSchur = false;
      continue;
    }
    if (strcmp(argv[i], "--profile") == 0) {
      profile = true;
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
  if (profile && benchmarkActionJson) {
    throw runtime_error(usage());
  }

  if (!filename.empty()) {
    return {profile, benchmarkActionJson, benchmarkActionJsonPath, {filename}};
  }

  if (profile) {
    return {profile,
            benchmarkActionJson,
            benchmarkActionJsonPath,
            {findExampleDataFile(kProfileDataset)}};
  }

  if (benchmarkActionJson) {
    return {profile,
            benchmarkActionJson,
            benchmarkActionJsonPath,
            {findExampleDataFile(kDefaultBenchmarkDataset)}};
  }

  return {profile,
          benchmarkActionJson,
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
    if (gUseSchur) {
      ordering = createSchurOrdering(db, false);
    }

    const double newTime = runSolver(
        graph, initial, ordering, NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
        "MultifrontalSolver");
    double legacyTime = 0.0;
    double batchTime = 0.0;
    double batchHessianTime = 0.0;
    if (!options.profile) {
      legacyTime = runSolver(graph, initial, ordering,
                             NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                             "MultifrontalCholesky");
      const NonlinearFactorGraph batchGraph = buildBatchSfmGraph(db, false);
      batchTime = runSolver(batchGraph, initial, ordering,
                            NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                            "BatchFactor/MultifrontalSolver");
      const NonlinearFactorGraph batchHessianGraph =
          buildBatchSfmGraph(db, true);
      batchHessianTime =
          runSolver(batchHessianGraph, initial, ordering,
                    NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                    "BatchFactorHessian/MultifrontalSolver");
    }

    if (!options.profile) {
      rows.push_back(
          {dataset, legacyTime, newTime, batchTime, batchHessianTime});
    }
  }
  if (!options.profile) {
    std::cout << "\n| Dataset | Regular Cholesky s | Regular Solver s | "
                 "Batch Solver s | Batch Hessian s | Batch Speedup |\n";
    std::cout << "| --- | --- | --- | --- | --- | --- |\n";
    std::cout << std::fixed << std::setprecision(3);
    for (const auto& row : rows) {
      const double batchSpeedup =
          row.batchSolver > 0.0 ? (row.regularSolver / row.batchSolver) : 0.0;
      std::cout << "| " << row.dataset << " | " << row.regularCholesky << " | "
                << row.regularSolver << " | " << row.batchSolver << " | "
                << row.batchHessianSolver << " | " << batchSpeedup << "x |\n";
    }
  }

  if (options.benchmarkActionJson) {
    if (rows.empty()) {
      throw runtime_error("No benchmark rows found to write.");
    }
    writeBenchmarkActionJson(rows, options.benchmarkActionJsonPath);
    std::cout << "\nWrote benchmark-action JSON to "
              << options.benchmarkActionJsonPath << "\n";
  }
  return 0;
}
