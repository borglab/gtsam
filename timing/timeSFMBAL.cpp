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

#if GTSAM_ENABLE_CUDA
#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>
#endif

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace {
constexpr const char* kDefaultBenchmarkDataset = "dubrovnik-16-22106-pre";
constexpr const char* kProfileDataset = "dubrovnik-135-90642-pre";

std::string usage() {
  return "Usage: timeSFMBAL [--colamd] [--profile] [--cuda-structure-only] "
         "[--cuda-lm] [--benchmark-action-json FILE] [BALfile]";
}

struct TimingRow {
  std::string dataset;
  double legacy = 0.0;
  double newer = 0.0;
};

struct RunOptions {
  bool profile = false;
  bool cudaStructureOnly = false;
  bool cudaLm = false;
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
                row.legacy);
    appendEntry("timeSFMBAL/" + row.dataset + "/MultifrontalSolver", row.newer);
  }
  out << "\n]\n";
}

RunOptions parseBalFiles(int argc, char* argv[]) {
  std::string filename;
  bool profile = false;
  bool cudaStructureOnly = false;
  bool cudaLm = false;
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
    if (strcmp(argv[i], "--cuda-structure-only") == 0) {
      cudaStructureOnly = true;
      continue;
    }
    if (strcmp(argv[i], "--cuda-lm") == 0) {
      cudaLm = true;
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
  if (cudaStructureOnly && benchmarkActionJson) {
    throw runtime_error(usage());
  }
  if (cudaLm && benchmarkActionJson) {
    throw runtime_error(usage());
  }
  if (cudaLm && cudaStructureOnly) {
    throw runtime_error(usage());
  }

  if (!filename.empty()) {
    return {profile, cudaStructureOnly, cudaLm, benchmarkActionJson,
            benchmarkActionJsonPath, {filename}};
  }

  if (profile) {
    return {profile, cudaStructureOnly, cudaLm, benchmarkActionJson,
            benchmarkActionJsonPath,
            {findExampleDataFile(kProfileDataset)}};
  }

  if (benchmarkActionJson) {
    return {profile, cudaStructureOnly, cudaLm, benchmarkActionJson,
            benchmarkActionJsonPath,
            {findExampleDataFile(kDefaultBenchmarkDataset)}};
  }

  return {profile, cudaStructureOnly, cudaLm, benchmarkActionJson,
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
  // The SUMMARY table above prints costs with 2 significant digits only;
  // emit a precise line for external harnesses to parse.
  std::cout << "Final error: " << std::setprecision(15) << lm.error()
            << ", iterations: " << lm.iterations() << std::setprecision(6)
            << "\n";
  return elapsed.count();
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

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
    if (options.cudaLm) {
      gtsam::cuda::CudaSfmLevenbergMarquardtParams params;
      params.maxIterations = 20;
      params.relativeErrorTol = 0.01;
      params.downloadOptimizedValues = false;

      const auto start = std::chrono::high_resolution_clock::now();
      const gtsam::cuda::CudaSfmLevenbergMarquardtResult result =
          gtsam::cuda::OptimizeCudaSfm(db, params);
      const auto end = std::chrono::high_resolution_clock::now();
      const std::chrono::duration<double> elapsed = end - start;

      std::cout << "  CUDA LM: " << elapsed.count() << " s\n";
      std::cout << "  CUDA LM solve loop: " << result.solveLoopElapsed
                << " s\n";
      std::cout << "Initial error: " << std::setprecision(15)
                << result.initialError << "\n";
      std::cout << "Final error: " << result.finalError
                << ", iterations: " << result.iterations
                << ", accepted: " << result.acceptedSteps
                << std::setprecision(6) << "\n";
      continue;
    }
#elif GTSAM_ENABLE_CUDA
    if (options.cudaLm) {
      throw std::runtime_error(
          "--cuda-lm requires configuring with GTSAM_ENABLE_CUDSS=ON");
    }
#else
    if (options.cudaLm) {
      throw std::runtime_error(
          "--cuda-lm requires configuring with GTSAM_ENABLE_CUDA=ON and "
          "GTSAM_ENABLE_CUDSS=ON");
    }
#endif

#if GTSAM_ENABLE_CUDA
    if (options.cudaStructureOnly) {
      gtsam::cuda::CudaContext context;
      const auto values = gtsam::cuda::PackSfmValues(db, context.stream());
      const auto batch =
          gtsam::cuda::CudaSfmProjectionBatch::FromSfmData(db, context.stream());
      const auto csr = gtsam::cuda::CudaBalCsrStructure::FromSfmData(db);
      context.synchronize();

      std::cout << "CUDA BAL structure: cameras=" << batch.numCameras()
                << " points=" << batch.numPoints()
                << " observations=" << batch.numObservations()
                << " packed_values=" << values.index().size()
                << " dimension=" << csr.dimension()
                << " csr_nnz=" << csr.colIndices().size() << std::endl;
      continue;
    }
#else
    if (options.cudaStructureOnly) {
      throw std::runtime_error(
          "--cuda-structure-only requires configuring with GTSAM_ENABLE_CUDA=ON");
    }
#endif

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
    if (!options.profile) {
      legacyTime = runSolver(graph, initial, ordering,
                             NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                             "MultifrontalCholesky");
    }

    if (!options.profile) {
      rows.push_back({dataset, legacyTime, newTime});
    }
  }
  if (!options.profile && !options.cudaStructureOnly && !options.cudaLm) {
    std::cout
        << "\n| Dataset | Legacy (Cholesky) s | New (Solver) s | Speedup |\n";
    std::cout << "| --- | --- | --- | --- |\n";
    std::cout << std::fixed << std::setprecision(3);
    for (const auto& row : rows) {
      const double speedup = row.newer > 0.0 ? (row.legacy / row.newer) : 0.0;
      std::cout << "| " << row.dataset << " | " << row.legacy << " | "
                << row.newer << " | " << speedup << "x |\n";
    }
  }

  if (options.benchmarkActionJson && !options.cudaStructureOnly &&
      !options.cudaLm) {
    if (rows.empty()) {
      throw runtime_error("No benchmark rows found to write.");
    }
    writeBenchmarkActionJson(rows, options.benchmarkActionJsonPath);
    std::cout << "\nWrote benchmark-action JSON to "
              << options.benchmarkActionJsonPath << "\n";
  }
  return 0;
}
