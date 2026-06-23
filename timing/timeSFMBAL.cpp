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
         "[--cuda-lm] [--cuda-lm-graph] "
         "[--cuda-linear-solver dense-schur|cudss-full-normal] "
         "[--cuda-warmup-file FILE] "
         "[--projection-noise unit|huber|tukey] "
         "[--benchmark-action-json FILE] [BALfile ...]";
}

struct TimingRow {
  std::string dataset;
  double legacy = 0.0;
  double newer = 0.0;
};

enum class CudaLinearSolverOption {
  DenseSchur,
  CudssFullNormal,
};

struct RunOptions {
  bool profile = false;
  bool cudaStructureOnly = false;
  bool cudaLm = false;
  bool cudaLmGraph = false;
  bool cudaLinearSolverSpecified = false;
  CudaLinearSolverOption cudaLinearSolver = CudaLinearSolverOption::DenseSchur;
  bool cudaWarmupFileSpecified = false;
  std::string cudaWarmupFile;
  bool benchmarkActionJson = false;
  std::string benchmarkActionJsonPath;
  std::vector<std::string> filenames;
};

const char* cudaLinearSolverName(CudaLinearSolverOption solver) {
  switch (solver) {
    case CudaLinearSolverOption::DenseSchur:
      return "dense-schur";
    case CudaLinearSolverOption::CudssFullNormal:
      return "cudss-full-normal";
  }
  return "unknown";
}

void setProjectionNoiseModel(const char* noiseModel) {
  if (strcmp(noiseModel, "unit") == 0) {
    gNoiseModel = noiseModel::Unit::Create(2);
  } else if (strcmp(noiseModel, "huber") == 0) {
    gNoiseModel = noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345),
        noiseModel::Unit::Create(2));
  } else if (strcmp(noiseModel, "tukey") == 0) {
    gNoiseModel = noiseModel::Robust::Create(
        noiseModel::mEstimator::Tukey::Create(4.6851),
        noiseModel::Unit::Create(2));
  } else {
    throw runtime_error(usage());
  }
}

template <typename Params>
void applyBalBenchmarkLmSettings(Params& params) {
  params.maxIterations = 20;
  params.relativeErrorTol = 0.01;
}

LevenbergMarquardtParams makeBalLevenbergMarquardtParams() {
  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  applyBalBenchmarkLmSettings(params);
  return params;
}

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
enum class CudaLmDefaults {
  Backend,
  Graph,
};

gtsam::cuda::CudaSfmLinearSolverType cudaLinearSolverType(
    CudaLinearSolverOption solver) {
  switch (solver) {
    case CudaLinearSolverOption::DenseSchur:
      return gtsam::cuda::CudaSfmLinearSolverType::DenseSchur;
    case CudaLinearSolverOption::CudssFullNormal:
      return gtsam::cuda::CudaSfmLinearSolverType::CudssFullNormal;
  }
  return gtsam::cuda::CudaSfmLinearSolverType::DenseSchur;
}

gtsam::cuda::CudaSfmLevenbergMarquardtParams makeBalCudaLmParams(
    CudaLinearSolverOption solverOption, CudaLmDefaults defaults) {
  gtsam::cuda::CudaSfmLevenbergMarquardtParams params =
      defaults == CudaLmDefaults::Graph
          ? gtsam::cuda::CudaSfmLevenbergMarquardtParams::CeresDefaults()
          : gtsam::cuda::CudaSfmLevenbergMarquardtParams();
  applyBalBenchmarkLmSettings(params);
  params.linearSolver = cudaLinearSolverType(solverOption);
  return params;
}

struct CudaBackendLmRun {
  double elapsed = 0.0;
  gtsam::cuda::CudaSfmLevenbergMarquardtResult result;
};

CudaBackendLmRun runCudaBackendLm(
    const SfmData& db,
    const gtsam::cuda::CudaSfmLevenbergMarquardtParams& params) {
  const auto start = std::chrono::high_resolution_clock::now();
  const gtsam::cuda::CudaSfmLevenbergMarquardtResult result =
      gtsam::cuda::OptimizeCudaSfmWithoutValueDownload(db, params);
  const auto end = std::chrono::high_resolution_clock::now();

  CudaBackendLmRun run;
  run.elapsed = std::chrono::duration<double>(end - start).count();
  run.result = result;
  return run;
}

void printCudaBackendLmRun(const CudaBackendLmRun& run,
                           CudaLinearSolverOption solverOption) {
  const auto& result = run.result;
  std::cout << "  CUDA LM: " << run.elapsed << " s\n";
  std::cout << "  CUDA LM linear solver: "
            << cudaLinearSolverName(solverOption) << "\n";
  std::cout << "  CUDA LM solve loop: " << result.solveLoopElapsed << " s\n";
  std::cout << "  CUDA LM measured total: " << result.totalMeasuredElapsed
            << " s\n";
  std::cout << "  CUDA LM setup before solve loop: " << result.setupElapsed
            << " s\n";
  std::cout << "  CUDA LM setup breakdown:\n";
  std::cout << "    context: " << result.contextElapsed << " s\n";
  std::cout << "    pack values: " << result.packValuesElapsed << " s\n";
  std::cout << "    allocate trial values: " << result.allocateTrialElapsed
            << " s\n";
  std::cout << "    projection batch: " << result.projectionBatchElapsed
            << " s\n";
  std::cout << "    initial error: " << result.initialErrorElapsed << " s\n";
  std::cout << "    cuDSS solver construction: "
            << result.cudssSolverConstructionElapsed << " s\n";
  std::cout << "    dense Schur solver construction: "
            << result.denseSchurSolverConstructionElapsed << " s\n";
  std::cout << "    CSR structure: " << result.csrStructureElapsed << " s\n";
  std::cout << "    upload pattern: " << result.uploadPatternElapsed << " s\n";
  std::cout << "    first cuDSS analyze: "
            << result.firstCudssAnalyzeElapsed << " s\n";
  std::cout << "    download values: " << result.downloadElapsed << " s\n";
  std::cout << "Initial error: " << std::setprecision(15)
            << result.initialError << "\n";
  std::cout << "Final error: " << result.finalError
            << ", iterations: " << result.iterations
            << ", accepted: " << result.acceptedSteps << std::setprecision(6)
            << "\n";
}

struct CudaGraphLmRun {
  double elapsed = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t iterations = 0;
  gtsam::cuda::CudaSfmLevenbergMarquardtResult backend;
};

CudaGraphLmRun runCudaGraphLm(const NonlinearFactorGraph& graph,
                              const Values& initial,
                              const gtsam::cuda::CudaSfmLevenbergMarquardtParams&
                                  params) {
  const auto start = std::chrono::high_resolution_clock::now();
  gtsam::cuda::CudaSfmLevenbergMarquardtOptimizer lm(graph, initial, params);
  lm.optimize();
  const auto end = std::chrono::high_resolution_clock::now();

  CudaGraphLmRun run;
  run.elapsed = std::chrono::duration<double>(end - start).count();
  run.initialError = lm.result().initialError;
  run.finalError = lm.error();
  run.iterations = lm.iterations();
  run.backend = lm.result();
  return run;
}

void printCudaGraphLmRun(const CudaGraphLmRun& run,
                         CudaLinearSolverOption solverOption) {
  std::cout << "  CUDA LM graph API: " << run.elapsed << " s\n";
  std::cout << "  CUDA LM graph linear solver: "
            << cudaLinearSolverName(solverOption) << "\n";
  std::cout << "  CUDA LM graph API overhead over backend: "
            << run.elapsed - run.backend.totalMeasuredElapsed << " s\n";
  std::cout << "  CUDA LM backend solve loop: " << run.backend.solveLoopElapsed
            << " s\n";
  std::cout << "  CUDA LM backend measured total: "
            << run.backend.totalMeasuredElapsed << " s\n";
  std::cout << "  CUDA LM backend setup before solve loop: "
            << run.backend.setupElapsed << " s\n";
  std::cout << "  CUDA LM backend setup breakdown:\n";
  std::cout << "    context: " << run.backend.contextElapsed << " s\n";
  std::cout << "    pack values: " << run.backend.packValuesElapsed << " s\n";
  std::cout << "    allocate trial values: "
            << run.backend.allocateTrialElapsed << " s\n";
  std::cout << "    projection batch: " << run.backend.projectionBatchElapsed
            << " s\n";
  std::cout << "    initial error: " << run.backend.initialErrorElapsed
            << " s\n";
  std::cout << "    cuDSS solver construction: "
            << run.backend.cudssSolverConstructionElapsed << " s\n";
  std::cout << "    dense Schur solver construction: "
            << run.backend.denseSchurSolverConstructionElapsed << " s\n";
  std::cout << "    CSR structure: " << run.backend.csrStructureElapsed
            << " s\n";
  std::cout << "    upload pattern: " << run.backend.uploadPatternElapsed
            << " s\n";
  std::cout << "    first cuDSS analyze: "
            << run.backend.firstCudssAnalyzeElapsed << " s\n";
  std::cout << "    download values: " << run.backend.downloadElapsed
            << " s\n";
  std::cout << "Initial error: " << std::setprecision(15)
            << run.initialError << "\n";
  std::cout << "Final error: " << run.finalError
            << ", iterations: " << run.iterations
            << ", accepted: " << run.backend.acceptedSteps
            << ", inner: " << run.backend.innerIterations
            << std::setprecision(6) << "\n";
}
#endif

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
  std::vector<std::string> filenames;
  bool profile = false;
  bool cudaStructureOnly = false;
  bool cudaLm = false;
  bool cudaLmGraph = false;
  bool cudaLinearSolverSpecified = false;
  CudaLinearSolverOption cudaLinearSolver = CudaLinearSolverOption::DenseSchur;
  bool cudaWarmupFileSpecified = false;
  std::string cudaWarmupFile;
  bool benchmarkActionJson = false;
  std::string benchmarkActionJsonPath;
  bool projectionNoiseSpecified = false;
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
    if (strcmp(argv[i], "--cuda-lm-graph") == 0) {
      cudaLmGraph = true;
      continue;
    }
    if (strcmp(argv[i], "--cuda-linear-solver") == 0) {
      if (++i >= argc || argv[i][0] == '-') {
        throw runtime_error(usage());
      }
      cudaLinearSolverSpecified = true;
      if (strcmp(argv[i], "dense-schur") == 0) {
        cudaLinearSolver = CudaLinearSolverOption::DenseSchur;
      } else if (strcmp(argv[i], "cudss-full-normal") == 0) {
        cudaLinearSolver = CudaLinearSolverOption::CudssFullNormal;
      } else {
        throw runtime_error(usage());
      }
      continue;
    }
    if (strcmp(argv[i], "--cuda-warmup-file") == 0) {
      if (++i >= argc || argv[i][0] == '-') {
        throw runtime_error(usage());
      }
      cudaWarmupFileSpecified = true;
      cudaWarmupFile = argv[i];
      continue;
    }
    if (strcmp(argv[i], "--projection-noise") == 0) {
      if (++i >= argc || argv[i][0] == '-') {
        throw runtime_error(usage());
      }
      projectionNoiseSpecified = true;
      setProjectionNoiseModel(argv[i]);
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
    filenames.emplace_back(argv[i]);
  }

  if (profile && !filenames.empty()) {
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
  if (cudaLmGraph && benchmarkActionJson) {
    throw runtime_error(usage());
  }
  if (cudaLm && cudaStructureOnly) {
    throw runtime_error(usage());
  }
  if (cudaLmGraph && cudaStructureOnly) {
    throw runtime_error(usage());
  }
  if (cudaLm && cudaLmGraph) {
    throw runtime_error(usage());
  }
  if (projectionNoiseSpecified && cudaLm) {
    throw runtime_error(
        "--projection-noise only applies to factor-graph runs; use "
        "--cuda-lm-graph for CUDA robust-noise benchmarking");
  }
  if (cudaLinearSolverSpecified && !cudaLm && !cudaLmGraph) {
    throw runtime_error(usage());
  }
  if (cudaWarmupFileSpecified && !cudaLm && !cudaLmGraph) {
    throw runtime_error(usage());
  }

  if (!filenames.empty()) {
    return {profile,
            cudaStructureOnly,
            cudaLm,
            cudaLmGraph,
            cudaLinearSolverSpecified,
            cudaLinearSolver,
            cudaWarmupFileSpecified,
            cudaWarmupFile,
            benchmarkActionJson,
            benchmarkActionJsonPath,
            filenames};
  }

  if (profile) {
    return {profile,
            cudaStructureOnly,
            cudaLm,
            cudaLmGraph,
            cudaLinearSolverSpecified,
            cudaLinearSolver,
            cudaWarmupFileSpecified,
            cudaWarmupFile,
            benchmarkActionJson,
            benchmarkActionJsonPath,
            {findExampleDataFile(kProfileDataset)}};
  }

  if (benchmarkActionJson) {
    return {profile,
            cudaStructureOnly,
            cudaLm,
            cudaLmGraph,
            cudaLinearSolverSpecified,
            cudaLinearSolver,
            cudaWarmupFileSpecified,
            cudaWarmupFile,
            benchmarkActionJson,
            benchmarkActionJsonPath,
            {findExampleDataFile(kDefaultBenchmarkDataset)}};
  }

  return {profile,
          cudaStructureOnly,
          cudaLm,
          cudaLmGraph,
          cudaLinearSolverSpecified,
          cudaLinearSolver,
          cudaWarmupFileSpecified,
          cudaWarmupFile,
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
  LevenbergMarquardtParams params = makeBalLevenbergMarquardtParams();
  params.setVerbosityLM("SUMMARY");
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
  bool cudaWarmupDone = false;

  for (const auto& filename : options.filenames) {
    const std::string dataset =
        std::filesystem::path(filename).filename().string();
    std::cout << "\nProcessing BAL file: " << filename << std::endl;
    const SfmData db = SfmData::FromBalFile(filename);

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
    if (options.cudaLm) {
      const auto cudaParams =
          makeBalCudaLmParams(options.cudaLinearSolver, CudaLmDefaults::Backend);

      if (options.cudaWarmupFileSpecified && !cudaWarmupDone) {
        std::cout << "  CUDA warmup file: " << options.cudaWarmupFile
                  << " (timing ignored)\n";
        const SfmData warmupDb = SfmData::FromBalFile(options.cudaWarmupFile);
        const CudaBackendLmRun warmupRun =
            runCudaBackendLm(warmupDb, cudaParams);
        std::cout << "  CUDA warmup: " << warmupRun.elapsed
                  << " s ignored, final error: " << std::setprecision(15)
                  << warmupRun.result.finalError
                  << ", iterations: " << warmupRun.result.iterations
                  << std::setprecision(6) << "\n";
        cudaWarmupDone = true;
      }

      const CudaBackendLmRun run = runCudaBackendLm(db, cudaParams);
      printCudaBackendLmRun(run, options.cudaLinearSolver);
      continue;
    }
#elif GTSAM_ENABLE_CUDA
    if (options.cudaLm || options.cudaLmGraph) {
      throw std::runtime_error(
          "--cuda-lm and --cuda-lm-graph require configuring with "
          "GTSAM_ENABLE_CUDSS=ON");
    }
#else
    if (options.cudaLm || options.cudaLmGraph) {
      throw std::runtime_error(
          "--cuda-lm and --cuda-lm-graph require configuring with "
          "GTSAM_ENABLE_CUDA=ON and GTSAM_ENABLE_CUDSS=ON");
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

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
    if (options.cudaLmGraph) {
      const auto cudaParams =
          makeBalCudaLmParams(options.cudaLinearSolver, CudaLmDefaults::Graph);

      if (options.cudaWarmupFileSpecified && !cudaWarmupDone) {
        std::cout << "  CUDA graph warmup file: " << options.cudaWarmupFile
                  << " (timing ignored)\n";
        const SfmData warmupDb = SfmData::FromBalFile(options.cudaWarmupFile);
        const NonlinearFactorGraph warmupGraph =
            buildGeneralSfmGraph(warmupDb);
        const Values warmupInitial = buildGeneralSfmInitial(warmupDb);
        const CudaGraphLmRun warmupRun =
            runCudaGraphLm(warmupGraph, warmupInitial, cudaParams);
        std::cout << "  CUDA graph warmup: " << warmupRun.elapsed
                  << " s ignored, final error: " << std::setprecision(15)
                  << warmupRun.finalError
                  << ", iterations: " << warmupRun.iterations
                  << std::setprecision(6) << "\n";
        cudaWarmupDone = true;
      }

      const CudaGraphLmRun run = runCudaGraphLm(graph, initial, cudaParams);
      printCudaGraphLmRun(run, options.cudaLinearSolver);
      continue;
    }
#endif

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
  if (!options.profile && !options.cudaStructureOnly && !options.cudaLm &&
      !options.cudaLmGraph) {
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
      !options.cudaLm && !options.cudaLmGraph) {
    if (rows.empty()) {
      throw runtime_error("No benchmark rows found to write.");
    }
    writeBenchmarkActionJson(rows, options.benchmarkActionJsonPath);
    std::cout << "\nWrote benchmark-action JSON to "
              << options.benchmarkActionJsonPath << "\n";
  }
  return 0;
}
