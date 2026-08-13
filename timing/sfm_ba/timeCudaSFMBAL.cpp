/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeCudaSFMBAL.cpp
 * @brief   time CUDA SFM/BA with BAL files
 * @author  Frank Dellaert
 * @date    June 6, 2015
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/BatchFactor.h>

#include "../internal/TimingUtils.h"
#include "../timeSFMBAL.h"
#include "GncOutlierSampling.h"

#if GTSAM_ENABLE_CUDA
#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>
#endif

#include <gtsam/nonlinear/GncOptimizer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <random>

namespace {
using namespace gtsam;
using namespace std;
using symbol_shorthand::C;
using symbol_shorthand::P;
namespace bal = gtsam::timing::bal;

std::string usage() {
  return "Usage: timeCudaSFMBAL [--colamd] [--profile] [--cuda-structure-only] "
         "[--cuda-lm] [--cuda-lm-graph] "
         "[--cuda-linear-solver dense-schur|cudss-full-normal] "
         "[--cuda-lm-graph-kind raw|point-batch|camera-batch] "
         "[--batch-chunk-size N] "
         "[--cuda-warmup-file FILE] "
         "[--projection-noise unit|huber|tukey] "
         "[--gnc cpu|cuda] [--gnc-loss tls|gm] "
         "[--gnc-outlier-fraction F] [--gnc-outlier-pixels P] "
         "[--gnc-seed N] [--gnc-max-outer N] "
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

enum class CudaGraphKind {
  Raw,
  PointBatch,
  CameraBatch,
};

enum class GncBackend {
  None,
  Cpu,
  Cuda,
};

struct GncRunOptions {
  GncBackend backend = GncBackend::None;
  GncLossType lossType = GncLossType::TLS;
  double outlierFraction = 0.1;
  double outlierPixels = 12.0;
  unsigned int seed = 42;
  size_t maxOuterIterations = 100;
};

struct RunOptions {
  bal::BalBenchmarkConfig config;
  bool profile = false;
  bool cudaStructureOnly = false;
  bool cudaLm = false;
  bool cudaLmGraph = false;
  bool cudaLinearSolverSpecified = false;
  CudaLinearSolverOption cudaLinearSolver = CudaLinearSolverOption::DenseSchur;
  bool cudaGraphKindSpecified = false;
  CudaGraphKind cudaGraphKind = CudaGraphKind::Raw;
  bool batchChunkSizeSpecified = false;
  size_t batchChunkSize = 0;
  bool cudaWarmupFileSpecified = false;
  std::string cudaWarmupFile;
  bool benchmarkActionJson = false;
  std::string benchmarkActionJsonPath;
  GncRunOptions gnc;
  std::vector<std::string> filenames;
  bool help = false;
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

const char* cudaGraphKindName(CudaGraphKind kind) {
  switch (kind) {
    case CudaGraphKind::Raw:
      return "raw";
    case CudaGraphKind::PointBatch:
      return "point-batch";
    case CudaGraphKind::CameraBatch:
      return "camera-batch";
  }
  return "unknown";
}

void setProjectionNoiseModel(bal::BalBenchmarkConfig* config,
                             const std::string& name) {
  if (name == "unit") {
    config->projectionNoise = noiseModel::Unit::Create(2);
  } else if (name == "huber") {
    config->projectionNoise =
        noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345),
                                   noiseModel::Unit::Create(2));
  } else if (name == "tukey") {
    config->projectionNoise = noiseModel::Robust::Create(
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

LevenbergMarquardtParams makeBalLevenbergMarquardtParams(
    const bal::BalBenchmarkConfig& config) {
  LevenbergMarquardtParams params =
      bal::makeLevenbergMarquardtParams(config, nullptr, "SILENT");
  applyBalBenchmarkLmSettings(params);
  return params;
}

void printProfileRow(const std::string& indent, const std::string& label,
                     double elapsed, double total) {
  std::cout << indent << std::left << std::setw(30) << label << std::right
            << elapsed << " s";
  if (total > 0.0) {
    std::cout << " (" << (100.0 * elapsed / total) << "%)";
  }
  std::cout << "\n";
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
    CudaLinearSolverOption solverOption, CudaLmDefaults defaults,
    bool enableDetailedProfiling) {
  gtsam::cuda::CudaSfmLevenbergMarquardtParams params =
      defaults == CudaLmDefaults::Graph
          ? gtsam::cuda::CudaSfmLevenbergMarquardtParams::CeresDefaults()
          : gtsam::cuda::CudaSfmLevenbergMarquardtParams();
  applyBalBenchmarkLmSettings(params);
  params.linearSolver = cudaLinearSolverType(solverOption);
  params.enableDetailedProfiling = enableDetailedProfiling;
  return params;
}

struct CudaBackendLmRun {
  double elapsed = 0.0;
  gtsam::cuda::CudaSfmLevenbergMarquardtResult result;
};

CudaBackendLmRun runCudaBackendLm(
    const SfmData& db,
    const gtsam::cuda::CudaSfmLevenbergMarquardtParams& params) {
  CudaBackendLmRun run;
  run.elapsed = gtsam::timing::measureSeconds([&] {
    run.result = gtsam::cuda::OptimizeCudaSfmWithoutValueDownload(db, params);
  });
  return run;
}

const char* yesNo(bool value) { return value ? "yes" : "no"; }

void printTransferRow(const std::string& indent, const std::string& label,
                      double elapsed, size_t bytes, double total) {
  std::cout << indent << std::left << std::setw(30) << label << std::right
            << elapsed << " s";
  if (total > 0.0) {
    std::cout << " (" << (100.0 * elapsed / total) << "%)";
  }
  std::cout << " [" << bytes << " B";
  if (elapsed > 0.0 && bytes > 0) {
    const double gib = static_cast<double>(bytes) / (1024.0 * 1024.0 * 1024.0);
    std::cout << ", " << (gib / elapsed) << " GiB/s";
  }
  std::cout << "]\n";
}

void printCudaLmSetupBreakdown(
    const gtsam::cuda::CudaSfmLevenbergMarquardtResult& result,
    const std::string& prefix) {
  std::cout << prefix << "stage breakdown:\n";
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
  std::cout << "    first cuDSS analyze (solve loop): "
            << result.firstCudssAnalyzeElapsed << " s\n";
  std::cout << "    download values (post solve): " << result.downloadElapsed
            << " s\n";
}

void printCudaLmTransferBreakdown(
    const gtsam::cuda::CudaSfmLevenbergMarquardtResult& result,
    const std::string& prefix) {
  const double total = result.totalMeasuredElapsed;
  std::cout << prefix << "pure transfer breakdown:\n";
  printProfileRow("    ", "pack values host build",
                  result.packValuesHostBuildElapsed, total);
  printProfileRow("    ", "pack values device alloc",
                  result.packValuesDeviceAllocElapsed, total);
  printTransferRow("    ", "pack values H2D memcpy",
                   result.packValuesH2dCopyElapsed, result.packValuesH2dBytes,
                   total);
  printProfileRow("    ", "projection host build",
                  result.projectionBatchHostBuildElapsed, total);
  printProfileRow("    ", "projection device alloc",
                  result.projectionBatchDeviceAllocElapsed, total);
  printTransferRow("    ", "projection H2D memcpy",
                   result.projectionBatchH2dCopyElapsed,
                   result.projectionBatchH2dBytes, total);
  printProfileRow("    ", "CSR pattern device alloc",
                  result.uploadPatternDeviceAllocElapsed, total);
  printTransferRow("    ", "CSR pattern H2D memcpy",
                   result.uploadPatternH2dCopyElapsed,
                   result.uploadPatternH2dBytes, total);
  printTransferRow("    ", "total H2D memcpy", result.totalH2dCopyElapsed,
                   result.totalH2dBytes, total);
  printProfileRow("    ", "download host alloc",
                  result.downloadHostAllocElapsed, total);
  printTransferRow("    ", "download D2H memcpy", result.downloadD2hCopyElapsed,
                   result.downloadD2hBytes, total);
  printProfileRow("    ", "download Values rebuild",
                  result.downloadValuesBuildElapsed, total);
  printTransferRow("    ", "total D2H memcpy", result.totalD2hCopyElapsed,
                   result.totalD2hBytes, total);
}

void printCudaLmDetailedBreakdown(
    const gtsam::cuda::CudaSfmLevenbergMarquardtResult& result,
    const std::string& prefix) {
  const double solveLoop = result.solveLoopElapsed;
  const double accounted =
      result.dampingDiagonalElapsed + result.denseSchurSolveElapsed +
      result.normalEquationsElapsed + result.addDampingElapsed +
      result.cudssAnalyzeElapsed + result.cudssSolveElapsed +
      result.linearizedErrorElapsed + result.applyDeltaElapsed +
      result.trialErrorElapsed + result.acceptTrialElapsed +
      result.lambdaUpdateElapsed;

  std::cout << prefix << "solve-loop aggregate breakdown:\n";
  printProfileRow("    ", "damping diagonal", result.dampingDiagonalElapsed,
                  solveLoop);
  printProfileRow("    ", "dense Schur solve", result.denseSchurSolveElapsed,
                  solveLoop);
  printProfileRow("    ", "normal equations", result.normalEquationsElapsed,
                  solveLoop);
  printProfileRow("    ", "add diagonal damping", result.addDampingElapsed,
                  solveLoop);
  printProfileRow("    ", "cuDSS analyze", result.cudssAnalyzeElapsed,
                  solveLoop);
  printProfileRow("    ", "cuDSS solve", result.cudssSolveElapsed, solveLoop);
  printProfileRow("    ", "linearized error change",
                  result.linearizedErrorElapsed, solveLoop);
  printProfileRow("    ", "apply delta", result.applyDeltaElapsed, solveLoop);
  printProfileRow("    ", "trial projection error", result.trialErrorElapsed,
                  solveLoop);
  printProfileRow("    ", "accept trial copy", result.acceptTrialElapsed,
                  solveLoop);
  printProfileRow("    ", "lambda/convergence update",
                  result.lambdaUpdateElapsed, solveLoop);
  printProfileRow("    ", "profiled subtotal", accounted, solveLoop);
  printProfileRow("    ", "unprofiled loop time", solveLoop - accounted,
                  solveLoop);

  std::cout << prefix << "per-iteration breakdown:\n";
  if (result.iterationProfiles.empty()) {
    std::cout << "    no LM iterations recorded\n";
    return;
  }
  for (const auto& iteration : result.iterationProfiles) {
    std::cout << "    iteration " << iteration.iteration << ": total "
              << iteration.totalElapsed << " s, attempts "
              << iteration.attemptProfiles.size()
              << ", accepted: " << yesNo(iteration.acceptedStep)
              << ", terminated: " << yesNo(iteration.terminated) << "\n";
    std::cout << "      error: " << std::setprecision(15)
              << iteration.startError << " -> " << iteration.endError
              << std::setprecision(6) << "\n";
    std::cout << "      lambda: " << iteration.startLambda << " -> "
              << iteration.endLambda << "\n";
    printProfileRow("      ", "damping diagonal",
                    iteration.dampingDiagonalElapsed, iteration.totalElapsed);
    printProfileRow("      ", "accept trial copy", iteration.acceptTrialElapsed,
                    iteration.totalElapsed);

    for (const auto& attempt : iteration.attemptProfiles) {
      std::cout << "      attempt " << attempt.attempt << ": total "
                << attempt.totalElapsed << " s, lambda " << attempt.lambda
                << ", accepted: " << yesNo(attempt.accepted)
                << ", step successful: " << yesNo(attempt.stepSuccessful)
                << ", attempted trial: " << yesNo(attempt.attemptedTrial)
                << ", terminated: " << yesNo(attempt.terminated) << "\n";
      std::cout << "        costs: linearized change "
                << attempt.linearizedCostChange << ", actual change "
                << attempt.costChange << ", fidelity " << attempt.modelFidelity
                << ", trial error " << std::setprecision(15)
                << attempt.trialError << std::setprecision(6) << "\n";
      std::cout << "        flags: stop lambda search "
                << yesNo(attempt.stopSearchingLambda) << ", lambda upper bound "
                << yesNo(attempt.lambdaUpperBoundReached) << "\n";
      printProfileRow("        ", "dense Schur solve",
                      attempt.denseSchurSolveElapsed, attempt.totalElapsed);
      printProfileRow("        ", "normal equations",
                      attempt.normalEquationsElapsed, attempt.totalElapsed);
      printProfileRow("        ", "add diagonal damping",
                      attempt.addDampingElapsed, attempt.totalElapsed);
      printProfileRow("        ", "cuDSS analyze", attempt.cudssAnalyzeElapsed,
                      attempt.totalElapsed);
      printProfileRow("        ", "cuDSS solve", attempt.cudssSolveElapsed,
                      attempt.totalElapsed);
      printProfileRow("        ", "linearized error change",
                      attempt.linearizedErrorElapsed, attempt.totalElapsed);
      printProfileRow("        ", "apply delta", attempt.applyDeltaElapsed,
                      attempt.totalElapsed);
      printProfileRow("        ", "trial projection error",
                      attempt.trialErrorElapsed, attempt.totalElapsed);
      printProfileRow("        ", "lambda/convergence update",
                      attempt.lambdaUpdateElapsed, attempt.totalElapsed);
    }
  }
}

void printCudaBackendLmRun(const CudaBackendLmRun& run,
                           CudaLinearSolverOption solverOption,
                           bool detailedProfiling) {
  const auto& result = run.result;
  std::cout << "  CUDA LM: " << run.elapsed << " s\n";
  std::cout << "  CUDA LM linear solver: " << cudaLinearSolverName(solverOption)
            << "\n";
  std::cout << "  CUDA LM solve loop: " << result.solveLoopElapsed << " s\n";
  std::cout << "  CUDA LM measured total: " << result.totalMeasuredElapsed
            << " s\n";
  std::cout << "  CUDA LM setup before solve loop: " << result.setupElapsed
            << " s\n";
  printCudaLmSetupBreakdown(result, "  CUDA LM ");
  if (detailedProfiling) {
    printCudaLmTransferBreakdown(result, "  CUDA LM ");
    printCudaLmDetailedBreakdown(result, "  CUDA LM ");
  } else {
    std::cout << "  CUDA LM detailed profiling: disabled (use --profile to "
                 "enable)\n";
  }
  std::cout << "Initial error: " << std::setprecision(15) << result.initialError
            << "\n";
  std::cout << "Final error: " << result.finalError
            << ", iterations: " << result.iterations
            << ", accepted: " << result.acceptedSteps << std::setprecision(6)
            << "\n";
}

struct CudaGraphLmRun {
  double elapsed = 0.0;
  double optimizerConstructionElapsed = 0.0;
  double optimizeElapsed = 0.0;
  double resultQueryElapsed = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t iterations = 0;
  gtsam::cuda::CudaSfmLevenbergMarquardtResult backend;
};

CudaGraphLmRun runCudaGraphLm(
    const NonlinearFactorGraph& graph, const Values& initial,
    const gtsam::cuda::CudaSfmLevenbergMarquardtParams& params) {
  CudaGraphLmRun run;
  run.elapsed = gtsam::timing::measureSeconds([&] {
    std::optional<gtsam::cuda::CudaSfmLevenbergMarquardtOptimizer> optimizer;
    run.optimizerConstructionElapsed = gtsam::timing::measureSeconds(
        [&] { optimizer.emplace(graph, initial, params); });
    run.optimizeElapsed = gtsam::timing::measureSeconds([&] {
      const Values& optimized = optimizer->optimize();
      (void)optimized;
    });
    run.resultQueryElapsed = gtsam::timing::measureSeconds([&] {
      run.initialError = optimizer->result().initialError;
      run.finalError = optimizer->error();
      run.iterations = optimizer->iterations();
      run.backend = optimizer->result();
    });
  });
  return run;
}

void printCudaGraphLmRun(const CudaGraphLmRun& run,
                         CudaLinearSolverOption solverOption,
                         CudaGraphKind graphKind, bool detailedProfiling) {
  const double graphBackendCallOverhead =
      run.backend.graphBackendCallElapsed - run.backend.totalMeasuredElapsed;
  const double graphApiOtherOverhead =
      run.optimizeElapsed - run.backend.totalMeasuredElapsed -
      run.backend.graphConversionElapsed - run.backend.graphValueMergeElapsed;
  const double graphApiRemainingOptimizeOverhead =
      run.optimizeElapsed - run.backend.graphConversionElapsed -
      run.backend.graphBackendCallElapsed - run.backend.graphValueMergeElapsed -
      run.backend.graphConvertedDataDestructionElapsed;

  std::cout << "  CUDA LM graph API: " << run.elapsed << " s\n";
  std::cout << "  CUDA LM graph linear solver: "
            << cudaLinearSolverName(solverOption) << "\n";
  std::cout << "  CUDA LM graph kind: " << cudaGraphKindName(graphKind) << "\n";
  std::cout << "  CUDA LM graph API overhead over backend: "
            << run.elapsed - run.backend.totalMeasuredElapsed << " s\n";
  std::cout << "  CUDA LM graph API breakdown:\n";
  printProfileRow("    ", "optimizer construction",
                  run.optimizerConstructionElapsed, run.elapsed);
  printProfileRow("    ", "optimize call", run.optimizeElapsed, run.elapsed);
  printProfileRow("    ", "result/error queries", run.resultQueryElapsed,
                  run.elapsed);
  printProfileRow("    ", "graph conversion",
                  run.backend.graphConversionElapsed, run.elapsed);
  printProfileRow("    ", "backend measured total",
                  run.backend.totalMeasuredElapsed, run.elapsed);
  printProfileRow("    ", "backend return/assignment", graphBackendCallOverhead,
                  run.elapsed);
  printProfileRow("    ", "value merge/state update",
                  run.backend.graphValueMergeElapsed, run.elapsed);
  printProfileRow("    ", "converted data destruction",
                  run.backend.graphConvertedDataDestructionElapsed,
                  run.elapsed);
  printProfileRow("    ", "other graph optimize overhead",
                  graphApiOtherOverhead, run.elapsed);
  printProfileRow("    ", "remaining optimize overhead",
                  graphApiRemainingOptimizeOverhead, run.elapsed);
  std::cout << "  CUDA LM backend solve loop: " << run.backend.solveLoopElapsed
            << " s\n";
  std::cout << "  CUDA LM backend measured total: "
            << run.backend.totalMeasuredElapsed << " s\n";
  std::cout << "  CUDA LM backend setup before solve loop: "
            << run.backend.setupElapsed << " s\n";
  printCudaLmSetupBreakdown(run.backend, "  CUDA LM backend ");
  if (detailedProfiling) {
    printCudaLmTransferBreakdown(run.backend, "  CUDA LM backend ");
    printCudaLmDetailedBreakdown(run.backend, "  CUDA LM backend ");
  } else {
    std::cout << "  CUDA LM backend detailed profiling: disabled (use "
                 "--profile to enable)\n";
  }
  std::cout << "Initial error: " << std::setprecision(15) << run.initialError
            << "\n";
  std::cout << "Final error: " << run.finalError
            << ", iterations: " << run.iterations
            << ", accepted: " << run.backend.acceptedSteps
            << ", inner: " << run.backend.innerIterations
            << std::setprecision(6) << "\n";
}
#endif

void writeBenchmarkActionJson(const std::vector<TimingRow>& rows,
                              const std::string& outputPath) {
  std::vector<gtsam::timing::BenchmarkMetric> metrics;
  for (const auto& row : rows) {
    const std::string prefix = "timeSFMBAL/" + row.dataset + "/";
    metrics.push_back({prefix + "MultifrontalCholesky", "s", row.legacy});
    metrics.push_back({prefix + "MultifrontalSolver", "s", row.newer});
  }
  gtsam::timing::writeBenchmarkActionMetrics(outputPath, metrics);
}
RunOptions parseBalFiles(int argc, char* argv[]) {
  gtsam::timing::Arguments arguments(argc, argv);
  RunOptions options;
  options.config.useSchur = !arguments.flag("--colamd");
  options.profile = arguments.flag("--profile");
  options.cudaStructureOnly = arguments.flag("--cuda-structure-only");
  options.cudaLm = arguments.flag("--cuda-lm");
  options.cudaLmGraph = arguments.flag("--cuda-lm-graph");
  options.help = arguments.helpRequested();

  const auto linearSolver = arguments.optionalString("--cuda-linear-solver");
  options.cudaLinearSolverSpecified = linearSolver.has_value();
  if (linearSolver == "dense-schur") {
    options.cudaLinearSolver = CudaLinearSolverOption::DenseSchur;
  } else if (linearSolver == "cudss-full-normal") {
    options.cudaLinearSolver = CudaLinearSolverOption::CudssFullNormal;
  } else if (linearSolver) {
    throw std::runtime_error(usage());
  }

  const auto graphKind = arguments.optionalString("--cuda-lm-graph-kind");
  options.cudaGraphKindSpecified = graphKind.has_value();
  if (graphKind == "raw") {
    options.cudaGraphKind = CudaGraphKind::Raw;
  } else if (graphKind == "point-batch") {
    options.cudaGraphKind = CudaGraphKind::PointBatch;
  } else if (graphKind == "camera-batch") {
    options.cudaGraphKind = CudaGraphKind::CameraBatch;
  } else if (graphKind) {
    throw std::runtime_error(usage());
  }

  const auto batchChunkSize = arguments.optionalString("--batch-chunk-size");
  options.batchChunkSizeSpecified = batchChunkSize.has_value();
  if (batchChunkSize) {
    gtsam::timing::Arguments valueArguments({"--value", *batchChunkSize});
    options.batchChunkSize = valueArguments.sizeValue("--value", 0);
  }

  const auto warmupFile = arguments.optionalString("--cuda-warmup-file");
  options.cudaWarmupFileSpecified = warmupFile.has_value();
  options.cudaWarmupFile = warmupFile.value_or("");

  const auto projectionNoise = arguments.optionalString("--projection-noise");
  const bool projectionNoiseSpecified = projectionNoise.has_value();
  if (projectionNoise) {
    setProjectionNoiseModel(&options.config, *projectionNoise);
  }

  const auto gncBackend = arguments.optionalString("--gnc");
  if (gncBackend == "cpu") {
    options.gnc.backend = GncBackend::Cpu;
  } else if (gncBackend == "cuda") {
    options.gnc.backend = GncBackend::Cuda;
  } else if (gncBackend) {
    throw std::runtime_error(usage());
  }

  bool gncOptionSpecified = false;
  const auto gncLoss = arguments.optionalString("--gnc-loss");
  gncOptionSpecified |= gncLoss.has_value();
  if (gncLoss == "tls") {
    options.gnc.lossType = GncLossType::TLS;
  } else if (gncLoss == "gm") {
    options.gnc.lossType = GncLossType::GM;
  } else if (gncLoss) {
    throw std::runtime_error(usage());
  }

  const auto outlierFraction =
      arguments.optionalString("--gnc-outlier-fraction");
  gncOptionSpecified |= outlierFraction.has_value();
  if (outlierFraction) {
    gtsam::timing::Arguments valueArguments({"--value", *outlierFraction});
    options.gnc.outlierFraction =
        valueArguments.doubleValue("--value", options.gnc.outlierFraction);
  }
  if (options.gnc.outlierFraction < 0.0 || options.gnc.outlierFraction >= 1.0) {
    throw std::runtime_error("--gnc-outlier-fraction must be in [0, 1)");
  }

  const auto outlierPixels = arguments.optionalString("--gnc-outlier-pixels");
  gncOptionSpecified |= outlierPixels.has_value();
  if (outlierPixels) {
    gtsam::timing::Arguments valueArguments({"--value", *outlierPixels});
    options.gnc.outlierPixels =
        valueArguments.doubleValue("--value", options.gnc.outlierPixels);
  }

  const auto gncSeed = arguments.optionalString("--gnc-seed");
  gncOptionSpecified |= gncSeed.has_value();
  if (gncSeed) {
    gtsam::timing::Arguments valueArguments({"--value", *gncSeed});
    options.gnc.seed = static_cast<unsigned int>(
        valueArguments.uint64Value("--value", options.gnc.seed));
  }

  const auto maxOuter = arguments.optionalString("--gnc-max-outer");
  gncOptionSpecified |= maxOuter.has_value();
  if (maxOuter) {
    gtsam::timing::Arguments valueArguments({"--value", *maxOuter});
    options.gnc.maxOuterIterations =
        valueArguments.sizeValue("--value", options.gnc.maxOuterIterations);
  }

  const auto jsonPath = arguments.optionalString("--benchmark-action-json");
  options.benchmarkActionJson = jsonPath.has_value();
  options.benchmarkActionJsonPath = jsonPath.value_or("");
  options.filenames = arguments.positionals();
  arguments.validateAllConsumed();
  if (options.help) return options;

  const bool gnc = options.gnc.backend != GncBackend::None;
  if (options.profile && !options.filenames.empty()) {
    throw std::runtime_error(usage());
  }
  if ((options.profile || options.cudaStructureOnly || options.cudaLm ||
       options.cudaLmGraph) &&
      options.benchmarkActionJson) {
    throw std::runtime_error(usage());
  }
  if ((options.cudaLm && options.cudaStructureOnly) ||
      (options.cudaLmGraph && options.cudaStructureOnly) ||
      (options.cudaLm && options.cudaLmGraph)) {
    throw std::runtime_error(usage());
  }
  if (gnc &&
      (options.cudaLm || options.cudaLmGraph || options.cudaStructureOnly ||
       options.profile || options.benchmarkActionJson)) {
    throw std::runtime_error("--gnc cannot be combined with other run modes");
  }
  if (gncOptionSpecified && !gnc) {
    throw std::runtime_error("--gnc-* options require --gnc cpu|cuda");
  }
  if (gnc && projectionNoiseSpecified) {
    throw std::runtime_error(
        "--projection-noise cannot be combined with --gnc: GNC replaces "
        "robust noise with its own weighting");
  }
  if (projectionNoiseSpecified && options.cudaLm) {
    throw std::runtime_error(
        "--projection-noise only applies to factor-graph runs; use "
        "--cuda-lm-graph for CUDA robust-noise benchmarking");
  }
  if (options.cudaLinearSolverSpecified && !options.cudaLm &&
      !options.cudaLmGraph && options.gnc.backend != GncBackend::Cuda) {
    throw std::runtime_error(usage());
  }
  if (options.cudaWarmupFileSpecified && !options.cudaLm &&
      !options.cudaLmGraph && !gnc) {
    throw std::runtime_error(usage());
  }
  if (options.cudaGraphKindSpecified && !options.cudaLmGraph) {
    throw std::runtime_error("--cuda-lm-graph-kind requires --cuda-lm-graph");
  }
  if (options.batchChunkSizeSpecified &&
      (!options.cudaLmGraph ||
       options.cudaGraphKind != CudaGraphKind::PointBatch)) {
    throw std::runtime_error(
        "--batch-chunk-size only applies to --cuda-lm-graph-kind point-batch");
  }

  if (options.filenames.empty()) {
    if (options.profile) {
      options.filenames = {bal::profileDataset()};
    } else if (options.benchmarkActionJson || gnc) {
      options.filenames = {bal::defaultDataset()};
    } else {
      options.filenames = bal::standardDatasets();
    }
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
  params.maxIterations = 20;
  params.linearSolverType = solverType;
  if (solverType == NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    params.multifrontalParams.qrMode = MultifrontalParameters::QRMode::Allow;
  }

  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  const double elapsed =
      gtsam::timing::measureSeconds([&] { optimizer.optimize(); });
  std::cout << "  " << label << ": " << elapsed << " s\n";
  // SUMMARY rounds costs, so retain this precise machine-readable line.
  std::cout << "Final error: " << std::setprecision(15) << optimizer.error()
            << ", iterations: " << optimizer.iterations()
            << std::setprecision(6) << "\n";
  return elapsed;
}

NonlinearFactorGraph buildCudaGraphSfmGraph(
    const SfmData& data, CudaGraphKind graphKind, size_t batchChunkSize,
    const bal::BalBenchmarkConfig& config) {
  switch (graphKind) {
    case CudaGraphKind::Raw:
      return bal::buildGeneralSfmGraph(data, config);
    case CudaGraphKind::PointBatch:
      return bal::buildBatchSfmGraph(data, config, false, batchChunkSize);
    case CudaGraphKind::CameraBatch:
      return bal::buildCameraBatchSfmGraph(data, config);
  }
  return bal::buildGeneralSfmGraph(data, config);
}
/* ************************************************************************* */
// GNC benchmarking

const char* gncBackendName(GncBackend backend) {
  switch (backend) {
    case GncBackend::Cpu:
      return "cpu";
    case GncBackend::Cuda:
      return "cuda";
    case GncBackend::None:
      break;
  }
  return "none";
}

const char* gncLossName(GncLossType lossType) {
  return lossType == GncLossType::TLS ? "tls" : "gm";
}

// A BAL problem with a fraction of measurements corrupted by a fixed-magnitude
// offset in a random direction. Factor slot i of the graph corresponds to
// isOutlier[i].
struct CorruptedBalProblem {
  SfmData data;  // corrupted copy
  std::vector<bool> isOutlier;
  size_t outlierCount = 0;
};

// Corrupt a deterministic random subset of measurements. Only tracks with
// > 2 measurements are eligible so that every corrupted track keeps at least
// two clean views and remains constrained when GNC removes the outlier.
CorruptedBalProblem corruptBalMeasurements(const SfmData& db,
                                           double outlierFraction,
                                           double outlierPixels,
                                           unsigned int seed) {
  CorruptedBalProblem problem;
  problem.data = db;

  // Global factor slots follow buildGeneralSfmGraph order: tracks with < 2
  // measurements are skipped entirely.
  std::vector<size_t> trackSizes(db.numberTracks(), 0);
  size_t numFactors = 0;
  for (size_t j = 0; j < db.numberTracks(); ++j) {
    const size_t n = db.tracks[j].measurements.size();
    trackSizes[j] = n;
    if (n < 2) continue;
    numFactors += n;
  }
  problem.isOutlier.assign(numFactors, false);

  const size_t requested = static_cast<size_t>(
      std::llround(outlierFraction * static_cast<double>(numFactors)));
  const auto selected = gtsam::timing::SelectConstrainedOutlierMeasurements(
      trackSizes, requested, seed);

  std::mt19937 angleRng(seed);
  std::uniform_real_distribution<double> angle(0.0, 2.0 * M_PI);

  // Corrupt the chosen measurements in the SfmData copy.
  std::vector<std::vector<bool>> corruptByTrack(db.numberTracks());
  for (const auto& [trackIndex, measIndex] : selected) {
    SfmMeasurement& measurement =
        problem.data.tracks[trackIndex].measurements[measIndex];
    const double a = angle(angleRng);
    measurement.second +=
        Point2(outlierPixels * std::cos(a), outlierPixels * std::sin(a));
    if (corruptByTrack[trackIndex].empty()) {
      corruptByTrack[trackIndex].assign(
          db.tracks[trackIndex].measurements.size(), false);
    }
    corruptByTrack[trackIndex][measIndex] = true;
  }
  problem.outlierCount = selected.size();

  // Map (track, measurement) corruption flags to global factor slots.
  size_t slot = 0;
  for (size_t j = 0; j < db.numberTracks(); ++j) {
    const size_t n = db.tracks[j].measurements.size();
    if (n < 2) continue;
    for (size_t m = 0; m < n; ++m, ++slot) {
      if (!corruptByTrack[j].empty() && corruptByTrack[j][m]) {
        problem.isOutlier[slot] = true;
      }
    }
  }
  return problem;
}

struct GncClassificationMetrics {
  size_t truePositives = 0;   // outlier classified as outlier
  size_t falsePositives = 0;  // inlier classified as outlier
  size_t falseNegatives = 0;  // outlier classified as inlier
  double minInlierWeight = 1.0;
  double maxOutlierWeight = 0.0;

  double precision() const {
    const size_t denominator = truePositives + falsePositives;
    return denominator == 0 ? 1.0
                            : static_cast<double>(truePositives) / denominator;
  }
  double recall() const {
    const size_t denominator = truePositives + falseNegatives;
    return denominator == 0 ? 1.0
                            : static_cast<double>(truePositives) / denominator;
  }
};

GncClassificationMetrics classifyGncWeights(
    const Vector& weights, const std::vector<bool>& isOutlier) {
  GncClassificationMetrics metrics;
  for (size_t i = 0; i < isOutlier.size(); ++i) {
    const double w = weights[i];
    const bool classifiedOutlier = w < 0.5;
    if (isOutlier[i]) {
      metrics.maxOutlierWeight = std::max(metrics.maxOutlierWeight, w);
      if (classifiedOutlier) {
        ++metrics.truePositives;
      } else {
        ++metrics.falseNegatives;
      }
    } else {
      metrics.minInlierWeight = std::min(metrics.minInlierWeight, w);
      if (classifiedOutlier) {
        ++metrics.falsePositives;
      }
    }
  }
  return metrics;
}

// Inlier-only RMS reprojection error (pixels) of the solution.
double inlierRmsReprojectionError(const NonlinearFactorGraph& graph,
                                  const std::vector<bool>& isOutlier,
                                  const Values& solution) {
  double sumSquared = 0.0;
  size_t count = 0;
  for (size_t i = 0; i < graph.size(); ++i) {
    if (isOutlier[i]) continue;
    // error() is 0.5 * ||r||^2 for the unit-noise BAL factors.
    sumSquared += 2.0 * graph.at(i)->error(solution);
    ++count;
  }
  return count == 0 ? 0.0 : std::sqrt(sumSquared / static_cast<double>(count));
}

struct GncBenchmarkRun {
  double elapsed = 0.0;
  GncTiming timing;
  Vector weights;
  Values solution;
  size_t outerIterations = 0;
};

template <typename BaseParams>
GncBenchmarkRun runGncBenchmark(const NonlinearFactorGraph& graph,
                                const Values& initial,
                                const BaseParams& baseParams,
                                const GncRunOptions& gncOptions) {
  GncParams<BaseParams> gncParams(baseParams);
  gncParams.setLossType(gncOptions.lossType);
  if (gncOptions.maxOuterIterations != gncParams.maxIterations) {
    gncParams.maxIterations = gncOptions.maxOuterIterations;
  }

  GncBenchmarkRun run;
  std::optional<GncOptimizer<GncParams<BaseParams>>> optimizer;
  run.elapsed = gtsam::timing::measureSeconds([&] {
    optimizer.emplace(graph, initial, gncParams);
    run.solution = optimizer->optimize();
  });
  run.timing = optimizer->getTiming();
  run.weights = optimizer->getWeights();
  run.outerIterations = run.timing.iterations.size();
  return run;
}

void printGncRun(const GncBenchmarkRun& run, const CorruptedBalProblem& problem,
                 const NonlinearFactorGraph& graph,
                 const GncRunOptions& gncOptions) {
  const GncTiming& timing = run.timing;
  std::cout << "  GNC (" << gncBackendName(gncOptions.backend) << ", "
            << gncLossName(gncOptions.lossType) << "): " << run.elapsed
            << " s\n";
  std::cout << "  GNC outer iterations: " << run.outerIterations
            << " (+1 initial optimize)\n";
  std::cout << "  GNC timing breakdown:\n";
  printProfileRow("    ", "initial optimize", timing.initialOptimizeElapsed,
                  timing.totalElapsed);
  printProfileRow("    ", "weights update (factor errors)",
                  timing.sumWeightsUpdate(), timing.totalElapsed);
  printProfileRow("    ", "weighted graph rebuild", timing.sumMakeGraph(),
                  timing.totalElapsed);
  printProfileRow("    ", "base optimize calls", timing.sumBaseOptimize(),
                  timing.totalElapsed);
  printProfileRow("    ", "cost evaluation", timing.sumCostEvaluation(),
                  timing.totalElapsed);
  std::cout << "  GNC per-outer-iteration timings:\n";
  for (size_t i = 0; i < timing.iterations.size(); ++i) {
    const GncIterationTiming& it = timing.iterations[i];
    std::cout << "    outer " << i << ": total " << it.totalElapsed
              << " s (weights " << it.weightsUpdateElapsed << ", graph "
              << it.makeGraphElapsed << ", optimize " << it.baseOptimizeElapsed
              << ", cost " << it.costEvaluationElapsed << ")\n";
  }

  const GncClassificationMetrics metrics =
      classifyGncWeights(run.weights, problem.isOutlier);
  std::cout << "  GNC injected outliers: " << problem.outlierCount << " / "
            << problem.isOutlier.size() << " measurements ("
            << gncOptions.outlierPixels << " px, seed " << gncOptions.seed
            << ")\n";
  std::cout << "  GNC classification: precision " << metrics.precision()
            << ", recall " << metrics.recall() << " (TP "
            << metrics.truePositives << ", FP " << metrics.falsePositives
            << ", FN " << metrics.falseNegatives << ")\n";
  std::cout << "  GNC weight separation: min inlier weight "
            << metrics.minInlierWeight << ", max outlier weight "
            << metrics.maxOutlierWeight << "\n";
  std::cout << "Inlier RMS reprojection error: " << std::setprecision(15)
            << inlierRmsReprojectionError(graph, problem.isOutlier,
                                          run.solution)
            << " px" << std::setprecision(6) << "\n";
}
}  // namespace

int main(int argc, char* argv[]) {
  const auto options = parseBalFiles(argc, argv);
  if (options.help) {
    std::cout << usage() << '\n';
    return 0;
  }
  std::vector<TimingRow> rows;
#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
  bool cudaWarmupDone = false;
#endif

  for (const auto& filename : options.filenames) {
    const std::string dataset =
        std::filesystem::path(filename).filename().string();
    std::cout << "\nProcessing BAL file: " << filename << std::endl;
    const SfmData db = bal::loadDataset(filename);

    if (options.gnc.backend != GncBackend::None) {
      const CorruptedBalProblem problem =
          corruptBalMeasurements(db, options.gnc.outlierFraction,
                                 options.gnc.outlierPixels, options.gnc.seed);
      const NonlinearFactorGraph graph =
          bal::buildGeneralSfmGraph(problem.data, options.config);
      const Values initial = bal::buildGeneralSfmInitial(problem.data);
      std::cout << "  GNC problem: " << graph.size() << " factors, "
                << problem.outlierCount << " corrupted\n";

      if (options.gnc.backend == GncBackend::Cpu) {
        const GncBenchmarkRun run = runGncBenchmark(
            graph, initial, makeBalLevenbergMarquardtParams(options.config),
            options.gnc);
        printGncRun(run, problem, graph, options.gnc);
        continue;
      }

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
      const auto cudaParams = makeBalCudaLmParams(options.cudaLinearSolver,
                                                  CudaLmDefaults::Graph, false);

      if (options.cudaWarmupFileSpecified && !cudaWarmupDone) {
        std::cout << "  CUDA GNC warmup file: " << options.cudaWarmupFile
                  << " (timing ignored)\n";
        const SfmData warmupDb = bal::loadDataset(options.cudaWarmupFile);
        const CudaBackendLmRun warmupRun =
            runCudaBackendLm(warmupDb, cudaParams);
        std::cout << "  CUDA GNC warmup: " << warmupRun.elapsed
                  << " s ignored\n";
        cudaWarmupDone = true;
      }

      // Reference single CUDA LM solve on the corrupted problem: the gap
      // between (outer iterations x this) and the GNC total is the per-outer
      // re-conversion/re-upload overhead that a device-resident GNC removes.
      const CudaGraphLmRun referenceLm =
          runCudaGraphLm(graph, initial, cudaParams);
      std::cout << "  CUDA LM single solve (reference): " << referenceLm.elapsed
                << " s (backend measured "
                << referenceLm.backend.totalMeasuredElapsed << " s, setup "
                << referenceLm.backend.setupElapsed << " s, solve loop "
                << referenceLm.backend.solveLoopElapsed << " s)\n";

      const GncBenchmarkRun run =
          runGncBenchmark(graph, initial, cudaParams, options.gnc);
      printGncRun(run, problem, graph, options.gnc);
      continue;
#else
      throw std::runtime_error(
          "--gnc cuda requires configuring with GTSAM_ENABLE_CUDA=ON and "
          "GTSAM_ENABLE_CUDSS=ON");
#endif
    }

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
    if (options.cudaLm) {
      const auto cudaParams = makeBalCudaLmParams(
          options.cudaLinearSolver, CudaLmDefaults::Backend, options.profile);

      if (options.cudaWarmupFileSpecified && !cudaWarmupDone) {
        std::cout << "  CUDA warmup file: " << options.cudaWarmupFile
                  << " (timing ignored)\n";
        const SfmData warmupDb = bal::loadDataset(options.cudaWarmupFile);
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
      printCudaBackendLmRun(run, options.cudaLinearSolver, options.profile);
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
      const auto batch = gtsam::cuda::CudaSfmProjectionBatch::FromSfmData(
          db, context.stream());
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
          "--cuda-structure-only requires configuring with "
          "GTSAM_ENABLE_CUDA=ON");
    }
#endif

    NonlinearFactorGraph graph = buildCudaGraphSfmGraph(
        db, options.cudaGraphKind, options.batchChunkSize, options.config);
    Values initial = bal::buildGeneralSfmInitial(db);

    Ordering ordering;
    if (options.config.useSchur) {
      ordering = bal::createSchurOrdering(db, false);
    }

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
    if (options.cudaLmGraph) {
      const auto cudaParams = makeBalCudaLmParams(
          options.cudaLinearSolver, CudaLmDefaults::Graph, options.profile);

      if (options.cudaWarmupFileSpecified && !cudaWarmupDone) {
        std::cout << "  CUDA graph warmup file: " << options.cudaWarmupFile
                  << " (timing ignored)\n";
        const SfmData warmupDb = bal::loadDataset(options.cudaWarmupFile);
        const NonlinearFactorGraph warmupGraph =
            buildCudaGraphSfmGraph(warmupDb, options.cudaGraphKind,
                                   options.batchChunkSize, options.config);
        const Values warmupInitial = bal::buildGeneralSfmInitial(warmupDb);
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
      printCudaGraphLmRun(run, options.cudaLinearSolver, options.cudaGraphKind,
                          options.profile);
      continue;
    }
#endif

    const double newTime = runSolver(
        graph, initial, ordering, NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
        "MultifrontalSolver", options.config);
    double legacyTime = 0.0;
    if (!options.profile) {
      legacyTime = runSolver(graph, initial, ordering,
                             NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                             "MultifrontalCholesky", options.config);
    }

    if (!options.profile) {
      rows.push_back({dataset, legacyTime, newTime});
    }
  }
  if (!options.profile && !options.cudaStructureOnly && !options.cudaLm &&
      !options.cudaLmGraph && options.gnc.backend == GncBackend::None) {
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
