/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
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

#include "../timeSFMBAL.h"

#include <gtsam/nonlinear/BatchFactor.h>

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
#include <map>

namespace {
constexpr const char* kDefaultBenchmarkDataset = "dubrovnik-16-22106-pre";
constexpr const char* kProfileDataset = "dubrovnik-135-90642-pre";

using Camera = PinholeCamera<Cal3Bundler>;
using SfmFactor = GeneralSFMFactor<Camera, Point3>;

std::string usage() {
  return "Usage: timeCudaSFMBAL [--colamd] [--profile] [--cuda-structure-only] "
         "[--cuda-lm] [--cuda-lm-graph] "
         "[--cuda-linear-solver dense-schur|cudss-full-normal] "
         "[--cuda-lm-graph-kind raw|point-batch|camera-batch] "
         "[--batch-chunk-size N] "
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

enum class CudaGraphKind {
  Raw,
  PointBatch,
  CameraBatch,
};

struct RunOptions {
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

const char* yesNo(bool value) { return value ? "yes" : "no"; }

void printProfileRow(const std::string& indent, const std::string& label,
                     double elapsed, double total) {
  std::cout << indent << std::left << std::setw(30) << label << std::right
            << elapsed << " s";
  if (total > 0.0) {
    std::cout << " (" << (100.0 * elapsed / total) << "%)";
  }
  std::cout << "\n";
}

void printTransferRow(const std::string& indent, const std::string& label,
                      double elapsed, size_t bytes, double total) {
  std::cout << indent << std::left << std::setw(30) << label << std::right
            << elapsed << " s";
  if (total > 0.0) {
    std::cout << " (" << (100.0 * elapsed / total) << "%)";
  }
  std::cout << " [" << bytes << " B";
  if (elapsed > 0.0 && bytes > 0) {
    const double gib =
        static_cast<double>(bytes) / (1024.0 * 1024.0 * 1024.0);
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
  printTransferRow("    ", "download D2H memcpy",
                   result.downloadD2hCopyElapsed, result.downloadD2hBytes,
                   total);
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
    printProfileRow("      ", "accept trial copy",
                    iteration.acceptTrialElapsed, iteration.totalElapsed);

    for (const auto& attempt : iteration.attemptProfiles) {
      std::cout << "      attempt " << attempt.attempt << ": total "
                << attempt.totalElapsed << " s, lambda " << attempt.lambda
                << ", accepted: " << yesNo(attempt.accepted)
                << ", step successful: " << yesNo(attempt.stepSuccessful)
                << ", attempted trial: " << yesNo(attempt.attemptedTrial)
                << ", terminated: " << yesNo(attempt.terminated) << "\n";
      std::cout << "        costs: linearized change "
                << attempt.linearizedCostChange << ", actual change "
                << attempt.costChange << ", fidelity "
                << attempt.modelFidelity << ", trial error "
                << std::setprecision(15) << attempt.trialError
                << std::setprecision(6) << "\n";
      std::cout << "        flags: stop lambda search "
                << yesNo(attempt.stopSearchingLambda)
                << ", lambda upper bound "
                << yesNo(attempt.lambdaUpperBoundReached) << "\n";
      printProfileRow("        ", "dense Schur solve",
                      attempt.denseSchurSolveElapsed, attempt.totalElapsed);
      printProfileRow("        ", "normal equations",
                      attempt.normalEquationsElapsed, attempt.totalElapsed);
      printProfileRow("        ", "add diagonal damping",
                      attempt.addDampingElapsed, attempt.totalElapsed);
      printProfileRow("        ", "cuDSS analyze",
                      attempt.cudssAnalyzeElapsed, attempt.totalElapsed);
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
  printCudaLmSetupBreakdown(result, "  CUDA LM ");
  printCudaLmTransferBreakdown(result, "  CUDA LM ");
  printCudaLmDetailedBreakdown(result, "  CUDA LM ");
  std::cout << "Initial error: " << std::setprecision(15)
            << result.initialError << "\n";
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

CudaGraphLmRun runCudaGraphLm(const NonlinearFactorGraph& graph,
                              const Values& initial,
                              const gtsam::cuda::CudaSfmLevenbergMarquardtParams&
                                  params) {
  const auto start = std::chrono::high_resolution_clock::now();

  const auto constructionStart = std::chrono::high_resolution_clock::now();
  gtsam::cuda::CudaSfmLevenbergMarquardtOptimizer lm(graph, initial, params);
  const auto constructionEnd = std::chrono::high_resolution_clock::now();

  const auto optimizeStart = std::chrono::high_resolution_clock::now();
  const Values& optimized = lm.optimize();
  (void)optimized;
  const auto optimizeEnd = std::chrono::high_resolution_clock::now();

  const auto resultQueryStart = std::chrono::high_resolution_clock::now();
  const double initialError = lm.result().initialError;
  const double finalError = lm.error();
  const size_t iterations = lm.iterations();
  const gtsam::cuda::CudaSfmLevenbergMarquardtResult backend = lm.result();
  const auto resultQueryEnd = std::chrono::high_resolution_clock::now();

  const auto end = std::chrono::high_resolution_clock::now();

  CudaGraphLmRun run;
  run.elapsed = std::chrono::duration<double>(end - start).count();
  run.optimizerConstructionElapsed =
      std::chrono::duration<double>(constructionEnd - constructionStart)
          .count();
  run.optimizeElapsed =
      std::chrono::duration<double>(optimizeEnd - optimizeStart).count();
  run.resultQueryElapsed =
      std::chrono::duration<double>(resultQueryEnd - resultQueryStart).count();
  run.initialError = initialError;
  run.finalError = finalError;
  run.iterations = iterations;
  run.backend = backend;
  return run;
}

void printCudaGraphLmRun(const CudaGraphLmRun& run,
                         CudaLinearSolverOption solverOption,
                         CudaGraphKind graphKind) {
  const double graphBackendCallOverhead =
      run.backend.graphBackendCallElapsed - run.backend.totalMeasuredElapsed;
  const double graphApiOtherOverhead =
      run.optimizeElapsed - run.backend.totalMeasuredElapsed -
      run.backend.graphConversionElapsed - run.backend.graphValueMergeElapsed;
  const double graphApiRemainingOptimizeOverhead =
      run.optimizeElapsed - run.backend.graphConversionElapsed -
      run.backend.graphBackendCallElapsed -
      run.backend.graphValueMergeElapsed -
      run.backend.graphConvertedDataDestructionElapsed;

  std::cout << "  CUDA LM graph API: " << run.elapsed << " s\n";
  std::cout << "  CUDA LM graph linear solver: "
            << cudaLinearSolverName(solverOption) << "\n";
  std::cout << "  CUDA LM graph kind: " << cudaGraphKindName(graphKind)
            << "\n";
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
  printProfileRow("    ", "backend return/assignment",
                  graphBackendCallOverhead, run.elapsed);
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
  printCudaLmTransferBreakdown(run.backend, "  CUDA LM backend ");
  printCudaLmDetailedBreakdown(run.backend, "  CUDA LM backend ");
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
  bool cudaGraphKindSpecified = false;
  CudaGraphKind cudaGraphKind = CudaGraphKind::Raw;
  bool batchChunkSizeSpecified = false;
  size_t batchChunkSize = 0;
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
    if (strcmp(argv[i], "--cuda-lm-graph-kind") == 0) {
      if (++i >= argc || argv[i][0] == '-') {
        throw runtime_error(usage());
      }
      cudaGraphKindSpecified = true;
      if (strcmp(argv[i], "raw") == 0) {
        cudaGraphKind = CudaGraphKind::Raw;
      } else if (strcmp(argv[i], "point-batch") == 0) {
        cudaGraphKind = CudaGraphKind::PointBatch;
      } else if (strcmp(argv[i], "camera-batch") == 0) {
        cudaGraphKind = CudaGraphKind::CameraBatch;
      } else {
        throw runtime_error(usage());
      }
      continue;
    }
    if (strcmp(argv[i], "--batch-chunk-size") == 0) {
      if (++i >= argc || argv[i][0] == '-') {
        throw runtime_error(usage());
      }
      batchChunkSizeSpecified = true;
      batchChunkSize = std::stoul(argv[i]);
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
  if (cudaGraphKindSpecified && !cudaLmGraph) {
    throw runtime_error("--cuda-lm-graph-kind requires --cuda-lm-graph");
  }
  if (batchChunkSizeSpecified &&
      (!cudaLmGraph || cudaGraphKind != CudaGraphKind::PointBatch)) {
    throw runtime_error(
        "--batch-chunk-size only applies to --cuda-lm-graph-kind point-batch");
  }

  if (!filenames.empty()) {
    return {profile,
            cudaStructureOnly,
            cudaLm,
            cudaLmGraph,
            cudaLinearSolverSpecified,
            cudaLinearSolver,
            cudaGraphKindSpecified,
            cudaGraphKind,
            batchChunkSizeSpecified,
            batchChunkSize,
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
            cudaGraphKindSpecified,
            cudaGraphKind,
            batchChunkSizeSpecified,
            batchChunkSize,
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
            cudaGraphKindSpecified,
            cudaGraphKind,
            batchChunkSizeSpecified,
            batchChunkSize,
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
          cudaGraphKindSpecified,
          cudaGraphKind,
          batchChunkSizeSpecified,
          batchChunkSize,
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

NonlinearFactorGraph buildPointBatchSfmGraph(const SfmData& db,
                                             size_t chunkSize) {
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.numberTracks(); ++j) {
    const auto& measurementsForTrack = db.tracks[j].measurements;
    if (measurementsForTrack.size() < 2) continue;

    const size_t nMeasurements = measurementsForTrack.size();
    const size_t effectiveChunkSize =
        (chunkSize == 0) ? nMeasurements : std::min(chunkSize, nMeasurements);
    if (effectiveChunkSize == 0) continue;

    for (size_t start = 0; start < nMeasurements; start += effectiveChunkSize) {
      const size_t end = std::min(start + effectiveChunkSize, nMeasurements);
      std::map<Key, Point2> measurements;
      for (size_t i = start; i < end; ++i) {
        const SfmMeasurement& measurement = measurementsForTrack[i];
        measurements[C(measurement.first)] = measurement.second;
      }
      graph.add(std::make_shared<BatchFactor<SfmFactor, 2>>(
          measurements, P(j), gNoiseModel));
    }
  }
  return graph;
}

NonlinearFactorGraph buildCameraBatchSfmGraph(const SfmData& db) {
  NonlinearFactorGraph graph;
  std::vector<std::map<Key, Point2>> measurementsByCamera(db.numberCameras());

  for (size_t j = 0; j < db.numberTracks(); ++j) {
    const auto& measurementsForTrack = db.tracks[j].measurements;
    if (measurementsForTrack.size() < 2) continue;

    for (const SfmMeasurement& measurement : measurementsForTrack) {
      measurementsByCamera[measurement.first][P(j)] = measurement.second;
    }
  }

  for (size_t i = 0; i < measurementsByCamera.size(); ++i) {
    const auto& measurements = measurementsByCamera[i];
    if (measurements.empty()) continue;

    graph.add(std::make_shared<BatchFactor<SfmFactor, 2>>(
        C(i), measurements, gNoiseModel));
  }
  return graph;
}

NonlinearFactorGraph buildCudaGraphSfmGraph(const SfmData& db,
                                            CudaGraphKind graphKind,
                                            size_t batchChunkSize) {
  switch (graphKind) {
    case CudaGraphKind::Raw:
      return buildGeneralSfmGraph(db);
    case CudaGraphKind::PointBatch:
      return buildPointBatchSfmGraph(db, batchChunkSize);
    case CudaGraphKind::CameraBatch:
      return buildCameraBatchSfmGraph(db);
  }
  return buildGeneralSfmGraph(db);
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

    NonlinearFactorGraph graph = buildCudaGraphSfmGraph(
        db, options.cudaGraphKind, options.batchChunkSize);
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
        const NonlinearFactorGraph warmupGraph = buildCudaGraphSfmGraph(
            warmupDb, options.cudaGraphKind, options.batchChunkSize);
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
      printCudaGraphLmRun(run, options.cudaLinearSolver,
                          options.cudaGraphKind);
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
