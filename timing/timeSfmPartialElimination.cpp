/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeSfmPartialElimination.cpp
 * @brief Compare compact SFM Schur assembly with partial multifrontal
 * elimination.
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/SfmBalBenchmark.h"
#include "internal/SfmCholmodBenchmark.h"
#include "internal/SfmPointBatchSchur.h"
#include "internal/TimingUtils.h"

using namespace gtsam;
using namespace gtsam::timing;
using namespace gtsam::timing::bal;
using symbol_shorthand::C;
using symbol_shorthand::P;

namespace {

std::string usage() {
  return R"(Usage: timeSfmPartialElimination [options]

Options:
  --dataset PATH  BAL dataset (default: dubrovnik-16-22106-pre)
  --warmup N      Untimed paired warmups (default: 1)
  --repeats N     Measured paired repetitions (default: 7)
  --lambda VALUE  Global identity damping (default: 0.4)
  --leaf-merge N      Algebraic leaf-merge dimension cap (default: 256)
  --leaf-aggregate N  Total sibling-leaf problem size per task (default: 2048)
  --leaf-mode MODE    bounded or separator (default: bounded)
  --merge N       Multifrontal general merge dimension cap (default: 32)
  --threads N     Workers used by both pipelines (default: 1)
  --report        Print multifrontal structure statistics
  --help          Show this message
)";
}

GaussianFactorGraph createDampedPointBatchSystem(const SfmData& data,
                                                 double lambda) {
  const BalBenchmarkConfig config;
  const NonlinearFactorGraph nonlinear =
      buildBatchSfmGraph(data, config, false, 0);
  const Values initial = buildGeneralSfmInitial(data);
  GaussianFactorGraph damped = *nonlinear.linearize(initial);

  const double sqrtLambda = std::sqrt(lambda);
  const Matrix99 cameraDamping = sqrtLambda * Matrix99::Identity();
  const Matrix3 pointDamping = sqrtLambda * Matrix3::Identity();
  for (size_t camera = 0; camera < data.numberCameras(); ++camera) {
    damped.emplace_shared<JacobianFactor>(C(camera), cameraDamping,
                                          Vector9::Zero());
  }
  for (size_t point = 0; point < data.numberTracks(); ++point) {
    damped.emplace_shared<JacobianFactor>(P(point), pointDamping,
                                          Vector3::Zero());
  }
  return damped;
}

Matrix denseCameraMatrix(const CompactCameraSystem& system) {
  Matrix result = Matrix::Zero(9 * system.cameraCount, 9 * system.cameraCount);
  for (size_t row = 0; row < system.cameraCount; ++row) {
    for (size_t column = row; column < system.cameraCount; ++column) {
      const size_t slot =
          upperCameraBlockIndex(row, column, system.cameraCount);
      if (!system.usedBlocks[slot]) continue;
      result.block<9, 9>(9 * row, 9 * column) = system.blocks[slot];
      if (row != column) {
        result.block<9, 9>(9 * column, 9 * row) =
            system.blocks[slot].transpose();
      }
    }
  }
  return result;
}

double maximumAbsoluteDifference(const Matrix& first, const Matrix& second) {
  if (first.rows() != second.rows() || first.cols() != second.cols()) {
    throw std::invalid_argument("Cannot compare matrices of different sizes");
  }
  return first.size() == 0 ? 0.0 : (first - second).cwiseAbs().maxCoeff();
}

void printSummary(const std::string& label, const TimingSummary& summary) {
  std::cout << "| " << label << " | " << summary.median << " | " << summary.mean
            << " | " << summary.minimum << " | " << summary.maximum << " |\n";
}

struct ReducedSolveTiming {
  std::string solver;
  TimingSummary timing;
  Vector solution;
  double relativeResidual = 0.0;
  size_t iterations = 0;
  double symbolicSetupMilliseconds = 0.0;
  bool succeeded = false;
  std::string failure;
};

Vector flattenCameraSolution(const VectorValues& solution, size_t cameraCount) {
  Vector result(9 * cameraCount);
  for (size_t camera = 0; camera < cameraCount; ++camera) {
    result.segment<9>(9 * camera) = solution.at(C(camera));
  }
  return result;
}

std::vector<size_t> cameraPermutation(const Ordering& ordering,
                                      size_t cameraCount) {
  if (ordering.size() != cameraCount) {
    throw std::invalid_argument("Camera ordering has the wrong size");
  }
  std::vector<size_t> permutation;
  std::vector<uint8_t> seen(cameraCount, 0);
  permutation.reserve(cameraCount);
  for (Key key : ordering) {
    const Symbol symbol(key);
    const size_t camera = symbol.index();
    if (symbol.chr() != 'c' || camera >= cameraCount || seen[camera]) {
      throw std::invalid_argument("Invalid camera ordering");
    }
    seen[camera] = 1;
    permutation.push_back(camera);
  }
  return permutation;
}

std::pair<Matrix, Vector> permuteCameraSystem(
    const Matrix& hessian, const Vector& rhs,
    const std::vector<size_t>& permutation) {
  const size_t cameraCount = permutation.size();
  Matrix permutedHessian(9 * cameraCount, 9 * cameraCount);
  Vector permutedRhs(9 * cameraCount);
  for (size_t orderedRow = 0; orderedRow < cameraCount; ++orderedRow) {
    const size_t naturalRow = permutation[orderedRow];
    permutedRhs.segment<9>(9 * orderedRow) = rhs.segment<9>(9 * naturalRow);
    for (size_t orderedColumn = 0; orderedColumn < cameraCount;
         ++orderedColumn) {
      const size_t naturalColumn = permutation[orderedColumn];
      permutedHessian.block<9, 9>(9 * orderedRow, 9 * orderedColumn) =
          hessian.block<9, 9>(9 * naturalRow, 9 * naturalColumn);
    }
  }
  return {std::move(permutedHessian), std::move(permutedRhs)};
}

Vector unpermuteCameraVector(const Vector& permuted,
                             const std::vector<size_t>& permutation) {
  Vector natural(permuted.size());
  for (size_t orderedCamera = 0; orderedCamera < permutation.size();
       ++orderedCamera) {
    natural.segment<9>(9 * permutation[orderedCamera]) =
        permuted.segment<9>(9 * orderedCamera);
  }
  return natural;
}

GaussianFactorGraph sparseCameraJacobianGraph(const Matrix& permutedHessian,
                                              const Vector& permutedRhs,
                                              const Ordering& cameraOrdering) {
  const Eigen::LLT<Matrix> factorization(permutedHessian);
  if (factorization.info() != Eigen::Success) {
    throw std::runtime_error("Reduced camera Hessian Cholesky failed");
  }
  const Matrix squareRoot = factorization.matrixU();
  const Vector jacobianRhs = factorization.matrixL().solve(permutedRhs);
  const size_t cameraCount = cameraOrdering.size();
  const double nonzeroThreshold =
      1e-14 * std::max(1.0, squareRoot.cwiseAbs().maxCoeff());
  GaussianFactorGraph graph;
  graph.reserve(cameraCount);
  for (size_t blockRow = 0; blockRow < cameraCount; ++blockRow) {
    std::vector<std::pair<Key, Matrix>> terms;
    for (size_t blockColumn = blockRow; blockColumn < cameraCount;
         ++blockColumn) {
      const Matrix99 block =
          squareRoot.block<9, 9>(9 * blockRow, 9 * blockColumn);
      if (blockColumn == blockRow ||
          block.cwiseAbs().maxCoeff() > nonzeroThreshold) {
        terms.emplace_back(cameraOrdering[blockColumn], block);
      }
    }
    graph.emplace_shared<JacobianFactor>(terms,
                                         jacobianRhs.segment<9>(9 * blockRow));
  }
  return graph;
}

CompactCameraSystem compactSystemFromDense(const Matrix& hessian,
                                           const Vector& rhs,
                                           size_t cameraCount) {
  CompactCameraSystem system;
  system.cameraCount = cameraCount;
  const size_t blockCount = cameraCount * (cameraCount + 1) / 2;
  system.blocks.resize(blockCount);
  system.usedBlocks.assign(blockCount, 0);
  system.rhs = rhs;
  for (size_t row = 0; row < cameraCount; ++row) {
    for (size_t column = row; column < cameraCount; ++column) {
      const size_t slot = upperCameraBlockIndex(row, column, cameraCount);
      system.blocks[slot] = hessian.block<9, 9>(9 * row, 9 * column);
      system.usedBlocks[slot] = system.blocks[slot].cwiseAbs().maxCoeff() > 0.0;
    }
  }
  return system;
}

GaussianFactorGraph compactCameraFactorGraph(
    const CompactCameraSystem& system) {
  GaussianFactorGraph graph;
  const Matrix99 zeroMatrix = Matrix99::Zero();
  const Vector9 zeroVector = Vector9::Zero();
  for (size_t row = 0; row < system.cameraCount; ++row) {
    const size_t diagonal = upperCameraBlockIndex(row, row, system.cameraCount);
    graph.emplace_shared<HessianFactor>(C(row), system.blocks[diagonal],
                                        system.rhs.segment<9>(9 * row), 0.0);
    for (size_t column = row + 1; column < system.cameraCount; ++column) {
      const size_t slot =
          upperCameraBlockIndex(row, column, system.cameraCount);
      if (!system.usedBlocks[slot]) continue;
      graph.emplace_shared<HessianFactor>(C(row), C(column), zeroMatrix,
                                          system.blocks[slot], zeroVector,
                                          zeroMatrix, zeroVector, 0.0);
    }
  }
  return graph;
}

ReducedSolveTiming measureReducedSolve(const std::string& label, size_t warmups,
                                       size_t repetitions,
                                       const Matrix& hessian, const Vector& rhs,
                                       const std::function<Vector()>& solve) {
  ReducedSolveTiming result;
  result.solver = label;
  try {
    for (size_t repetition = 0; repetition < warmups; ++repetition) {
      result.solution = solve();
    }
    std::vector<double> samples;
    samples.reserve(repetitions);
    for (size_t repetition = 0; repetition < repetitions; ++repetition) {
      samples.push_back(1000.0 *
                        measureSeconds([&] { result.solution = solve(); }));
    }
    result.timing = summarizeSamples(samples, MedianPolicy::kAverageMiddle);
    result.relativeResidual =
        (hessian * result.solution - rhs).norm() / std::max(1.0, rhs.norm());
    result.succeeded = true;
  } catch (const std::exception& exception) {
    result.failure = exception.what();
  }
  return result;
}

std::vector<ReducedSolveTiming> benchmarkReducedSolvers(
    const Matrix& hessian, const Vector& rhs,
    const GaussianFactorGraph& regularFactorGraph, size_t cameraCount,
    const Ordering& cameraOrdering,
    const MultifrontalSolver::Parameters& solverParameters, size_t warmups,
    size_t repetitions) {
  const std::vector<size_t> permutation =
      cameraPermutation(cameraOrdering, cameraCount);
  const auto permutedSystem = permuteCameraSystem(hessian, rhs, permutation);
  const Matrix& permutedHessian = permutedSystem.first;
  const Vector& permutedRhs = permutedSystem.second;
  GaussianFactorGraph jacobianGraph =
      sparseCameraJacobianGraph(permutedHessian, permutedRhs, cameraOrdering);
  std::unique_ptr<MultifrontalSolver> reducedMultifrontal;
  const double multifrontalSetupMilliseconds =
      1000.0 * measureSeconds([&] {
        reducedMultifrontal = std::make_unique<MultifrontalSolver>(
            jacobianGraph, cameraOrdering, solverParameters);
      });

  PCGSolverParameters pcgParameters(
      std::make_shared<BlockJacobiPreconditionerParameters>());
  pcgParameters.minIterations = 0;
  pcgParameters.maxIterations = 500;
  pcgParameters.reset = 501;
  pcgParameters.epsilon_abs = 0.0;
  pcgParameters.epsilon_rel = 1e-8;
  pcgParameters.parallel = solverParameters.numThreads != 1;
  pcgParameters.numThreads = solverParameters.numThreads;

  const CompactCameraSystem cholmodSystem =
      compactSystemFromDense(hessian, rhs, cameraCount);
  CholmodCameraSystemSolver cholmodSolver;
  size_t pcgIterations = 0;

  std::vector<ReducedSolveTiming> results;
  results.push_back(measureReducedSolve(
      "Dense Cholesky", warmups, repetitions, hessian, rhs, [&] {
        return unpermuteCameraVector(permutedHessian.llt().solve(permutedRhs),
                                     permutation);
      }));
  results.push_back(measureReducedSolve(
      "Factor Cholesky", warmups, repetitions, hessian, rhs, [&] {
        return flattenCameraSolution(
            regularFactorGraph.optimize(cameraOrdering, EliminateCholesky),
            cameraCount);
      }));
  results.push_back(measureReducedSolve(
      "MultifrontalSolver", warmups, repetitions, hessian, rhs, [&] {
        reducedMultifrontal->eliminateInPlace(jacobianGraph);
        return flattenCameraSolution(reducedMultifrontal->updateSolution(),
                                     cameraCount);
      }));
  results.back().symbolicSetupMilliseconds = multifrontalSetupMilliseconds;
  results.push_back(measureReducedSolve(
      "PCG(BlockJacobi)", warmups, repetitions, hessian, rhs, [&] {
        const PCGSolverResult pcg =
            PCGSolver(pcgParameters)
                .optimizeDetailed(regularFactorGraph, false);
        pcgIterations = pcg.stats.iterations;
        return flattenCameraSolution(pcg.solution, cameraCount);
      }));
  results.back().iterations = pcgIterations;
  if (cholmodBackendAvailable()) {
    results.push_back(measureReducedSolve(
        "CHOLMOD", warmups, repetitions, hessian, rhs,
        [&] { return cholmodSolver.solve(cholmodSystem, permutation); }));
  }
  return results;
}

void printSolverMatrixRow(const std::string& producer,
                          const TimingSummary& producerTiming,
                          const std::vector<ReducedSolveTiming>& solvers) {
  std::cout << "| " << producer;
  for (const ReducedSolveTiming& solver : solvers) {
    if (solver.succeeded) {
      std::cout << " | " << producerTiming.median + solver.timing.median;
    } else {
      std::cout << " | FAILED";
    }
  }
  std::cout << " |\n";
}

std::string firstFailureLine(const std::string& failure) {
  const size_t begin = failure.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return "unknown failure";
  const size_t end = failure.find_first_of("\r\n", begin);
  return failure.substr(begin, end - begin);
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    Arguments arguments(argc, argv);
    const bool help = arguments.helpRequested();
    const std::optional<std::string> dataset =
        arguments.optionalString("--dataset");
    const size_t warmups = arguments.sizeValue("--warmup", 1);
    const size_t repetitions = arguments.sizeValue("--repeats", 7);
    const double lambda = arguments.doubleValue("--lambda", 0.4);
    const size_t leafMergeDimCap = arguments.sizeValue("--leaf-merge", 256);
    const size_t leafAggregationProblemSize =
        arguments.sizeValue("--leaf-aggregate", 2048);
    const std::string leafMode =
        arguments.optionalString("--leaf-mode").value_or("bounded");
    const size_t mergeCap = arguments.sizeValue("--merge", 32);
    const size_t numThreads = arguments.sizeValue("--threads", 1);
    const bool reportStructure = arguments.flag("--report");
    arguments.validateAllConsumed();

    if (help) {
      std::cout << usage();
      return 0;
    }
    const std::string filename = dataset ? *dataset : defaultDataset();
    if (repetitions == 0) {
      throw std::invalid_argument("--repeats must be positive");
    }
    if (lambda <= 0.0) {
      throw std::invalid_argument("--lambda must be positive");
    }
    if (numThreads == 0) {
      throw std::invalid_argument("--threads must be positive");
    }

    const SfmData data = loadDataset(filename);
    const GaussianFactorGraph damped =
        createDampedPointBatchSystem(data, lambda);
    const CompactCameraSystem compactReference =
        buildPointBatchCameraSystemParallel(damped, numThreads);
    const GaussianFactorGraph compactReducedFactors =
        compactCameraFactorGraph(compactReference);
    const Ordering cameraOrdering = Ordering::Metis(compactReducedFactors);
    Ordering pointFirstOrdering;
    for (size_t point = 0; point < data.numberTracks(); ++point) {
      pointFirstOrdering.push_back(P(point));
    }
    pointFirstOrdering.insert(pointFirstOrdering.end(), cameraOrdering.begin(),
                              cameraOrdering.end());

    MultifrontalSolver::Parameters solverParameters;
    solverParameters.leafMergeDimCap = leafMergeDimCap;
    solverParameters.leafAggregationProblemSize = leafAggregationProblemSize;
    if (leafMode == "bounded") {
      solverParameters.leafMode = MultifrontalParameters::LeafMode::Bounded;
    } else if (leafMode == "separator") {
      solverParameters.leafMode =
          MultifrontalParameters::LeafMode::SameSeparator;
    } else {
      throw std::invalid_argument("--leaf-mode must be bounded or separator");
    }
    solverParameters.mergeDimCap = mergeCap;
    solverParameters.qrMode = MultifrontalParameters::QRMode::Off;
    solverParameters.numThreads = numThreads;
    if (reportStructure) solverParameters.reportStream = &std::cout;
    std::unique_ptr<MultifrontalSolver> solver;
    const double symbolicSetupMilliseconds =
        1000.0 * measureSeconds([&] {
          solver = std::make_unique<MultifrontalSolver>(
              damped, pointFirstOrdering, data.numberTracks(),
              solverParameters);
        });

    solver->eliminatePartialInPlace(damped);
    const GaussianFactorGraph remainingReference =
        solver->remainingFactorGraph();

    Ordering naturalCameraOrdering;
    for (size_t camera = 0; camera < data.numberCameras(); ++camera) {
      naturalCameraOrdering.push_back(C(camera));
    }
    const auto [partialHessian, partialRhs] =
        remainingReference.hessian(naturalCameraOrdering);
    const Matrix compactHessian = denseCameraMatrix(compactReference);
    const double maximumHessianDifference =
        maximumAbsoluteDifference(compactHessian, partialHessian);
    const double maximumRhsDifference =
        maximumAbsoluteDifference(compactReference.rhs, partialRhs);
    const double hessianScale =
        std::max(1.0, compactHessian.cwiseAbs().maxCoeff());
    const double rhsScale =
        std::max(1.0, compactReference.rhs.cwiseAbs().maxCoeff());
    const double relativeHessianDifference =
        maximumHessianDifference / hessianScale;
    const double relativeRhsDifference = maximumRhsDifference / rhsScale;
    if (relativeHessianDifference > 1e-8 || relativeRhsDifference > 1e-8) {
      throw std::runtime_error(
          "Partial multifrontal and compact camera systems disagree");
    }

    size_t benchmarkSink = 0;
    const auto assembleCompact = [&] {
      const CompactCameraSystem system =
          buildPointBatchCameraSystemParallel(damped, numThreads);
      benchmarkSink += system.blocks.size() + system.landmarks.size();
    };
    const auto loadPartial = [&] { solver->load(damped); };
    const auto eliminatePartial = [&] {
      solver->eliminatePartialInPlace();
      benchmarkSink += solver->roots().size();
    };
    const auto exportRemaining = [&] {
      const GaussianFactorGraph remaining = solver->remainingFactorGraph();
      benchmarkSink += remaining.size();
      if (!remaining.empty() && remaining.front()) {
        benchmarkSink += remaining.front()->size();
      }
    };
    const auto runPartial = [&] {
      loadPartial();
      eliminatePartial();
    };
    const auto runPartialFused = [&] {
      solver->eliminatePartialInPlace(damped);
      benchmarkSink += solver->roots().size();
      exportRemaining();
    };
    for (size_t repetition = 0; repetition < warmups; ++repetition) {
      if (repetition % 2 == 0) {
        assembleCompact();
        runPartialFused();
        runPartial();
      } else {
        runPartial();
        runPartialFused();
        assembleCompact();
      }
    }

    std::vector<double> compactSamples;
    std::vector<double> fusedEndToEndSamples;
    std::vector<double> fusedEliminationSamples;
    std::vector<double> exportSamples;
    std::vector<double> partialSamples;
    std::vector<double> loadSamples;
    std::vector<double> eliminationSamples;
    compactSamples.reserve(repetitions);
    fusedEndToEndSamples.reserve(repetitions);
    fusedEliminationSamples.reserve(repetitions);
    exportSamples.reserve(repetitions);
    partialSamples.reserve(repetitions);
    loadSamples.reserve(repetitions);
    eliminationSamples.reserve(repetitions);
    for (size_t repetition = 0; repetition < repetitions; ++repetition) {
      const auto timeCompact = [&] {
        compactSamples.push_back(1000.0 * measureSeconds(assembleCompact));
      };
      const auto timePartial = [&] {
        const double loadMilliseconds = 1000.0 * measureSeconds(loadPartial);
        const double eliminationMilliseconds =
            1000.0 * measureSeconds(eliminatePartial);
        loadSamples.push_back(loadMilliseconds);
        eliminationSamples.push_back(eliminationMilliseconds);
        partialSamples.push_back(loadMilliseconds + eliminationMilliseconds);
      };
      const auto timeFusedPartial = [&] {
        const double eliminationMilliseconds =
            1000.0 * measureSeconds([&] {
              solver->eliminatePartialInPlace(damped);
              benchmarkSink += solver->roots().size();
            });
        // Export must immediately follow its elimination: the returned factors
        // own compact information that may outlive the solver's clique views.
        const double exportMilliseconds =
            1000.0 * measureSeconds(exportRemaining);
        fusedEliminationSamples.push_back(eliminationMilliseconds);
        exportSamples.push_back(exportMilliseconds);
        fusedEndToEndSamples.push_back(eliminationMilliseconds +
                                       exportMilliseconds);
      };
      if (repetition % 2 == 0) {
        timeCompact();
        timeFusedPartial();
        timePartial();
      } else {
        timePartial();
        timeFusedPartial();
        timeCompact();
      }
    }

    const TimingSummary compactSummary =
        summarizeSamples(compactSamples, MedianPolicy::kAverageMiddle);
    const TimingSummary partialSummary =
        summarizeSamples(partialSamples, MedianPolicy::kAverageMiddle);
    const TimingSummary fusedEndToEndSummary =
        summarizeSamples(fusedEndToEndSamples, MedianPolicy::kAverageMiddle);
    const TimingSummary fusedEliminationSummary =
        summarizeSamples(fusedEliminationSamples, MedianPolicy::kAverageMiddle);
    const TimingSummary exportSummary =
        summarizeSamples(exportSamples, MedianPolicy::kAverageMiddle);
    const TimingSummary loadSummary =
        summarizeSamples(loadSamples, MedianPolicy::kAverageMiddle);
    const TimingSummary eliminationSummary =
        summarizeSamples(eliminationSamples, MedianPolicy::kAverageMiddle);

    std::cout << "\nSFM reduced-camera assembly benchmark\n"
              << "Dataset: " << filename << "\n"
              << "Cameras: " << data.numberCameras()
              << ", points: " << data.numberTracks() << "\n"
              << std::fixed << std::setprecision(3)
              << "Global damping lambda: " << lambda << "\n"
              << "Algebraic leaf merge dimension cap: " << leafMergeDimCap
              << "\n"
              << "Leaf aggregation problem size: " << leafAggregationProblemSize
              << "\n"
              << "Leaf mode: " << leafMode << "\n"
              << "General merge cap: " << mergeCap << "\n"
              << "Reduced-system ordering: METIS (shared camera blocks)\n"
              << "Threads per pipeline: " << numThreads << "\n"
              << "Warmups: " << warmups << ", repetitions: " << repetitions
              << "\n"
              << "Partial multifrontal symbolic setup: "
              << symbolicSetupMilliseconds << " ms\n"
              << std::scientific << "Maximum relative Hessian difference: "
              << relativeHessianDifference << "\n"
              << "Maximum relative RHS difference: " << relativeRhsDifference
              << "\n\n"
              << std::fixed
              << "| Pipeline | Median ms | Mean ms | Min ms | Max ms |\n"
              << "| --- | ---: | ---: | ---: | ---: |\n";
    printSummary("Compact complete assembly", compactSummary);
    printSummary("Partial fused end-to-end", fusedEndToEndSummary);
    printSummary("  partial elimination only", fusedEliminationSummary);
    printSummary("  retained-factor export only", exportSummary);
    printSummary("Partial separate load+eliminate (diagnostic)",
                 partialSummary);
    printSummary("  separate load", loadSummary);
    printSummary("  separate eliminate", eliminationSummary);
    std::cout << "\nPartial end-to-end/compact median ratio: "
              << fusedEndToEndSummary.median / compactSummary.median << "x\n"
              << "Partial elimination-only/compact median ratio (diagnostic): "
              << fusedEliminationSummary.median / compactSummary.median
              << "x\n";

    const std::vector<ReducedSolveTiming> compactSolvers =
        benchmarkReducedSolvers(compactHessian, compactReference.rhs,
                                compactReducedFactors, data.numberCameras(),
                                cameraOrdering, solverParameters, warmups,
                                repetitions);
    const std::vector<ReducedSolveTiming> partialSolvers =
        benchmarkReducedSolvers(partialHessian, partialRhs, remainingReference,
                                data.numberCameras(), cameraOrdering,
                                solverParameters, warmups, repetitions);
    if (compactSolvers.size() != partialSolvers.size()) {
      throw std::runtime_error("Reduced solver matrix is incomplete");
    }
    for (size_t solverIndex = 0; solverIndex < compactSolvers.size();
         ++solverIndex) {
      if ((compactSolvers[solverIndex].succeeded &&
           compactSolvers[solverIndex].relativeResidual > 1e-7) ||
          (partialSolvers[solverIndex].succeeded &&
           partialSolvers[solverIndex].relativeResidual > 1e-7)) {
        throw std::runtime_error("Reduced camera solve failed residual check");
      }
    }

    std::cout << "\nReduced-camera solve details\n"
              << "The MultifrontalSolver Hessian-to-Jacobian adapter and "
                 "symbolic setup are outside the timed solve.\n"
              << "CHOLMOD reuses symbolic analysis after warmup; its numeric "
                 "triplet assembly is included.\n"
              << "| Point elimination | Reduced solver | Elimination median "
                 "ms | Solve median ms | Total median ms | Relative residual "
                 "| Solver setup ms | Iterations |\n"
              << "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |\n"
              << std::fixed << std::setprecision(3);
    const auto printDetails = [&](const std::string& producer,
                                  const TimingSummary& producerTiming,
                                  const std::vector<ReducedSolveTiming>& rows) {
      for (const ReducedSolveTiming& row : rows) {
        std::cout << "| " << producer << " | " << row.solver << " | "
                  << producerTiming.median << " | ";
        if (row.succeeded) {
          std::cout << row.timing.median << " | "
                    << producerTiming.median + row.timing.median << " | "
                    << std::scientific << row.relativeResidual << std::fixed;
        } else {
          std::cout << "FAILED | FAILED | --";
        }
        std::cout << " | " << row.symbolicSetupMilliseconds << " | "
                  << row.iterations << " |\n";
      }
    };
    printDetails("Compact", compactSummary, compactSolvers);
    printDetails("Partial MultifrontalSolver", fusedEndToEndSummary,
                 partialSolvers);

    std::cout << "\nEnd-to-end median matrix (point elimination + reduced "
                 "solve), ms\n| Point elimination";
    for (const ReducedSolveTiming& solverTiming : compactSolvers) {
      std::cout << " | " << solverTiming.solver;
    }
    std::cout << " |\n| ---";
    for (size_t solverIndex = 0; solverIndex < compactSolvers.size();
         ++solverIndex) {
      std::cout << " | ---:";
    }
    std::cout << " |\n";
    printSolverMatrixRow("Compact", compactSummary, compactSolvers);
    printSolverMatrixRow("Partial MultifrontalSolver", fusedEndToEndSummary,
                         partialSolvers);
    for (size_t solverIndex = 0; solverIndex < compactSolvers.size();
         ++solverIndex) {
      if (!compactSolvers[solverIndex].succeeded) {
        std::cout << "FAILED Compact/" << compactSolvers[solverIndex].solver
                  << ": "
                  << firstFailureLine(compactSolvers[solverIndex].failure)
                  << "\n";
      }
      if (!partialSolvers[solverIndex].succeeded) {
        std::cout << "FAILED Partial MultifrontalSolver/"
                  << partialSolvers[solverIndex].solver << ": "
                  << firstFailureLine(partialSolvers[solverIndex].failure)
                  << "\n";
      }
    }

    if (benchmarkSink == 0) std::cerr << "Unexpected empty benchmark result\n";
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
