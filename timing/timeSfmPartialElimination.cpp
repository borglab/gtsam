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
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalSolver.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/SfmBalBenchmark.h"
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
  --lambda VALUE  Global identity damping (default: 0.04)
  --leaf-aggregate N  Total sibling-leaf problem size per task (default: 2048)
  --leaf-mode MODE    bounded or separator (default: bounded)
  --merge N       Multifrontal general merge dimension cap (default: 32)
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

}  // namespace

int main(int argc, char* argv[]) {
  try {
    Arguments arguments(argc, argv);
    const bool help = arguments.helpRequested();
    const std::optional<std::string> dataset =
        arguments.optionalString("--dataset");
    const size_t warmups = arguments.sizeValue("--warmup", 1);
    const size_t repetitions = arguments.sizeValue("--repeats", 7);
    const double lambda = arguments.doubleValue("--lambda", 0.04);
    const size_t leafAggregationProblemSize =
        arguments.sizeValue("--leaf-aggregate", 2048);
    const std::string leafMode =
        arguments.optionalString("--leaf-mode").value_or("bounded");
    const size_t mergeCap = arguments.sizeValue("--merge", 32);
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

    const SfmData data = loadDataset(filename);
    const GaussianFactorGraph damped =
        createDampedPointBatchSystem(data, lambda);
    const Ordering pointFirstOrdering = createSchurOrdering(data, false);

    MultifrontalSolver::Parameters solverParameters;
    solverParameters.leafAggregationProblemSize =
        leafAggregationProblemSize;
    if (leafMode == "bounded") {
      solverParameters.leafMode = MultifrontalParameters::LeafMode::Bounded;
    } else if (leafMode == "separator") {
      solverParameters.leafMode =
          MultifrontalParameters::LeafMode::SameSeparator;
    } else {
      throw std::invalid_argument(
          "--leaf-mode must be bounded or separator");
    }
    solverParameters.mergeDimCap = mergeCap;
    solverParameters.qrMode = MultifrontalParameters::QRMode::Off;
    if (reportStructure) solverParameters.reportStream = &std::cout;
    std::unique_ptr<MultifrontalSolver> solver;
    const double symbolicSetupMilliseconds =
        1000.0 * measureSeconds([&] {
          solver = std::make_unique<MultifrontalSolver>(
              damped, pointFirstOrdering, data.numberTracks(),
              solverParameters);
        });

    const CompactCameraSystem compactReference =
        buildPointBatchCameraSystemParallel(damped);
    solver->eliminatePartialInPlace(damped);
    const GaussianFactorGraph remainingReference =
        solver->remainingFactorGraph();

    Ordering cameraOrdering;
    for (size_t camera = 0; camera < data.numberCameras(); ++camera) {
      cameraOrdering.push_back(C(camera));
    }
    const auto [partialHessian, partialRhs] =
        remainingReference.hessian(cameraOrdering);
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
          buildPointBatchCameraSystemParallel(damped);
      benchmarkSink += system.blocks.size() + system.landmarks.size();
    };
    const auto loadPartial = [&] { solver->load(damped); };
    const auto eliminatePartial = [&] {
      solver->eliminatePartialInPlace();
      benchmarkSink += solver->roots().size();
    };
    const auto runPartial = [&] {
      loadPartial();
      eliminatePartial();
    };
    const auto runPartialFused = [&] {
      solver->eliminatePartialInPlace(damped);
      benchmarkSink += solver->roots().size();
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
    std::vector<double> fusedPartialSamples;
    std::vector<double> partialSamples;
    std::vector<double> loadSamples;
    std::vector<double> eliminationSamples;
    compactSamples.reserve(repetitions);
    fusedPartialSamples.reserve(repetitions);
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
        fusedPartialSamples.push_back(1000.0 * measureSeconds(runPartialFused));
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
    const TimingSummary fusedPartialSummary =
        summarizeSamples(fusedPartialSamples, MedianPolicy::kAverageMiddle);
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
              << "Leaf aggregation problem size: "
              << leafAggregationProblemSize << "\n"
              << "Leaf mode: " << leafMode << "\n"
              << "General merge cap: " << mergeCap << "\n"
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
    printSummary("Compact reduced-camera assembly", compactSummary);
    printSummary("Partial multifrontal fused total", fusedPartialSummary);
    printSummary("Partial multifrontal separate total", partialSummary);
    printSummary("  load factors", loadSummary);
    printSummary("  eliminate points", eliminationSummary);
    std::cout << "\nFused partial/compact median ratio: "
              << fusedPartialSummary.median / compactSummary.median << "x\n";

    if (benchmarkSink == 0) std::cerr << "Unexpected empty benchmark result\n";
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
