/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeBetweenFactor.cpp
 * @brief Benchmark Rot3 BetweenFactor error and Jacobian evaluation.
 * @date August 2026
 * @author Codex 5.6, prompted by Frank Dellaert
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/config.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"

namespace {

using gtsam::BetweenFactor;
using gtsam::Matrix;
using gtsam::Rot3;
using gtsam::Vector3;

constexpr size_t kInputCount = 64;
volatile double timingChecksum = 0.0;

struct BenchmarkOptions {
  size_t warmups = 5;
  size_t repetitions = 20;
  size_t iterations = 100000;
  std::optional<std::string> outputPath;
};

struct Accuracy {
  double error = 0.0;
  double h1 = 0.0;
  double h2 = 0.0;
};

struct BenchmarkResults {
  std::vector<double> current;
  std::vector<double> legacy;
};

class LegacyBetweenFactorRot3 : public gtsam::NoiseModelFactorN<Rot3, Rot3> {
  using Base = gtsam::NoiseModelFactorN<Rot3, Rot3>;

  Rot3 measured_;

 public:
  LegacyBetweenFactorRot3(gtsam::Key key1, gtsam::Key key2,
                          const Rot3& measured,
                          const gtsam::SharedNoiseModel& model)
      : Base(gtsam::noiseModel::validOrDefault(measured, model), key1, key2),
        measured_(measured) {}

  gtsam::Vector evaluateError(const Rot3& r1, const Rot3& r2,
                              gtsam::OptionalMatrixType h1,
                              gtsam::OptionalMatrixType h2) const override {
    const Rot3 hx = gtsam::traits<Rot3>::Between(r1, r2, h1, h2);
    return gtsam::traits<Rot3>::Local(measured_, hx);
  }
};

std::array<Rot3, kInputCount> makeInputs(const Rot3& nominal) {
  std::array<Rot3, kInputCount> inputs;
  for (size_t i = 0; i < inputs.size(); ++i) {
    const double phase = static_cast<double>(i);
    const Vector3 perturbation(1e-3 * std::sin(phase), 1e-3 * std::cos(phase),
                               5e-4 * std::sin(2.0 * phase));
    inputs[i] = nominal.retract(perturbation);
  }
  return inputs;
}

template <class Evaluator>
Accuracy evaluateAccuracy(Evaluator&& evaluator,
                          const BetweenFactor<Rot3>& factor, const Rot3& r1,
                          const Rot3& r2, const Rot3& measured) {
  Matrix actualH1, actualH2;
  const Vector3 actual = evaluator(r1, r2, actualH1, actualH2);
  const Vector3 expected = Rot3::Logmap(measured.inverse() * r1.between(r2));

  const Matrix numericalH1 = gtsam::numericalDerivative21<Vector3, Rot3, Rot3>(
      [&factor](const Rot3& x1, const Rot3& x2) {
        return factor.evaluateError(x1, x2);
      },
      r1, r2, 1e-5);
  const Matrix numericalH2 = gtsam::numericalDerivative22<Vector3, Rot3, Rot3>(
      [&factor](const Rot3& x1, const Rot3& x2) {
        return factor.evaluateError(x1, x2);
      },
      r1, r2, 1e-5);

  return {(expected - actual).cwiseAbs().maxCoeff(),
          (numericalH1 - actualH1).cwiseAbs().maxCoeff(),
          (numericalH2 - actualH2).cwiseAbs().maxCoeff()};
}

template <class Evaluator>
double timeBatch(Evaluator&& evaluator, const Rot3& r1,
                 const std::array<Rot3, kInputCount>& inputs,
                 size_t iterations) {
  Vector3 checksum = Vector3::Zero();
  Matrix h1, h2;
  const double seconds = gtsam::timing::measureSeconds([&] {
    for (size_t i = 0; i < iterations; ++i) {
      const Vector3 error = evaluator(r1, inputs[i % kInputCount], h1, h2);
      checksum += error + h1.col(0) + h2.col(0);
    }
  });
  timingChecksum = checksum.squaredNorm();
  return seconds * 1e9 / static_cast<double>(iterations);
}

template <class CurrentEvaluator, class LegacyEvaluator>
BenchmarkResults runBenchmark(CurrentEvaluator&& currentEvaluator,
                              LegacyEvaluator&& legacyEvaluator, const Rot3& r1,
                              const std::array<Rot3, kInputCount>& inputs,
                              const BenchmarkOptions& options) {
  for (size_t i = 0; i < options.warmups; ++i) {
    static_cast<void>(
        timeBatch(currentEvaluator, r1, inputs, options.iterations));
    static_cast<void>(
        timeBatch(legacyEvaluator, r1, inputs, options.iterations));
  }

  BenchmarkResults results;
  results.current.reserve(options.repetitions);
  results.legacy.reserve(options.repetitions);
  for (size_t i = 0; i < options.repetitions; ++i) {
    if (i % 2 == 0) {
      results.current.push_back(
          timeBatch(currentEvaluator, r1, inputs, options.iterations));
      results.legacy.push_back(
          timeBatch(legacyEvaluator, r1, inputs, options.iterations));
    } else {
      results.legacy.push_back(
          timeBatch(legacyEvaluator, r1, inputs, options.iterations));
      results.current.push_back(
          timeBatch(currentEvaluator, r1, inputs, options.iterations));
    }
  }
  return results;
}

void printUsage() {
  std::cout << "Usage: timeBetweenFactor [--warmup N] [--repeats N] "
               "[--iterations N] [--output FILE]\n";
}

}  // namespace

int main(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  if (arguments.helpRequested()) {
    printUsage();
    return 0;
  }

  const BenchmarkOptions options{
      arguments.sizeValue("--warmup", 5),
      arguments.sizeValue("--repeats", 20),
      arguments.sizeValue("--iterations", 100000),
      [&] {
        const std::string path = arguments.stringValue("--output", "");
        return path.empty() ? std::optional<std::string>()
                            : std::optional<std::string>(path);
      }(),
  };
  arguments.validateAllConsumed();
  if (options.repetitions == 0 || options.iterations == 0) {
    throw std::invalid_argument(
        "--repeats and --iterations must both be at least 1");
  }

  const Rot3 r1 = Rot3::Rodrigues(0.1, 0.2, 0.3);
  const Rot3 r2 = Rot3::Rodrigues(0.4, 0.5, 0.6);
  const Rot3 noise = Rot3::Rodrigues(0.01, 0.01, 0.01);
  const Rot3 measured = r1.between(r2) * noise;
  const BetweenFactor<Rot3> factor(
      1, 2, measured, gtsam::noiseModel::Isotropic::Sigma(3, 0.05));
  const LegacyBetweenFactorRot3 legacyFactor(
      1, 2, measured, gtsam::noiseModel::Isotropic::Sigma(3, 0.05));

  const auto currentEvaluator = [&factor](const Rot3& x1, const Rot3& x2,
                                          Matrix& h1, Matrix& h2) {
    return factor.evaluateError(x1, x2, &h1, &h2);
  };
  const auto legacyEvaluator = [&legacyFactor](const Rot3& x1, const Rot3& x2,
                                               Matrix& h1, Matrix& h2) {
    return legacyFactor.evaluateError(x1, x2, &h1, &h2);
  };

  const Accuracy currentAccuracy =
      evaluateAccuracy(currentEvaluator, factor, r1, r2, measured);
  const Accuracy legacyAccuracy =
      evaluateAccuracy(legacyEvaluator, factor, r1, r2, measured);
  const auto inputs = makeInputs(r2);
  const BenchmarkResults samples =
      runBenchmark(currentEvaluator, legacyEvaluator, r1, inputs, options);
  const auto currentSummary = gtsam::timing::summarizeSamples(
      samples.current, gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto legacySummary = gtsam::timing::summarizeSamples(
      samples.legacy, gtsam::timing::MedianPolicy::kAverageMiddle);

#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
  constexpr const char* mode = "local_jacobians";
#else
  constexpr const char* mode = "legacy";
#endif

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Rot3 BetweenFactor::evaluateError benchmark\n";
  std::cout << "mode=" << mode << " warmups=" << options.warmups
            << " repetitions=" << options.repetitions
            << " iterations_per_repetition=" << options.iterations << '\n';
  std::cout << "current_nanoseconds_per_call_median=" << currentSummary.median
            << " mean=" << currentSummary.mean
            << " standard_deviation=" << currentSummary.standardDeviation
            << " minimum=" << currentSummary.minimum
            << " p90=" << currentSummary.p90
            << " maximum=" << currentSummary.maximum << '\n';
  std::cout << "legacy_nanoseconds_per_call_median=" << legacySummary.median
            << " mean=" << legacySummary.mean
            << " standard_deviation=" << legacySummary.standardDeviation
            << " minimum=" << legacySummary.minimum
            << " p90=" << legacySummary.p90
            << " maximum=" << legacySummary.maximum << '\n';
  std::cout << "median_ratio_current_over_legacy="
            << currentSummary.median / legacySummary.median << '\n';
  std::cout << std::scientific;
  std::cout << "current_max_abs_error=" << currentAccuracy.error
            << " max_abs_h1_error=" << currentAccuracy.h1
            << " max_abs_h2_error=" << currentAccuracy.h2 << '\n';
  std::cout << "legacy_max_abs_error=" << legacyAccuracy.error
            << " max_abs_h1_error=" << legacyAccuracy.h1
            << " max_abs_h2_error=" << legacyAccuracy.h2 << '\n';

  if (options.outputPath) {
    gtsam::timing::writeBenchmarkActionMetrics(
        *options.outputPath,
        {{std::string("BetweenFactorRot3/") + mode, "ns/call",
          currentSummary.median},
         {"BetweenFactorRot3/legacy_reference", "ns/call",
          legacySummary.median},
         {std::string("BetweenFactorRot3/") + mode + "/h1_error",
          "max_abs_error", currentAccuracy.h1},
         {std::string("BetweenFactorRot3/") + mode + "/h2_error",
          "max_abs_error", currentAccuracy.h2},
         {"BetweenFactorRot3/legacy_reference/h1_error", "max_abs_error",
          legacyAccuracy.h1},
         {"BetweenFactorRot3/legacy_reference/h2_error", "max_abs_error",
          legacyAccuracy.h2}});
  }

  return timingChecksum < 0.0 ? 1 : 0;
}
