/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeTernarySfmBAL.cpp
 * @brief Compare generic and fixed-size ternary BAL linearization.
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/FixedJacobianFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/SfmBalBenchmark.h"
#include "internal/TimingUtils.h"

namespace {

using gtsam::Cal3Bundler;
using gtsam::GeneralSFMFactor2;
using gtsam::NonlinearFactorGraph;
using gtsam::Ordering;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Values;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::K;
using gtsam::symbol_shorthand::P;
using std::size_t;

enum class Mode { kGeneric, kTernary, kPackedBinary };

const char* modeName(Mode mode) {
  switch (mode) {
    case Mode::kGeneric:
      return "generic_split";
    case Mode::kTernary:
      return "ternary_split";
    case Mode::kPackedBinary:
      return "binary_packed";
  }
  throw std::logic_error("Unknown BAL benchmark mode");
}

struct Options {
  std::string dataset;
  size_t warmups;
  size_t repeats;
  size_t linearizeRepeats;
  size_t iterations;
  std::optional<std::string> output;
};

struct TrialResult {
  size_t trial;
  Mode mode;
  double linearizeMilliseconds;
  double optimizeMilliseconds;
  double finalError;
  Values values;
};

/** GeneralSFMFactor2 with the pre-specialization linearization path. */
class GenericGeneralSFMFactor2 : public GeneralSFMFactor2<Cal3Bundler> {
 public:
  using GeneralSFMFactor2<Cal3Bundler>::GeneralSFMFactor2;

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<GenericGeneralSFMFactor2>(*this);
  }

  std::shared_ptr<gtsam::GaussianFactor> linearize(
      const Values& values) const override {
    return this->gtsam::NoiseModelFactor::linearize(values);
  }
};

Values makeSplitInitial(const gtsam::SfmData& data) {
  Values initial;
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    initial.insert(C(i), data.camera(i).pose());
    initial.insert(K(i), data.camera(i).calibration());
  }
  for (size_t j = 0; j < data.numberTracks(); ++j) {
    initial.insert(P(j), data.track(j).point3());
  }
  return initial;
}

template <class FACTOR>
NonlinearFactorGraph makeSplitGraph(
    const gtsam::SfmData& data,
    const gtsam::SharedNoiseModel& model = gtsam::noiseModel::Unit::Create(2)) {
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < data.numberTracks(); ++j) {
    const auto& measurements = data.track(j).measurements;
    if (measurements.size() < 2) continue;
    for (const auto& measurement : measurements) {
      graph.emplace_shared<FACTOR>(measurement.second, model,
                                   C(measurement.first), P(j),
                                   K(measurement.first));
    }
  }
  return graph;
}

template <class FACTOR>
size_t countLinearFactors(const gtsam::GaussianFactorGraph& graph) {
  return std::count_if(graph.begin(), graph.end(), [](const auto& factor) {
    return static_cast<bool>(std::dynamic_pointer_cast<FACTOR>(factor));
  });
}

TrialResult runTrial(size_t trial, Mode mode, const NonlinearFactorGraph& graph,
                     const Values& initial, const Ordering& ordering,
                     const Options& options) {
  size_t linearFactorCount = 0;
  const auto linearizeSamples = gtsam::timing::measureMilliseconds(
      [&] {
        const auto linear = graph.linearize(initial);
        linearFactorCount += linear->size();
      },
      0, options.linearizeRepeats);
  if (linearFactorCount == 0) {
    throw std::runtime_error("BAL linearization produced an empty graph");
  }

  Values optimized;
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SILENT");
  params.setOrdering(ordering);
  const auto optimizeSamples = gtsam::timing::measureMilliseconds(
      [&] {
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
        for (size_t i = 0; i < options.iterations; ++i) optimizer.iterate();
        optimized = optimizer.values();
      },
      0, 1);
  const auto linearizeSummary = gtsam::timing::summarizeSamples(
      linearizeSamples, gtsam::timing::MedianPolicy::kAverageMiddle);
  return {trial,
          mode,
          linearizeSummary.mean,
          optimizeSamples.front(),
          graph.error(optimized),
          std::move(optimized)};
}

double maxSplitDifference(const Values& expected, const Values& actual) {
  double maximum = 0.0;
  for (gtsam::Key key : expected.keys()) {
    const char symbol = gtsam::Symbol(key).chr();
    double difference = 0.0;
    if (symbol == 'c') {
      difference = gtsam::traits<Pose3>::Local(expected.at<Pose3>(key),
                                               actual.at<Pose3>(key))
                       .norm();
    } else if (symbol == 'k') {
      difference = expected.at<Cal3Bundler>(key)
                       .localCoordinates(actual.at<Cal3Bundler>(key))
                       .norm();
    } else if (symbol == 'p') {
      difference = (expected.at<Point3>(key) - actual.at<Point3>(key)).norm();
    }
    maximum = std::max(maximum, difference);
  }
  return maximum;
}

std::vector<double> samples(const std::vector<TrialResult>& results, Mode mode,
                            bool linearize) {
  std::vector<double> selected;
  for (const auto& result : results) {
    if (result.mode == mode) {
      selected.push_back(linearize ? result.linearizeMilliseconds
                                   : result.optimizeMilliseconds);
    }
  }
  return selected;
}

void writeCsv(const std::vector<TrialResult>& results,
              const std::string& path) {
  std::ofstream output = gtsam::timing::openOutputFile(path);
  output << "trial,mode,linearize_ms,optimize_ms,final_error\n";
  output << std::fixed << std::setprecision(9);
  for (const auto& result : results) {
    output << result.trial << ',' << modeName(result.mode) << ','
           << result.linearizeMilliseconds << ',' << result.optimizeMilliseconds
           << ',' << result.finalError << '\n';
  }
}

void printSummary(const Options& options, const gtsam::SfmData& data,
                  size_t projectionFactors,
                  const std::vector<TrialResult>& results,
                  double maxDifference) {
  const auto summarize = [](const std::vector<double>& values) {
    return gtsam::timing::summarizeSamples(
        values, gtsam::timing::MedianPolicy::kAverageMiddle);
  };
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Ternary SFM BAL benchmark\n"
            << "dataset=" << options.dataset
            << " cameras=" << data.numberCameras()
            << " points=" << data.numberTracks()
            << " projection_factors=" << projectionFactors << '\n';
  for (Mode mode : {Mode::kGeneric, Mode::kTernary, Mode::kPackedBinary}) {
    const auto linearize = summarize(samples(results, mode, true));
    const auto optimize = summarize(samples(results, mode, false));
    std::cout << modeName(mode) << "_linearize_ms_median=" << linearize.median
              << ' ' << modeName(mode)
              << "_optimize_ms_median=" << optimize.median << '\n';
  }
  const double genericLinearize =
      summarize(samples(results, Mode::kGeneric, true)).median;
  const double ternaryLinearize =
      summarize(samples(results, Mode::kTernary, true)).median;
  const double genericOptimize =
      summarize(samples(results, Mode::kGeneric, false)).median;
  const double ternaryOptimize =
      summarize(samples(results, Mode::kTernary, false)).median;
  std::cout << "ternary_linearize_change_percent="
            << 100.0 * (ternaryLinearize / genericLinearize - 1.0)
            << " ternary_optimize_change_percent="
            << 100.0 * (ternaryOptimize / genericOptimize - 1.0) << '\n'
            << "generic_ternary_max_local_difference=" << maxDifference << '\n';
}

void printUsage() {
  std::cout
      << "Usage: timeTernarySfmBAL [options]\n"
      << "  --dataset PATH          BAL file (default: standard 16-camera "
         "set)\n"
      << "  --warmup N              Warmup trial triplets (default: 1)\n"
      << "  --repeats N             Measured trial triplets (default: 5)\n"
      << "  --linearize-repeats N   Linearizations per trial (default: 3)\n"
      << "  --iterations N          LM iterations per trial (default: 5)\n"
      << "  --output PATH           Optional per-trial CSV\n";
}

}  // namespace

int main(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  if (arguments.helpRequested()) {
    printUsage();
    return 0;
  }
  const std::optional<std::string> datasetArgument =
      arguments.optionalString("--dataset");
  const std::string dataset =
      datasetArgument ? *datasetArgument : gtsam::timing::bal::defaultDataset();
  const Options options{
      dataset,
      arguments.sizeValue("--warmup", 1),
      arguments.sizeValue("--repeats", 5),
      arguments.sizeValue("--linearize-repeats", 3),
      arguments.sizeValue("--iterations", 5),
      arguments.optionalString("--output"),
  };
  arguments.validateAllConsumed();
  if (options.repeats == 0 || options.linearizeRepeats == 0 ||
      options.iterations == 0) {
    throw std::invalid_argument(
        "--repeats, --linearize-repeats, and --iterations must be positive");
  }

  const gtsam::SfmData data = gtsam::timing::bal::loadDataset(options.dataset);
  const Values splitInitial = makeSplitInitial(data);
  NonlinearFactorGraph genericGraph =
      makeSplitGraph<GenericGeneralSFMFactor2>(data);
  NonlinearFactorGraph ternaryGraph =
      makeSplitGraph<GeneralSFMFactor2<Cal3Bundler>>(data);
  const size_t projectionFactors = ternaryGraph.size();
  const gtsam::timing::bal::BalBenchmarkConfig config;
  NonlinearFactorGraph packedGraph =
      gtsam::timing::bal::buildGeneralSfmGraph(data, config);
  const Values packedInitial = gtsam::timing::bal::buildGeneralSfmInitial(data);

  const auto posePrior = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
  const auto pointPrior = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
  const auto calibrationPrior = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
  for (NonlinearFactorGraph* graph : {&genericGraph, &ternaryGraph}) {
    graph->addPrior(C(0), data.camera(0).pose(), posePrior);
    graph->addPrior(K(0), data.camera(0).calibration(), calibrationPrior);
    graph->addPrior(P(0), data.track(0).point3(), pointPrior);
  }
  packedGraph.addPrior(C(0), data.camera(0),
                       gtsam::noiseModel::Isotropic::Sigma(9, 1e-3));
  packedGraph.addPrior(P(0), data.track(0).point3(), pointPrior);

  const Ordering splitOrdering =
      gtsam::timing::bal::createSchurOrdering(data, true);
  const Ordering packedOrdering =
      gtsam::timing::bal::createSchurOrdering(data, false);

  const auto genericLinear = genericGraph.linearize(splitInitial);
  const auto ternaryLinear = ternaryGraph.linearize(splitInitial);
  const auto packedLinear = packedGraph.linearize(packedInitial);
  using SplitFixed = gtsam::FixedJacobianFactor<2, 6, 3, 3>;
  using PackedFixed = gtsam::FixedJacobianFactor<2, 9, 3>;
  if (countLinearFactors<SplitFixed>(*genericLinear) != 0 ||
      countLinearFactors<SplitFixed>(*ternaryLinear) != projectionFactors ||
      countLinearFactors<PackedFixed>(*packedLinear) != projectionFactors) {
    throw std::runtime_error(
        "BAL graphs did not produce expected factor types");
  }

  const auto graphFor = [&](Mode mode) -> const NonlinearFactorGraph& {
    return mode == Mode::kGeneric
               ? genericGraph
               : (mode == Mode::kTernary ? ternaryGraph : packedGraph);
  };
  const auto initialFor = [&](Mode mode) -> const Values& {
    return mode == Mode::kPackedBinary ? packedInitial : splitInitial;
  };
  const auto orderingFor = [&](Mode mode) -> const Ordering& {
    return mode == Mode::kPackedBinary ? packedOrdering : splitOrdering;
  };
  const std::array<Mode, 3> modes{Mode::kGeneric, Mode::kTernary,
                                  Mode::kPackedBinary};
  for (size_t warmup = 0; warmup < options.warmups; ++warmup) {
    for (size_t pass = 0; pass < modes.size(); ++pass) {
      const Mode mode = modes[(pass + warmup) % modes.size()];
      static_cast<void>(runTrial(warmup, mode, graphFor(mode), initialFor(mode),
                                 orderingFor(mode), options));
    }
  }

  std::vector<TrialResult> results;
  Values genericResult, ternaryResult;
  double genericError = 0.0, ternaryError = 0.0;
  for (size_t trial = 0; trial < options.repeats; ++trial) {
    for (size_t pass = 0; pass < modes.size(); ++pass) {
      const Mode mode = modes[(pass + trial) % modes.size()];
      TrialResult result =
          runTrial(trial + 1, mode, graphFor(mode), initialFor(mode),
                   orderingFor(mode), options);
      if (mode == Mode::kGeneric) {
        genericResult = result.values;
        genericError = result.finalError;
      } else if (mode == Mode::kTernary) {
        ternaryResult = result.values;
        ternaryError = result.finalError;
      }
      results.push_back(std::move(result));
    }
  }

  const double maxDifference = maxSplitDifference(genericResult, ternaryResult);
  if (std::abs(genericError - ternaryError) >
          1e-10 * std::max(1.0, std::abs(genericError)) ||
      maxDifference > 1e-8) {
    throw std::runtime_error(
        "Generic and ternary BAL optimization results differ");
  }
  if (options.output) writeCsv(results, *options.output);
  printSummary(options, data, projectionFactors, results, maxDifference);
  return 0;
}
