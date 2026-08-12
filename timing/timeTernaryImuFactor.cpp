/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeTernaryImuFactor.cpp
 * @brief Compare generic and fixed-size ternary ImuFactor2 linearization.
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/TernaryJacobianFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"

namespace {

using Bias = gtsam::imuBias::ConstantBias;
using gtsam::NavState;
using gtsam::NonlinearFactorGraph;
using gtsam::Ordering;
using gtsam::Values;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::S;
using std::size_t;

struct Options {
  size_t steps;
  size_t warmups;
  size_t repeats;
  size_t linearizeRepeats;
  size_t iterations;
  std::optional<std::string> output;
};

struct TrialResult {
  size_t trial;
  bool generic;
  double linearizeMilliseconds;
  double optimizeMilliseconds;
  double finalError;
  Values values;
};

/** ImuFactor2 forced through the pre-specialization generic path. */
class GenericImuFactor2 : public gtsam::ImuFactor2 {
 public:
  using gtsam::ImuFactor2::ImuFactor2;

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<GenericImuFactor2>(*this);
  }

  std::shared_ptr<gtsam::GaussianFactor> linearize(
      const Values& values) const override {
    return this->gtsam::NoiseModelFactor::linearize(values);
  }
};

gtsam::PreintegratedImuMeasurements makePim() {
  auto params = gtsam::PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(gtsam::I_3x3 * 1e-4);
  params->setGyroscopeCovariance(gtsam::I_3x3 * 1e-6);
  params->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
  gtsam::PreintegratedImuMeasurements pim(params, Bias());
  for (size_t sample = 0; sample < 100; ++sample) {
    pim.integrateMeasurement(gtsam::Vector3(0.0, 0.0, 9.81),
                             gtsam::Vector3::Zero(), 0.01);
  }
  return pim;
}

Values makeInitial(size_t steps,
                   const gtsam::PreintegratedImuMeasurements& pim) {
  Values initial;
  const Bias bias;
  NavState truth;
  for (size_t i = 0; i <= steps; ++i) {
    gtsam::Vector9 perturbation = gtsam::Vector9::Zero();
    perturbation(0) = 1e-3 * std::sin(0.1 * static_cast<double>(i));
    perturbation(3) = 1e-2 * std::cos(0.03 * static_cast<double>(i));
    perturbation(7) = 2e-3 * std::sin(0.07 * static_cast<double>(i));
    initial.insert(S(i), truth.retract(perturbation));
    initial.insert(B(i), Bias(gtsam::Vector3::Constant(1e-4),
                              gtsam::Vector3::Constant(-1e-5)));
    if (i < steps) truth = pim.predict(truth, bias);
  }
  return initial;
}

template <class IMU_FACTOR>
NonlinearFactorGraph makeGraph(size_t steps,
                               const gtsam::PreintegratedImuMeasurements& pim) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<NavState>>(
      S(0), NavState(), gtsam::noiseModel::Isotropic::Sigma(9, 1e-3));
  graph.emplace_shared<gtsam::PriorFactor<Bias>>(
      B(0), Bias(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-4));
  const auto biasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
  const auto stateAnchorNoise = gtsam::noiseModel::Isotropic::Sigma(9, 0.1);
  const auto biasAnchorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  for (size_t i = 0; i < steps; ++i) {
    graph.emplace_shared<IMU_FACTOR>(S(i), S(i + 1), B(i), pim);
    graph.emplace_shared<gtsam::BetweenFactor<Bias>>(B(i), B(i + 1), Bias(),
                                                     biasNoise);
    if ((i + 1) % 100 == 0) {
      graph.emplace_shared<gtsam::PriorFactor<NavState>>(S(i + 1), NavState(),
                                                         stateAnchorNoise);
      graph.emplace_shared<gtsam::PriorFactor<Bias>>(B(i + 1), Bias(),
                                                     biasAnchorNoise);
    }
  }
  return graph;
}

size_t countTernaryFactors(const gtsam::GaussianFactorGraph& graph) {
  using Ternary = gtsam::TernaryJacobianFactor<9, 9, 9, 6>;
  return std::count_if(graph.begin(), graph.end(), [](const auto& factor) {
    return static_cast<bool>(std::dynamic_pointer_cast<Ternary>(factor));
  });
}

bool exactlyEqual(const gtsam::Matrix& left, const gtsam::Matrix& right) {
  return left.rows() == right.rows() && left.cols() == right.cols() &&
         (left.array() == right.array()).all();
}

/** Verify that changing the runtime factor type changes no stored numbers. */
void verifyIdenticalLinearGraphs(const gtsam::GaussianFactorGraph& generic,
                                 const gtsam::GaussianFactorGraph& ternary) {
  if (generic.size() != ternary.size()) {
    throw std::runtime_error("Linearized IMU graph sizes differ");
  }
  for (size_t index = 0; index < generic.size(); ++index) {
    const auto genericFactor =
        std::dynamic_pointer_cast<gtsam::JacobianFactor>(generic[index]);
    const auto ternaryFactor =
        std::dynamic_pointer_cast<gtsam::JacobianFactor>(ternary[index]);
    if (!genericFactor || !ternaryFactor ||
        genericFactor->keys() != ternaryFactor->keys() ||
        genericFactor->size() != ternaryFactor->size() ||
        !exactlyEqual(genericFactor->getb(), ternaryFactor->getb())) {
      throw std::runtime_error("Linearized IMU factors differ");
    }
    auto genericBlock = genericFactor->begin();
    auto ternaryBlock = ternaryFactor->begin();
    for (; genericBlock != genericFactor->end();
         ++genericBlock, ++ternaryBlock) {
      if (!exactlyEqual(genericFactor->getA(genericBlock),
                        ternaryFactor->getA(ternaryBlock))) {
        throw std::runtime_error("Linearized IMU Jacobian blocks differ");
      }
    }
    const auto& genericModel = genericFactor->get_model();
    const auto& ternaryModel = ternaryFactor->get_model();
    if (static_cast<bool>(genericModel) != static_cast<bool>(ternaryModel) ||
        (genericModel && !genericModel->equals(*ternaryModel, 0.0))) {
      throw std::runtime_error("Linearized IMU noise models differ");
    }
  }
}

TrialResult runTrial(size_t trial, bool generic,
                     const NonlinearFactorGraph& graph, const Values& initial,
                     const Ordering& ordering, const Options& options) {
  size_t linearFactorCount = 0;
  const auto linearizeSamples = gtsam::timing::measureMilliseconds(
      [&] {
        const auto linear = graph.linearize(initial);
        linearFactorCount += linear->size();
      },
      0, options.linearizeRepeats);
  if (linearFactorCount == 0) {
    throw std::runtime_error("IMU linearization produced an empty graph");
  }

  Values optimized;
  const auto optimizeSamples = gtsam::timing::measureMilliseconds(
      [&] {
        gtsam::GaussNewtonOptimizer optimizer(graph, initial, ordering);
        for (size_t i = 0; i < options.iterations; ++i) optimizer.iterate();
        optimized = optimizer.values();
      },
      0, 1);
  const auto linearizeSummary = gtsam::timing::summarizeSamples(
      linearizeSamples, gtsam::timing::MedianPolicy::kAverageMiddle);
  return {trial,
          generic,
          linearizeSummary.mean,
          optimizeSamples.front(),
          graph.error(optimized),
          std::move(optimized)};
}

double maxDifference(const Values& expected, const Values& actual) {
  double maximum = 0.0;
  for (gtsam::Key key : expected.keys()) {
    const char symbol = gtsam::Symbol(key).chr();
    double difference = 0.0;
    if (symbol == 's') {
      difference = gtsam::traits<NavState>::Local(expected.at<NavState>(key),
                                                  actual.at<NavState>(key))
                       .norm();
    } else if (symbol == 'b') {
      difference =
          expected.at<Bias>(key).localCoordinates(actual.at<Bias>(key)).norm();
    }
    maximum = std::max(maximum, difference);
  }
  return maximum;
}

std::vector<double> samples(const std::vector<TrialResult>& results,
                            bool generic, bool linearize) {
  std::vector<double> selected;
  for (const auto& result : results) {
    if (result.generic == generic) {
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
    output << result.trial << ',' << (result.generic ? "generic" : "ternary")
           << ',' << result.linearizeMilliseconds << ','
           << result.optimizeMilliseconds << ',' << result.finalError << '\n';
  }
}

void printSummary(const Options& options,
                  const std::vector<TrialResult>& results,
                  double maxLocalDifference) {
  const auto summarize = [](const std::vector<double>& values) {
    return gtsam::timing::summarizeSamples(
        values, gtsam::timing::MedianPolicy::kAverageMiddle);
  };
  const auto genericLinear = summarize(samples(results, true, true));
  const auto ternaryLinear = summarize(samples(results, false, true));
  const auto genericOptimize = summarize(samples(results, true, false));
  const auto ternaryOptimize = summarize(samples(results, false, false));
  std::cout << std::fixed << std::setprecision(6)
            << "Ternary ImuFactor2 chain benchmark\n"
            << "steps=" << options.steps
            << " variables=" << 2 * (options.steps + 1) << '\n'
            << "generic_linearize_ms_median=" << genericLinear.median
            << " ternary_linearize_ms_median=" << ternaryLinear.median
            << " change_percent="
            << 100.0 * (ternaryLinear.median / genericLinear.median - 1.0)
            << '\n'
            << "generic_optimize_ms_median=" << genericOptimize.median
            << " ternary_optimize_ms_median=" << ternaryOptimize.median
            << " change_percent="
            << 100.0 * (ternaryOptimize.median / genericOptimize.median - 1.0)
            << '\n'
            << "generic_ternary_max_local_difference=" << maxLocalDifference
            << '\n';
}

void printUsage() {
  std::cout
      << "Usage: timeTernaryImuFactor [options]\n"
      << "  --steps N               IMU intervals (default: 1000)\n"
      << "  --warmup N              Alternating warmup pairs (default: 1)\n"
      << "  --repeats N             Alternating measured pairs (default: 5)\n"
      << "  --linearize-repeats N   Linearizations per trial (default: 3)\n"
      << "  --iterations N          Gauss-Newton steps per trial (default: 1)\n"
      << "  --output PATH           Optional per-trial CSV\n";
}

}  // namespace

int main(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  if (arguments.helpRequested()) {
    printUsage();
    return 0;
  }
  const Options options{
      arguments.sizeValue("--steps", 1000),
      arguments.sizeValue("--warmup", 1),
      arguments.sizeValue("--repeats", 5),
      arguments.sizeValue("--linearize-repeats", 3),
      arguments.sizeValue("--iterations", 1),
      arguments.optionalString("--output"),
  };
  arguments.validateAllConsumed();
  if (options.steps == 0 || options.repeats == 0 ||
      options.linearizeRepeats == 0 || options.iterations == 0) {
    throw std::invalid_argument(
        "--steps, --repeats, --linearize-repeats, and --iterations must be "
        "positive");
  }

  const auto pim = makePim();
  const Values initial = makeInitial(options.steps, pim);
  const NonlinearFactorGraph genericGraph =
      makeGraph<GenericImuFactor2>(options.steps, pim);
  const NonlinearFactorGraph ternaryGraph =
      makeGraph<gtsam::ImuFactor2>(options.steps, pim);
  const Ordering ordering = Ordering::Create(Ordering::COLAMD, ternaryGraph);
  const auto genericLinear = genericGraph.linearize(initial);
  const auto ternaryLinear = ternaryGraph.linearize(initial);
  verifyIdenticalLinearGraphs(*genericLinear, *ternaryLinear);
  if (countTernaryFactors(*genericLinear) != 0 ||
      countTernaryFactors(*ternaryLinear) != options.steps) {
    throw std::runtime_error(
        "IMU graphs did not produce expected factor types");
  }
  for (size_t warmup = 0; warmup < options.warmups; ++warmup) {
    for (size_t pass = 0; pass < 2; ++pass) {
      const bool generic = (warmup + pass) % 2 == 0;
      static_cast<void>(runTrial(warmup, generic,
                                 generic ? genericGraph : ternaryGraph, initial,
                                 ordering, options));
    }
  }

  std::vector<TrialResult> results;
  Values genericResult, ternaryResult;
  double genericError = 0.0, ternaryError = 0.0;
  for (size_t trial = 0; trial < options.repeats; ++trial) {
    for (size_t pass = 0; pass < 2; ++pass) {
      const bool generic = (trial + pass) % 2 == 0;
      TrialResult result =
          runTrial(trial + 1, generic, generic ? genericGraph : ternaryGraph,
                   initial, ordering, options);
      if (generic) {
        genericResult = result.values;
        genericError = result.finalError;
      } else {
        ternaryResult = result.values;
        ternaryError = result.finalError;
      }
      results.push_back(std::move(result));
    }
  }

  const double maximum = maxDifference(genericResult, ternaryResult);
  const double maximumAllowed = 1e-7 * static_cast<double>(options.steps);
  if (std::abs(genericError - ternaryError) >
          1e-10 * std::max(1.0, std::abs(genericError)) ||
      maximum > maximumAllowed) {
    throw std::runtime_error(
        "Generic and ternary IMU optimization results differ: error_delta=" +
        std::to_string(std::abs(genericError - ternaryError)) +
        " max_local_delta=" + std::to_string(maximum));
  }
  if (options.output) writeCsv(results, *options.output);
  printSummary(options, results, maximum);
  return 0;
}
