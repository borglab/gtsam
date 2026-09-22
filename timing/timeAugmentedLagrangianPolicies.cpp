/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeAugmentedLagrangianPolicies.cpp
 * @brief Compare Aggressive and BCL augmented-Lagrangian update policies.
 */

#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"

namespace {

using gtsam::AugmentedLagrangianOptimizer;
using gtsam::AugmentedLagrangianParams;
using gtsam::AugmentedLagrangianState;
using gtsam::AugmentedLagrangianUpdatePolicy;
using gtsam::AugmentedLagrangianUpdateType;
using gtsam::ConstrainedOptProblem;
using gtsam::Double_;
using gtsam::ExpressionEqualityConstraint;
using gtsam::NonlinearEqualityConstraints;
using gtsam::NonlinearFactorGraph;
using gtsam::NonlinearInequalityConstraints;
using gtsam::ScalarExpressionInequalityConstraint;
using gtsam::Symbol;
using gtsam::Values;
using gtsam::Vector1;

const Symbol kX('x', 0);
const Symbol kY('y', 0);
const Double_ kXExpression(kX);
const Double_ kYExpression(kY);

struct Options {
  size_t warmups = 1;
  size_t repetitions = 5;
  std::optional<std::string> benchmarkActionJson;
};

struct Scenario {
  std::string name;
  ConstrainedOptProblem problem;
  Values initial;
};

struct RunResult {
  bool success = false;
  double milliseconds = 0.0;
  double stationarity = std::numeric_limits<double>::infinity();
  double generalizedResidual = std::numeric_limits<double>::infinity();
  double kktResidual = std::numeric_limits<double>::infinity();
  double objective = std::numeric_limits<double>::infinity();
  size_t outerIterations = 0;
  size_t totalLmIterations = 0;
  size_t multiplierUpdates = 0;
  size_t penaltyUpdates = 0;
  double maximumPenalty = 0.0;
};

struct Summary {
  double successRate = 0.0;
  double medianMilliseconds = 0.0;
  double medianStationarity = 0.0;
  double medianGeneralizedResidual = 0.0;
  double medianKktResidual = 0.0;
  double medianObjective = 0.0;
  double medianOuterIterations = 0.0;
  double medianTotalLmIterations = 0.0;
  double medianMultiplierUpdates = 0.0;
  double medianPenaltyUpdates = 0.0;
  double maximumPenalty = 0.0;
};

struct KktDiagnostics {
  double stationarity = 0.0;
  double generalizedResidual = 0.0;
};

/* ************************************************************************* */
Values ScalarValues(double x) {
  Values values;
  values.insert(kX, x);
  return values;
}

/* ************************************************************************* */
Values PairValues(double x, double y) {
  Values values;
  values.insert(kX, x);
  values.insert(kY, y);
  return values;
}

/* ************************************************************************* */
NonlinearFactorGraph ScalarCost(double target, double sigma = 1.0) {
  NonlinearFactorGraph costs;
  costs.addPrior(kX, target, gtsam::noiseModel::Isotropic::Sigma(1, sigma));
  return costs;
}

/* ************************************************************************* */
NonlinearEqualityConstraints Equality(const Double_& expression, double rhs,
                                      double sigma = 1.0) {
  NonlinearEqualityConstraints constraints;
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      expression, rhs, Vector1(sigma));
  return constraints;
}

/* ************************************************************************* */
NonlinearInequalityConstraints LessEqual(const Double_& expression, double rhs,
                                         double sigma = 1.0) {
  NonlinearInequalityConstraints constraints;
  constraints.emplace_shared<ScalarExpressionInequalityConstraint>(
      expression - Double_(rhs), sigma);
  return constraints;
}

/* ************************************************************************* */
Scenario EqualityOnlyScenario() {
  return {
      "equality_only",
      ConstrainedOptProblem(ScalarCost(2.0), Equality(kXExpression, 1.0), {}),
      ScalarValues(-2.0)};
}

/* ************************************************************************* */
Scenario ActiveInequalityScenario() {
  return {
      "active_inequality",
      ConstrainedOptProblem(ScalarCost(2.0), {}, LessEqual(kXExpression, 0.0)),
      ScalarValues(2.0)};
}

/* ************************************************************************* */
Scenario InactiveInequalityScenario() {
  return {
      "inactive_inequality",
      ConstrainedOptProblem(ScalarCost(-1.0), {}, LessEqual(kXExpression, 0.0)),
      ScalarValues(1.0)};
}

/* ************************************************************************* */
Scenario MixedScenario(const std::string& name, double equalitySigma,
                       double inequalitySigma) {
  NonlinearFactorGraph costs;
  costs.addPrior(kX, 1.5, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
  costs.addPrior(kY, 1.2, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
  return {name,
          ConstrainedOptProblem(
              costs, Equality(kXExpression + kYExpression, 1.0, equalitySigma),
              LessEqual(kXExpression, 0.4, inequalitySigma)),
          PairValues(2.0, -1.0)};
}

/* ************************************************************************* */
Scenario DifficultStartScenario() {
  NonlinearFactorGraph costs;
  costs.addPrior(kX, 1.0, gtsam::noiseModel::Unit::Create(1));
  costs.addPrior(kY, 1.0, gtsam::noiseModel::Unit::Create(1));
  const Double_ circle =
      kXExpression * kXExpression + kYExpression * kYExpression;
  const Double_ ellipse =
      4.0 * kXExpression * kXExpression + 0.25 * kYExpression * kYExpression;
  return {"difficult_start",
          ConstrainedOptProblem(costs, Equality(circle, 1.0),
                                LessEqual(ellipse, 1.0)),
          PairValues(-3.0, 3.0)};
}

/* ************************************************************************* */
std::vector<Scenario> Scenarios() {
  std::vector<Scenario> scenarios;
  scenarios.push_back(EqualityOnlyScenario());
  scenarios.push_back(ActiveInequalityScenario());
  scenarios.push_back(InactiveInequalityScenario());
  scenarios.push_back(MixedScenario("mixed", 1.0, 1.0));
  scenarios.push_back(MixedScenario("poorly_scaled", 0.01, 10.0));
  scenarios.push_back(DifficultStartScenario());
  return scenarios;
}

/* ************************************************************************* */
AugmentedLagrangianParams::shared_ptr Params(
    AugmentedLagrangianUpdatePolicy policy) {
  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->updatePolicy = policy;
  params->maxIterations = 40;
  params->absoluteViolationTolerance = 1e-7;
  params->relativeViolationTolerance = 1e-9;
  params->absoluteCostTolerance = 1e-9;
  params->relativeCostTolerance = 1e-9;
  params->absoluteStationarityTolerance = 1e-7;
  params->lmParams.maxIterations = 100;
  params->storeOptProgress = true;
  return params;
}

/* ************************************************************************* */
void CountUpdate(AugmentedLagrangianUpdateType update, RunResult* result) {
  if (update == AugmentedLagrangianUpdateType::Multiplier ||
      update == AugmentedLagrangianUpdateType::MultiplierAndPenalty) {
    ++result->multiplierUpdates;
  }
  if (update == AugmentedLagrangianUpdateType::Penalty ||
      update == AugmentedLagrangianUpdateType::MultiplierAndPenalty) {
    ++result->penaltyUpdates;
  }
}

/* ************************************************************************* */
double InfinityNorm(const gtsam::Vector& vector) {
  return vector.size() == 0 ? 0.0 : vector.cwiseAbs().maxCoeff();
}

/* ************************************************************************* */
KktDiagnostics EvaluateFinalKkt(const Scenario& scenario,
                                const AugmentedLagrangianOptimizer& optimizer,
                                const AugmentedLagrangianState& final) {
  KktDiagnostics diagnostics;
  const auto gradient = optimizer.augmentedLagrangianFunction(final)
                            .linearize(final.values)
                            ->gradientAtZero();
  for (const auto& keyGradient : gradient) {
    diagnostics.stationarity =
        std::max(diagnostics.stationarity, InfinityNorm(keyGradient.second));
  }
  for (const auto& equality : scenario.problem.eConstraints()) {
    diagnostics.generalizedResidual =
        std::max(diagnostics.generalizedResidual,
                 InfinityNorm(equality->whitenedError(final.values)));
  }
  for (size_t i = 0; i < scenario.problem.iConstraints().size(); ++i) {
    const double expression =
        scenario.problem.iConstraints().at(i)->whitenedExpr(final.values)(0);
    const double projectedResidual =
        std::max(expression, -final.lambdaIneq.at(i) / final.muIneq);
    diagnostics.generalizedResidual =
        std::max(diagnostics.generalizedResidual, std::abs(projectedResidual));
  }
  return diagnostics;
}

/* ************************************************************************* */
RunResult RunOnce(const Scenario& scenario,
                  AugmentedLagrangianUpdatePolicy policy) {
  RunResult result;
  const auto params = Params(policy);
  const AugmentedLagrangianOptimizer optimizer(scenario.problem,
                                               scenario.initial, params);
  const auto start = std::chrono::steady_clock::now();
  try {
    optimizer.optimize();
    result.milliseconds = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - start)
                              .count();

    const auto& progress = optimizer.progress();
    if (progress.size() < 2) {
      return result;
    }
    const AugmentedLagrangianState& final = progress.back();
    const KktDiagnostics diagnostics =
        EvaluateFinalKkt(scenario, optimizer, final);
    result.stationarity = diagnostics.stationarity;
    result.generalizedResidual = diagnostics.generalizedResidual;
    result.kktResidual =
        std::max(result.stationarity, result.generalizedResidual);
    result.objective = final.cost;
    result.outerIterations = final.iteration;
    result.totalLmIterations = final.totalUnconstrainedIterations;
    for (size_t i = 1; i < progress.size(); ++i) {
      CountUpdate(progress.at(i).updateType, &result);
    }
    for (const AugmentedLagrangianState& state : progress) {
      result.maximumPenalty =
          std::max({result.maximumPenalty, state.muEq, state.muIneq});
    }
    result.success = std::isfinite(result.kktResidual) &&
                     result.generalizedResidual <= 1e-5 &&
                     result.stationarity <= 1e-4;
  } catch (const std::exception&) {
    // A failed solve is intentionally retained in the success-rate metric.
    result.milliseconds = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - start)
                              .count();
  }
  return result;
}

/* ************************************************************************* */
std::vector<double> Values(const std::vector<RunResult>& runs,
                           double RunResult::* member) {
  std::vector<double> values;
  values.reserve(runs.size());
  for (const RunResult& run : runs) {
    values.push_back(run.*member);
  }
  return values;
}

/* ************************************************************************* */
std::vector<double> Counts(const std::vector<RunResult>& runs,
                           size_t RunResult::* member) {
  std::vector<double> values;
  values.reserve(runs.size());
  for (const RunResult& run : runs) {
    values.push_back(static_cast<double>(run.*member));
  }
  return values;
}

/* ************************************************************************* */
double Median(const std::vector<double>& values) {
  return gtsam::timing::summarizeSamples(
             values, gtsam::timing::MedianPolicy::kAverageMiddle)
      .median;
}

/* ************************************************************************* */
Summary Summarize(const std::vector<RunResult>& runs) {
  Summary summary;
  size_t successes = 0;
  for (const RunResult& run : runs) {
    successes += run.success ? 1 : 0;
    summary.maximumPenalty =
        std::max(summary.maximumPenalty, run.maximumPenalty);
  }
  summary.successRate = 100.0 * static_cast<double>(successes) / runs.size();
  summary.medianMilliseconds = Median(Values(runs, &RunResult::milliseconds));
  summary.medianStationarity = Median(Values(runs, &RunResult::stationarity));
  summary.medianGeneralizedResidual =
      Median(Values(runs, &RunResult::generalizedResidual));
  summary.medianKktResidual = Median(Values(runs, &RunResult::kktResidual));
  summary.medianObjective = Median(Values(runs, &RunResult::objective));
  summary.medianOuterIterations =
      Median(Counts(runs, &RunResult::outerIterations));
  summary.medianTotalLmIterations =
      Median(Counts(runs, &RunResult::totalLmIterations));
  summary.medianMultiplierUpdates =
      Median(Counts(runs, &RunResult::multiplierUpdates));
  summary.medianPenaltyUpdates =
      Median(Counts(runs, &RunResult::penaltyUpdates));
  return summary;
}

/* ************************************************************************* */
std::string PolicyName(AugmentedLagrangianUpdatePolicy policy) {
  return policy == AugmentedLagrangianUpdatePolicy::Aggressive ? "Aggressive"
                                                               : "BCL";
}

/* ************************************************************************* */
void PrintSummary(const Scenario& scenario,
                  AugmentedLagrangianUpdatePolicy policy,
                  const Summary& summary) {
  std::cout << scenario.name << ',' << PolicyName(policy) << ','
            << summary.successRate << ',' << std::setprecision(9)
            << summary.medianMilliseconds << ',' << summary.medianStationarity
            << ',' << summary.medianGeneralizedResidual << ','
            << summary.medianKktResidual << ',' << summary.medianObjective
            << ',' << summary.medianOuterIterations << ','
            << summary.medianTotalLmIterations << ','
            << summary.medianMultiplierUpdates << ','
            << summary.medianPenaltyUpdates << ',' << summary.maximumPenalty
            << '\n';
}

/* ************************************************************************* */
void AddMetrics(const Scenario& scenario,
                AugmentedLagrangianUpdatePolicy policy, const Summary& summary,
                std::vector<gtsam::timing::BenchmarkMetric>* metrics) {
  const std::string prefix = scenario.name + "/" + PolicyName(policy) + "/";
  metrics->push_back({prefix + "success_rate", "percent", summary.successRate});
  metrics->push_back(
      {prefix + "median_runtime", "ms", summary.medianMilliseconds});
  metrics->push_back(
      {prefix + "stationarity", "residual", summary.medianStationarity});
  metrics->push_back({prefix + "generalized_residual", "residual",
                      summary.medianGeneralizedResidual});
  metrics->push_back(
      {prefix + "kkt_residual", "residual", summary.medianKktResidual});
  metrics->push_back({prefix + "objective", "cost", summary.medianObjective});
  metrics->push_back({prefix + "outer_iterations", "iterations",
                      summary.medianOuterIterations});
  metrics->push_back({prefix + "lm_iterations", "iterations",
                      summary.medianTotalLmIterations});
  metrics->push_back({prefix + "multiplier_updates", "updates",
                      summary.medianMultiplierUpdates});
  metrics->push_back(
      {prefix + "penalty_updates", "updates", summary.medianPenaltyUpdates});
  metrics->push_back(
      {prefix + "maximum_penalty", "rho", summary.maximumPenalty});
}

/* ************************************************************************* */
Options ParseOptions(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  if (arguments.helpRequested()) {
    std::cout << "Usage: timeAugmentedLagrangianPolicies [--warmup N] "
                 "[--repeats N] [--benchmark-action-json FILE]\n";
    std::exit(0);
  }
  Options options;
  options.warmups = arguments.sizeValue("--warmup", 1);
  options.repetitions = arguments.sizeValue("--repeats", 5);
  options.benchmarkActionJson =
      arguments.optionalString("--benchmark-action-json");
  arguments.validateAllConsumed();
  if (options.repetitions == 0) {
    throw std::invalid_argument("--repeats must be at least one");
  }
  return options;
}

}  // namespace

/* ************************************************************************* */
int main(int argc, char** argv) {
  const Options options = ParseOptions(argc, argv);
  const std::vector<Scenario> scenarios = Scenarios();
  const std::vector<AugmentedLagrangianUpdatePolicy> policies{
      AugmentedLagrangianUpdatePolicy::Aggressive,
      AugmentedLagrangianUpdatePolicy::BCL};
  std::vector<gtsam::timing::BenchmarkMetric> metrics;

  std::cout << "scenario,policy,success_percent,median_ms,stationarity,"
               "generalized_residual,kkt_residual,objective,outer_iterations,"
               "lm_iterations,multiplier_updates,penalty_updates,max_penalty\n";
  for (const Scenario& scenario : scenarios) {
    for (size_t warmup = 0; warmup < options.warmups; ++warmup) {
      const bool reverse = warmup % 2 == 1;
      static_cast<void>(RunOnce(scenario, policies.at(reverse ? 1 : 0)));
      static_cast<void>(RunOnce(scenario, policies.at(reverse ? 0 : 1)));
    }

    std::vector<RunResult> aggressiveRuns;
    std::vector<RunResult> bclRuns;
    aggressiveRuns.reserve(options.repetitions);
    bclRuns.reserve(options.repetitions);
    for (size_t repetition = 0; repetition < options.repetitions;
         ++repetition) {
      if (repetition % 2 == 0) {
        aggressiveRuns.push_back(
            RunOnce(scenario, AugmentedLagrangianUpdatePolicy::Aggressive));
        bclRuns.push_back(
            RunOnce(scenario, AugmentedLagrangianUpdatePolicy::BCL));
      } else {
        bclRuns.push_back(
            RunOnce(scenario, AugmentedLagrangianUpdatePolicy::BCL));
        aggressiveRuns.push_back(
            RunOnce(scenario, AugmentedLagrangianUpdatePolicy::Aggressive));
      }
    }

    const Summary aggressive = Summarize(aggressiveRuns);
    const Summary bcl = Summarize(bclRuns);
    PrintSummary(scenario, AugmentedLagrangianUpdatePolicy::Aggressive,
                 aggressive);
    PrintSummary(scenario, AugmentedLagrangianUpdatePolicy::BCL, bcl);
    AddMetrics(scenario, AugmentedLagrangianUpdatePolicy::Aggressive,
               aggressive, &metrics);
    AddMetrics(scenario, AugmentedLagrangianUpdatePolicy::BCL, bcl, &metrics);
  }

  if (options.benchmarkActionJson) {
    gtsam::timing::writeBenchmarkActionMetrics(*options.benchmarkActionJson,
                                               metrics);
  }
  return 0;
}
