/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeImuIntegration.cpp
 * @brief Benchmark one-sample IMU preintegration for every PIM backend.
 */

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GalileanImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/LieGroupPreintegration.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"

namespace {

using gtsam::GalileanPreintegration;
using gtsam::LieGroupPreintegration;
using gtsam::ManifoldPreintegration;
using gtsam::Matrix9;
using gtsam::Matrix93;
using gtsam::PreintegratedCombinedMeasurementsG;
using gtsam::PreintegratedCombinedMeasurementsT;
using gtsam::PreintegratedImuMeasurementsG;
using gtsam::PreintegratedImuMeasurementsT;
using gtsam::PreintegrationParams;
using gtsam::TangentPreintegration;
using gtsam::Vector3;
using gtsam::imuBias::ConstantBias;
using gtsam::timing::MedianPolicy;
using gtsam::timing::TimingSummary;

constexpr double kDt = 0.005;
const Vector3 kMeasuredAcceleration{0.1, -0.2, 9.7};
const Vector3 kMeasuredAngularVelocity{0.01, -0.02, 0.015};
volatile double benchmarkSink = 0.0;

std::shared_ptr<PreintegrationParams> makeParams() {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(gtsam::I_3x3 * 1e-4);
  params->setGyroscopeCovariance(gtsam::I_3x3 * 1e-6);
  params->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
  return params;
}

std::shared_ptr<gtsam::PreintegrationCombinedParams> makeCombinedParams() {
  auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(gtsam::I_3x3 * 1e-4);
  params->setGyroscopeCovariance(gtsam::I_3x3 * 1e-6);
  params->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
  params->setBiasAccCovariance(gtsam::I_3x3 * 1e-7);
  params->setBiasOmegaCovariance(gtsam::I_3x3 * 1e-8);
  return params;
}

template <class PreintegrationType>
double timeBackendUpdate(size_t samples, size_t warmups, size_t repetitions) {
  const auto params = makeParams();
  const auto times = gtsam::timing::measureMilliseconds(
      [&] {
        PreintegrationType preintegration(params, ConstantBias());
        Matrix9 A;
        Matrix93 B, C;
        for (size_t sample = 0; sample < samples; ++sample) {
          preintegration.update(kMeasuredAcceleration, kMeasuredAngularVelocity,
                                kDt, &A, &B, &C);
        }
        benchmarkSink +=
            preintegration.deltaTij() + A(0, 0) + B(3, 0) + C(0, 0);
      },
      warmups, repetitions);
  const TimingSummary summary =
      gtsam::timing::summarizeSamples(times, MedianPolicy::kUpperMiddle);
  return 1e6 * summary.median / static_cast<double>(samples);
}

template <class Pim>
double timeFullIntegration(size_t samples, size_t warmups, size_t repetitions) {
  const auto params = makeParams();
  const auto times = gtsam::timing::measureMilliseconds(
      [&] {
        Pim pim(params, ConstantBias());
        for (size_t sample = 0; sample < samples; ++sample) {
          pim.integrateMeasurement(kMeasuredAcceleration,
                                   kMeasuredAngularVelocity, kDt);
        }
        benchmarkSink += pim.deltaTij() + pim.preintMeasCov()(0, 0);
      },
      warmups, repetitions);
  const TimingSummary summary =
      gtsam::timing::summarizeSamples(times, MedianPolicy::kUpperMiddle);
  return 1e6 * summary.median / static_cast<double>(samples);
}

template <class PreintegrationType, class Pim>
void printBackend(const std::string& name, size_t samples, size_t warmups,
                  size_t repetitions) {
  const double backend =
      timeBackendUpdate<PreintegrationType>(samples, warmups, repetitions);
  const double full = timeFullIntegration<Pim>(samples, warmups, repetitions);
  std::cout << std::left << std::setw(12) << name << std::right << std::fixed
            << std::setprecision(1) << std::setw(14) << backend << std::setw(16)
            << full << std::setw(16) << full - backend << '\n';
}

template <class Pim>
void printCombinedBackend(const std::string& name, size_t samples,
                          size_t warmups, size_t repetitions) {
  const auto params = makeCombinedParams();
  const auto times = gtsam::timing::measureMilliseconds(
      [&] {
        Pim pim(params, ConstantBias());
        for (size_t sample = 0; sample < samples; ++sample) {
          pim.integrateMeasurement(kMeasuredAcceleration,
                                   kMeasuredAngularVelocity, kDt);
        }
        benchmarkSink += pim.deltaTij() + pim.preintMeasCov()(0, 0);
      },
      warmups, repetitions);
  const TimingSummary summary =
      gtsam::timing::summarizeSamples(times, MedianPolicy::kUpperMiddle);
  const double full = 1e6 * summary.median / static_cast<double>(samples);
  std::cout << std::left << std::setw(12) << name << std::right << std::fixed
            << std::setprecision(1) << std::setw(20) << full << '\n';
}

}  // namespace

int main(int argc, const char* argv[]) {
  gtsam::timing::Arguments arguments(argc, argv);
  if (arguments.helpRequested()) {
    std::cout << "Usage: timeImuIntegration [--samples N] [--warmups N] "
                 "[--repetitions N]\n";
    return 0;
  }
  const size_t samples = arguments.sizeValue("--samples", 2000);
  const size_t warmups = arguments.sizeValue("--warmups", 5);
  const size_t repetitions = arguments.sizeValue("--repetitions", 21);
  arguments.validateAllConsumed();

  std::cout << "IMU integration benchmark (nanoseconds per sample)\n"
            << std::left << std::setw(12) << "backend" << std::right
            << std::setw(14) << "mean+bias" << std::setw(16) << "full PIM"
            << std::setw(16) << "PIM overhead" << '\n';
  printBackend<TangentPreintegration,
               PreintegratedImuMeasurementsT<TangentPreintegration>>(
      "Tangent", samples, warmups, repetitions);
  printBackend<ManifoldPreintegration,
               PreintegratedImuMeasurementsT<ManifoldPreintegration>>(
      "Manifold", samples, warmups, repetitions);
  printBackend<LieGroupPreintegration,
               PreintegratedImuMeasurementsT<LieGroupPreintegration>>(
      "LieGroup", samples, warmups, repetitions);
  printBackend<GalileanPreintegration, PreintegratedImuMeasurementsG>(
      "Galilean", samples, warmups, repetitions);

  std::cout << "\nCombined IMU integration benchmark (nanoseconds per sample)\n"
            << std::left << std::setw(12) << "backend" << std::right
            << std::setw(20) << "full Combined PIM" << '\n';
  printCombinedBackend<
      PreintegratedCombinedMeasurementsT<TangentPreintegration>>(
      "Tangent", samples, warmups, repetitions);
  printCombinedBackend<
      PreintegratedCombinedMeasurementsT<ManifoldPreintegration>>(
      "Manifold", samples, warmups, repetitions);
  printCombinedBackend<
      PreintegratedCombinedMeasurementsT<LieGroupPreintegration>>(
      "LieGroup", samples, warmups, repetitions);
  printCombinedBackend<PreintegratedCombinedMeasurementsG>(
      "Galilean", samples, warmups, repetitions);
  return benchmarkSink == 0.0 ? 1 : 0;
}
