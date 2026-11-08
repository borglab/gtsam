/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeIncrementalFixedLagSmoother.cpp
 * @brief   Times IncrementalFixedLagSmoother updates and marginals of the
 *          newest pose, with and without newestKeysLast.
 *
 * Problems, at 10 Hz:
 * - odom: poses, odometry, GPS at 1 Hz.
 * - imu:  as odom, plus velocities and biases with combined IMU factors.
 * - frames: as odom, plus two pose sensors, alternating, each measuring in its
 *   own frame, whose pose is a variable retained for the whole run.
 *
 * Usage: timeIncrementalFixedLagSmoother [steps] [lags, e.g. 2,6,12]
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::F;
using symbol_shorthand::V;
using symbol_shorthand::X;

namespace {

enum class Problem { Odom, Imu, Frames };

const char* name(Problem problem) {
  switch (problem) {
    case Problem::Odom:
      return "odom";
    case Problem::Imu:
      return "imu";
    default:
      return "frames";
  }
}

struct Result {
  double updateMs = 0.0;
  double marginalMs = 0.0;
  double depth = 0.0;
  double errorM = 0.0;
  size_t variables = 0;
  Values estimate;
};

size_t depthOf(const ISAM2& isam, Key key) {
  size_t depth = 0;
  for (auto clique = isam[key]; clique->parent(); clique = clique->parent()) {
    ++depth;
  }
  return depth;
}

double ms(std::chrono::steady_clock::duration d) {
  return std::chrono::duration<double, std::milli>(d).count();
}

Pose3 perturbed(const Pose3& pose, double sigmaRot, double sigmaPos,
                std::mt19937& rng) {
  std::normal_distribution<double> n(0.0, 1.0);
  Vector6 xi;
  xi << sigmaRot * n(rng), sigmaRot * n(rng), sigmaRot * n(rng),
      sigmaPos * n(rng), sigmaPos * n(rng), sigmaPos * n(rng);
  return pose.retract(xi);
}

Result run(Problem problem, double lag, bool newestKeysLast, size_t steps) {
  const double dt = 0.1;
  const double imuDt = 0.005;
  std::mt19937 rng(42);
  std::normal_distribution<double> n(0.0, 1.0);

  // A circle at 1 m/s, slowly climbing.
  const ConstantTwistScenario scenario(Vector3(0.0, 0.0, 0.1),
                                       Vector3(1.0, 0.0, 0.05));
  auto imuParams = PreintegrationCombinedParams::MakeSharedU(9.81);
  imuParams->setAccelerometerCovariance(I_3x3 * 1e-4);
  imuParams->setGyroscopeCovariance(I_3x3 * 1e-6);
  imuParams->setIntegrationCovariance(I_3x3 * 1e-8);
  imuParams->setBiasAccCovariance(I_3x3 * 1e-6);
  imuParams->setBiasOmegaCovariance(I_3x3 * 1e-8);
  const ScenarioRunner imu(scenario, imuParams, imuDt);

  const auto odomNoise = noiseModel::Diagonal::Sigmas(
      (Vector6() << Vector3::Constant(0.01), Vector3::Constant(0.05))
          .finished());
  const auto gpsNoise = noiseModel::Isotropic::Sigma(3, 0.5);
  const auto sensorNoise = noiseModel::Diagonal::Sigmas(
      (Vector6() << Vector3::Constant(0.02), Vector3::Constant(0.1))
          .finished());
  const std::vector<Pose3> sensorFrames = {
      Pose3(Rot3::Yaw(0.5), Point3(10.0, -5.0, 1.0)),
      Pose3(Rot3::Yaw(-1.0), Point3(-3.0, 8.0, 0.0))};

  IncrementalFixedLagSmootherParams params;
  params.newestKeysLast = newestKeysLast;
  IncrementalFixedLagSmoother smoother(lag, params);

  Result result;
  size_t timed = 0;
  for (size_t i = 0; i < steps; ++i) {
    const double t = static_cast<double>(i) * dt;
    const Pose3 truth = scenario.pose(t);
    NonlinearFactorGraph factors;
    Values values;
    FixedLagSmoother::KeyTimestampMap timestamps;
    KeySet retain;

    if (i == 0) {
      values.insert(X(0), truth);
      factors.addPrior(X(0), truth, noiseModel::Isotropic::Sigma(6, 1e-3));
    } else {
      const Pose3 odom =
          perturbed(scenario.pose(t - dt).between(truth), 0.01, 0.05, rng);
      values.insert(X(i),
                    smoother.calculateEstimate<Pose3>(X(i - 1)).compose(odom));
      factors.emplace_shared<BetweenFactor<Pose3>>(X(i - 1), X(i), odom,
                                                   odomNoise);
    }
    timestamps[X(i)] = t;

    if (i % 10 == 0) {
      const Point3 gps(truth.translation() +
                       0.5 * Vector3(n(rng), n(rng), n(rng)));
      factors.emplace_shared<GPSFactor>(X(i), gps, gpsNoise);
    }

    if (problem == Problem::Imu) {
      if (i == 0) {
        values.insert(V(0), scenario.velocity_n(0.0));
        values.insert(B(0), imuBias::ConstantBias());
        factors.addPrior(V(0), scenario.velocity_n(0.0),
                         noiseModel::Isotropic::Sigma(3, 0.1));
        factors.addPrior(B(0), imuBias::ConstantBias(),
                         noiseModel::Isotropic::Sigma(6, 0.1));
      } else {
        PreintegratedCombinedMeasurements pim(
            imuParams,
            smoother.calculateEstimate<imuBias::ConstantBias>(B(i - 1)));
        for (double s = t - dt; s < t - 1e-9; s += imuDt) {
          pim.integrateMeasurement(imu.measuredSpecificForce(s),
                                   imu.measuredAngularVelocity(s), imuDt);
        }
        values.insert(V(i), smoother.calculateEstimate<Vector3>(V(i - 1)));
        values.insert(
            B(i), smoother.calculateEstimate<imuBias::ConstantBias>(B(i - 1)));
        factors.emplace_shared<CombinedImuFactor>(X(i - 1), V(i - 1), X(i),
                                                  V(i), B(i - 1), B(i), pim);
      }
      timestamps[V(i)] = t;
      timestamps[B(i)] = t;
    }

    if (problem == Problem::Frames) {
      const size_t sensor = i % sensorFrames.size();
      const Pose3 measured =
          perturbed(sensorFrames[sensor].between(truth), 0.02, 0.1, rng);
      if (i < sensorFrames.size()) {
        values.insert(F(sensor),
                      perturbed(sensorFrames[sensor], 0.05, 1.0, rng));
        timestamps[F(sensor)] = t;
        retain.insert(F(sensor));
      }
      factors.emplace_shared<BetweenFactor<Pose3>>(F(sensor), X(i), measured,
                                                   sensorNoise);
    }

    const auto t0 = std::chrono::steady_clock::now();
    smoother.update(factors, values, timestamps, FactorIndices(), retain);
    const auto t1 = std::chrono::steady_clock::now();
    smoother.marginalCovariance(X(i));
    const auto t2 = std::chrono::steady_clock::now();

    // Time only once marginalization starts: before that, both settings give
    // the same ordering.
    if (t > lag + 1.0) {
      result.updateMs += ms(t1 - t0);
      result.marginalMs += ms(t2 - t1);
      result.depth += static_cast<double>(depthOf(smoother.getISAM2(), X(i)));
      result.errorM += (smoother.calculateEstimate<Pose3>(X(i)).translation() -
                        truth.translation())
                           .norm();
      ++timed;
    }
  }
  result.updateMs /= static_cast<double>(timed);
  result.marginalMs /= static_cast<double>(timed);
  result.depth /= static_cast<double>(timed);
  result.errorM /= static_cast<double>(timed);
  result.estimate = smoother.calculateEstimate();
  result.variables = result.estimate.size();
  return result;
}

double maxPositionDifference(const Values& a, const Values& b) {
  double maxDiff = 0.0;
  for (const auto& key_value : a) {
    if (Symbol(key_value.key).chr() != 'x' || !b.exists(key_value.key)) {
      continue;
    }
    const Point3 pa = a.at<Pose3>(key_value.key).translation();
    const Point3 pb = b.at<Pose3>(key_value.key).translation();
    maxDiff = std::max(maxDiff, (pa - pb).norm());
  }
  return maxDiff;
}

}  // namespace

int main(int argc, char** argv) {
  const size_t steps = argc > 1 ? std::stoul(argv[1]) : 1000;
  std::vector<double> lags;
  std::stringstream lagList(argc > 2 ? argv[2] : "2,6,12");
  for (std::string item; std::getline(lagList, item, ',');) {
    lags.push_back(std::stod(item));
  }

  std::printf(
      "| problem | lag s | variables | update ms | marginal ms | newest depth "
      "| mean error m | max diff m |\n");
  std::printf("|---|---:|---:|---|---|---|---|---:|\n");
  for (Problem problem : {Problem::Odom, Problem::Imu, Problem::Frames}) {
    for (double lag : lags) {
      const Result off = run(problem, lag, false, steps);
      const Result on = run(problem, lag, true, steps);
      std::printf(
          "| %s | %.0f | %zu | %.3f -> %.3f (%.2fx) | %.3f -> %.3f (%.1fx) | "
          "%.1f -> %.1f | %.4f / %.4f | %.1e |\n",
          name(problem), lag, off.variables, off.updateMs, on.updateMs,
          off.updateMs / on.updateMs, off.marginalMs, on.marginalMs,
          off.marginalMs / on.marginalMs, off.depth, on.depth, off.errorM,
          on.errorM, maxPositionDifference(off.estimate, on.estimate));
      std::fflush(stdout);
    }
  }
  return 0;
}
