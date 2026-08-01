/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose3MatrixWeightedLocalization_LiftedSDP_Monolithic.cpp
 * @brief Matrix-weighted Pose3 localization solved with the monolithic SDP.
 * @author Avinash Subramanian
 */

#include <gtsam/certifiable/LiftedSDPProblem.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/PointCorrespondenceFactor.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace gtsam;

int main() {
  constexpr size_t kNumPoses = 20;
  constexpr double kLateralSigma = 0.01;
  constexpr double kAnisotropicity = 10.0;
  constexpr double kOdometrySigma = 0.1;
  constexpr uint64_t kPointNoiseSeed = 42;
  constexpr uint64_t kOdometryNoiseSeed = 43;
  constexpr double kRankOneEigenRatioThreshold = 1e5;
  constexpr double kMaxOptimizerTimeSeconds = 600.0;

  const Pose3 step(Rot3::RzRyRx(0.03, -0.05, 0.08),
                   Point3(0.3, -0.1, 0.2));
  std::vector<Pose3> groundTruth{Pose3()};
  groundTruth.reserve(kNumPoses);
  for (size_t i = 1; i < kNumPoses; ++i) {
    groundTruth.push_back(groundTruth.back().compose(step));
  }

  const std::vector<Point3> landmarks{
      Point3(4.0, 1.0, 2.0), Point3(-1.0, 3.0, 5.0),
      Point3(2.0, -4.0, 3.0), Point3(5.0, 2.0, -1.0)};

  NonlinearFactorGraph graph;
  Values groundTruthValues;

  const double radialSigma = kAnisotropicity * kLateralSigma;
  const double lateralPrecision = 1.0 / (kLateralSigma * kLateralSigma);
  const double radialPrecision = 1.0 / (radialSigma * radialSigma);
  std::mt19937 pointNoiseGenerator(kPointNoiseSeed);
  std::normal_distribution<double> standardNormal(0.0, 1.0);

  for (size_t k = 0; k < groundTruth.size(); ++k) {
    groundTruthValues.insert(k, groundTruth[k]);
    for (const Point3& landmark : landmarks) {
      const Point3 exactMeasurement = groundTruth[k].transformFrom(landmark);
      const Vector3 ray = exactMeasurement.normalized();
      const Matrix3 information =
          lateralPrecision * Matrix3::Identity() +
          (radialPrecision - lateralPrecision) * ray * ray.transpose();

      Vector3 standardSample;
      standardSample << standardNormal(pointNoiseGenerator),
          standardNormal(pointNoiseGenerator),
          standardNormal(pointNoiseGenerator);
      // This square root samples the same ray-aligned covariance whose inverse
      // is supplied to the point-correspondence factor.
      const Vector3 pointNoise =
          kLateralSigma * standardSample +
          (radialSigma - kLateralSigma) * ray * ray.dot(standardSample);
      const Point3 measurement = exactMeasurement + pointNoise;

      graph.emplace_shared<PointCorrespondenceFactor<Pose3>>(
          k, landmark, measurement,
          noiseModel::Gaussian::Information(information));
    }
  }

  const auto odometryNoiseModel =
      noiseModel::Isotropic::Sigma(Pose3::dimension, kOdometrySigma);
  Sampler odometrySampler(odometryNoiseModel, kOdometryNoiseSeed);
  for (size_t i = 0; i + 1 < groundTruth.size(); ++i) {
    const size_t j = i + 1;
    const Pose3 exactRelative = groundTruth[i].between(groundTruth[j]);
    // Relative measurements are sampled in tangent space, while optimization
    // retains the isotropically weighted Frobenius residual.
    const Pose3 relativeMeasurement =
        exactRelative.retract(odometrySampler.sample());
    graph.emplace_shared<FrobeniusBetweenFactor<Pose3>>(
        i, j, relativeMeasurement, odometryNoiseModel);
  }

  Values initial;
  Vector6 perturbation;
  perturbation << 0.02, -0.015, 0.01, 0.08, -0.05, 0.06;
  for (size_t k = 0; k < groundTruth.size(); ++k) {
    initial.insert(
        k, groundTruth[k].retract(static_cast<double>(k + 1) * perturbation));
  }

  GaussNewtonParams parameters;
  parameters.setMaxIterations(100);
  parameters.setRelativeErrorTol(1e-12);
  const Values localResult =
      GaussNewtonOptimizer(graph, initial, parameters).optimize();

#ifdef GTSAM_USE_MOSEK
  const std::map<std::string, double> mosekParams{
      {"intpntCoTolRelGap", 1e-10},
      {"optimizerMaxTime", kMaxOptimizerTimeSeconds},
  };

  const QcqpProblem problem(graph, 1);
  LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> monolithicSdp(problem);
  if (!monolithicSdp.solve(mosekParams)) {
    std::cerr << "Monolithic MOSEK solve did not produce a readable primal "
              << "solution." << std::endl;
    return 1;
  }

  monolithicSdp.recoverLiftedVectors();
  const std::vector<Pose3> recoveredPoses =
      monolithicSdp.getRecoveredPoses<Pose3>();
  const std::vector<double>& recoveredPoseEVRs =
      monolithicSdp.getRecoveredVariableEVRs();

  const KeyVector& orderedKeys = monolithicSdp.orderedKeys();
  Values recoveredValues;
  std::vector<Pose3> orderedGroundTruth;
  orderedGroundTruth.reserve(orderedKeys.size());
  for (size_t index = 0; index < orderedKeys.size(); ++index) {
    recoveredValues.insert(orderedKeys[index], recoveredPoses[index]);
    orderedGroundTruth.push_back(groundTruth.at(orderedKeys[index]));
  }
  const std::vector<double> recoveredPoseErrorNorms =
      monolithicSdp.getRecoveredPoseErrorNorms(orderedGroundTruth);

  double totalPoseError = 0.0;
  double maximumPoseError = 0.0;
  bool allRankOne = true;
  for (size_t index = 0; index < recoveredPoseErrorNorms.size(); ++index) {
    totalPoseError += recoveredPoseErrorNorms[index];
    maximumPoseError =
        std::max(maximumPoseError, recoveredPoseErrorNorms[index]);
    allRankOne = allRankOne &&
                 recoveredPoseEVRs[index] >= kRankOneEigenRatioThreshold;
  }
  const double meanPoseError =
      totalPoseError / static_cast<double>(recoveredPoseErrorNorms.size());

  std::cout << "Ground-truth graph error: " << graph.error(groundTruthValues)
            << '\n'
            << "Initial graph error: " << graph.error(initial) << '\n'
            << "Gauss-Newton graph error: " << graph.error(localResult) << '\n'
            << "Monolithic SDP objective: " << monolithicSdp.objectiveValue()
            << '\n'
            << "Recovered feasible graph error: "
            << graph.error(recoveredValues) << '\n'
            << "Recovered mean pose error: " << meanPoseError << '\n'
            << "Recovered maximum pose error: " << maximumPoseError << '\n'
            << "All recovered blocks rank one: "
            << (allRankOne ? "yes" : "no") << std::endl;
#else
  std::cerr << "This example requires GTSAM_USE_MOSEK." << std::endl;
  return 1;
#endif

  return 0;
}
