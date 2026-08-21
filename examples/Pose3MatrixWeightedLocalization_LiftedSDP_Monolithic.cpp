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
#include <gtsam/slam/KnownLandmarkFactor.h>

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

  // Each point measurement has a ray-aligned covariance ellipsoid. Its two
  // equal lateral standard deviations are sigma_l, and its radial standard
  // deviation is sigma_r. We parameterize their ratio by
  //
  //   rho = anisotropicity = sigma_r / sigma_l,
  //
  // so sigma_r = rho*sigma_l. The covariance eigenvalues are therefore
  // sigma_l^2, sigma_l^2, and sigma_r^2, and
  //
  //   cond(Sigma) = sigma_r^2 / sigma_l^2 = rho^2.
  //
  // Thus kAnisotropicity is the axis ratio of the one-standard-deviation
  // ellipsoid, equivalently sqrt(cond(Sigma)). A value of 10 means that noise
  // along the observation ray has ten times the standard deviation, and one
  // hundred times the variance, of noise perpendicular to the ray.
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

  // Let u be a unit observation ray. The orthogonal projectors onto the radial
  // and lateral subspaces are P_r = uu^T and P_l = I-uu^T. Hence
  //
  //   Sigma = sigma_l^2 P_l + sigma_r^2 P_r
  //         = sigma_l^2 I + (sigma_r^2-sigma_l^2) uu^T.
  //
  // Since P_l and P_r are orthogonal projectors, Sigma is inverted by
  // inverting its eigenvalues:
  //
  //   W = Sigma^{-1}
  //     = (1/sigma_l^2) P_l + (1/sigma_r^2) P_r
  //     = lateralPrecision I
  //       + (radialPrecision-lateralPrecision) uu^T.
  //
  // This is the information matrix supplied to each point factor. Because
  // sigma_r > sigma_l here, radialPrecision < lateralPrecision: radial errors
  // are penalized less strongly than lateral errors.
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
      // To sample that same covariance, let z ~ N(0,I) and use
      //
      //   L = sigma_l P_l + sigma_r P_r
      //     = sigma_l I + (sigma_r-sigma_l) uu^T,
      //   epsilon = Lz.
      //
      // Orthogonality and idempotence of P_l and P_r give
      //
      //   Cov(epsilon) = LL^T
      //                = sigma_l^2 P_l + sigma_r^2 P_r
      //                = Sigma.
      //
      // Therefore the sampled measurement noise and the information matrix
      // used by the factor are exactly consistent.
      const Vector3 pointNoise =
          kLateralSigma * standardSample +
          (radialSigma - kLateralSigma) * ray * ray.dot(standardSample);
      const Point3 measurement = exactMeasurement + pointNoise;

      graph.emplace_shared<KnownLandmarkFactor<Pose3>>(
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
