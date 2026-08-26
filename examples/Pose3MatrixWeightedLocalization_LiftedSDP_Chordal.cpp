/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose3MatrixWeightedLocalization_LiftedSDP_Chordal.cpp
 * @brief Matrix-weighted Pose3 localization solved with the chordal SDP.
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
  constexpr double kLateralSigma = 0.01;
  constexpr double kAnisotropicity = 10.0;
  constexpr double kOdometrySigma = 0.1;
  constexpr uint64_t kPointNoiseSeed = 42;
  constexpr uint64_t kOdometryNoiseSeed = 43;
  constexpr double kRankOneEigenRatioThreshold = 1e5;
  constexpr double kMaxOptimizerTimeSeconds = 600.0;

  const Pose3 kTkp1(Rot3::RzRyRx(0.03, -0.05, 0.08), Point3(0.3, -0.1, 0.2));
  std::vector<Pose3> kTws{Pose3()};
  kTws.reserve(kNumPoses);
  for (size_t i = 1; i < kNumPoses; ++i) {
    kTws.push_back(kTws.back().compose(kTkp1));
  }

  const std::vector<Point3> wLs{Point3(4.0, 1.0, 2.0), Point3(-1.0, 3.0, 5.0),
                                Point3(2.0, -4.0, 3.0), Point3(5.0, 2.0, -1.0)};

  NonlinearFactorGraph graph;
  Values groundTruth;

  const double radialSigma = kAnisotropicity * kLateralSigma;
  const double lateralPrecision = 1.0 / (kLateralSigma * kLateralSigma);
  const double radialPrecision = 1.0 / (radialSigma * radialSigma);
  std::mt19937 pointNoiseGenerator(kPointNoiseSeed);
  std::normal_distribution<double> standardNormal(0.0, 1.0);

  for (size_t k = 0; k < kTws.size(); ++k) {
    groundTruth.insert(k, kTws[k]);
    for (const Point3& wL : wLs) {
      const Point3 exact_kP = kTws[k].transformFrom(wL);
      const Vector3 kRay = exact_kP.normalized();
      const Matrix3 information =
          lateralPrecision * Matrix3::Identity() +
          (radialPrecision - lateralPrecision) * kRay * kRay.transpose();

      Vector3 z;
      z << standardNormal(pointNoiseGenerator),
          standardNormal(pointNoiseGenerator),
          standardNormal(pointNoiseGenerator);
      // This square root samples the same ray-aligned covariance whose inverse
      // is supplied to the known-landmark factor.
      const Vector3 kNoise = kLateralSigma * z +
                             (radialSigma - kLateralSigma) * kRay * kRay.dot(z);
      const Point3 measured_kP = exact_kP + kNoise;

      graph.emplace_shared<KnownLandmarkFactor<Pose3>>(
          k, wL, measured_kP, noiseModel::Gaussian::Information(information));
    }
  }

  const auto odometryNoiseModel =
      noiseModel::Isotropic::Sigma(Pose3::dimension, kOdometrySigma);
  Sampler odometrySampler(odometryNoiseModel, kOdometryNoiseSeed);
  // Poses are sensor-from-world, so iTj = iTw*jTw^{-1} composes on the left.
  for (size_t i = 0; i + 1 < kTws.size(); ++i) {
    const size_t j = i + 1;
    const Pose3 exact_iTj = kTws[i].compose(kTws[j].inverse());
    // Relative measurements are sampled in tangent space, while optimization
    // retains the isotropically weighted Frobenius residual.
    const Pose3 measured_iTj = exact_iTj.retract(odometrySampler.sample());
    graph.emplace_shared<FrobeniusLeftBetweenFactor<Pose3>>(i, j, measured_iTj,
                                                            odometryNoiseModel);
  }

  Values initial;
  Vector6 perturbation;
  perturbation << 0.02, -0.015, 0.01, 0.08, -0.05, 0.06;
  for (size_t k = 0; k < kTws.size(); ++k) {
    initial.insert(k,
                   kTws[k].retract(static_cast<double>(k + 1) * perturbation));
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
  LiftedSDPProblem<ChordalSDP, MosekSDPSolver> chordalSdp(
      problem, ChordalOrderingType::Metis);
  if (!chordalSdp.solve(mosekParams)) {
    std::cerr << "Chordal MOSEK solve did not produce a readable primal "
              << "solution." << std::endl;
    return 1;
  }

  chordalSdp.recoverLiftedVectors();
  const std::vector<Pose3> recovered_kTws =
      chordalSdp.getRecoveredPoses<Pose3>();
  const std::vector<double>& recoveredPoseEVRs =
      chordalSdp.getRecoveredVariableEVRs();

  const KeyVector& orderedKeys = chordalSdp.orderedKeys();
  Values recovered;
  std::vector<Pose3> ordered_kTws;
  ordered_kTws.reserve(orderedKeys.size());
  for (size_t index = 0; index < orderedKeys.size(); ++index) {
    recovered.insert(orderedKeys[index], recovered_kTws[index]);
    ordered_kTws.push_back(kTws.at(orderedKeys[index]));
  }
  const std::vector<double> recoveredPoseErrorNorms =
      chordalSdp.getRecoveredPoseErrorNorms(ordered_kTws);

  double totalPoseError = 0.0;
  double maximumPoseError = 0.0;
  bool allRankOne = true;
  for (size_t index = 0; index < recoveredPoseErrorNorms.size(); ++index) {
    totalPoseError += recoveredPoseErrorNorms[index];
    maximumPoseError =
        std::max(maximumPoseError, recoveredPoseErrorNorms[index]);
    allRankOne =
        allRankOne && recoveredPoseEVRs[index] >= kRankOneEigenRatioThreshold;
  }
  const double meanPoseError =
      totalPoseError / static_cast<double>(recoveredPoseErrorNorms.size());

  std::cout << "Ground-truth graph error: " << graph.error(groundTruth) << '\n'
            << "Initial graph error: " << graph.error(initial) << '\n'
            << "Gauss-Newton graph error: " << graph.error(localResult) << '\n'
            << "Chordal SDP objective: " << chordalSdp.objectiveValue() << '\n'
            << "Recovered feasible graph error: " << graph.error(recovered)
            << '\n'
            << "Recovered mean pose error: " << meanPoseError << '\n'
            << "Recovered maximum pose error: " << maximumPoseError << '\n'
            << "All recovered blocks rank one: " << (allRankOne ? "yes" : "no")
            << std::endl;
#else
  std::cerr << "This example requires GTSAM_USE_MOSEK." << std::endl;
  return 1;
#endif

  return 0;
}
