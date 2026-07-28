/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose3RingSLAM_LiftedSDP_Chordal.cpp
 * @brief A Pose3 ring SLAM example solved with the chordal lifted SDP.
 */

#include <gtsam/certifiable/LiftedSDPProblem.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace gtsam;

template <typename T>
std::vector<T> createPoses(const T& initialPose, size_t N, const T& step) {
  std::vector<T> poses;
  poses.reserve(N);
  poses.push_back(initialPose);
  for (size_t i = 1; i < N; ++i) {
    poses.push_back(poses.back().compose(step));
  }
  return poses;
}

int main(int argc, char** argv) {
  size_t N = 20;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--N") {
      if (i + 1 >= argc) {
        std::cerr << "--N requires a value." << std::endl;
        return 1;
      }
      N = std::stoul(argv[++i]);
      continue;
    }

    std::cerr << "Unknown argument: " << arg << std::endl;
    return 1;
  }

  constexpr uint32_t kNoiseSeed = 42u;
  constexpr double kOdometryIsotropicNoise = 0.01;
  constexpr double kSampledOdometryNoise = 0.01;
  constexpr double kRankOneEigenRatioThreshold = 1e5;
  constexpr double kMaxOptimizerTimeSeconds = 1500.0;
  constexpr size_t kPose3FrobeniusDimension = 16;

  NonlinearFactorGraph graph;

  const double yawStep = 2.0 * M_PI / static_cast<double>(N);
  const double translationStep = 2.0;
  const Pose3 initialPose;
  const Pose3 step(Rot3::Rz(yawStep), Point3(translationStep, 0.0, 0.0));
  const std::vector<Pose3> gt = createPoses(initialPose, N, step);

  auto odometryNoiseModel =
      noiseModel::Isotropic::Sigma(Pose3::dimension, kOdometryIsotropicNoise);
  auto sampledNoiseModel =
      noiseModel::Isotropic::Sigma(Pose3::dimension, kSampledOdometryNoise);
  Sampler odometrySampler(sampledNoiseModel, kNoiseSeed);

  auto exactPriorNoiseModel =
      noiseModel::Constrained::All(kPose3FrobeniusDimension);
  graph.emplace_shared<FrobeniusPrior<Pose3>>(
      0, gt[0].matrix(), exactPriorNoiseModel);

  for (size_t i = 0; i < N; ++i) {
    const size_t j = (i + 1) % N;
    const Pose3 T_ij = gt[i].between(gt[j]);
    const Pose3 T_ij_noisy = T_ij.retract(odometrySampler.sample());
    graph.emplace_shared<FrobeniusBetweenFactor<Pose3>>(
        i, j, T_ij_noisy, odometryNoiseModel);
  }

#ifdef GTSAM_USE_MOSEK
  const std::map<std::string, double> mosekParams{
      {"intpntCoTolRelGap", 1e-10},
      {"optimizerMaxTime", kMaxOptimizerTimeSeconds},
  };

  const QcqpProblem problem(graph);
  LiftedSDPProblem<ChordalSDP, MosekSDPSolver> chordalSdp(
      problem, ChordalOrderingType::Metis);
  if (!chordalSdp.solve(mosekParams)) {
    std::cerr << "Chordal MOSEK solve did not produce a readable primal "
              << "solution." << std::endl;
    return 1;
  }

  chordalSdp.recoverLiftedVectors();
  const std::vector<Pose3> recoveredPoses =
      chordalSdp.getRecoveredPoses<Pose3>();
  const std::vector<double>& recoveredPoseEVRs =
      chordalSdp.getRecoveredVariableEVRs();
  const std::vector<double> recoveredPoseErrorNorms =
      chordalSdp.getRecoveredPoseErrorNorms(gt);

  double totalPoseErrorNorm = 0.0;
  for (double errorNorm : recoveredPoseErrorNorms) {
    totalPoseErrorNorm += errorNorm;
  }
  const double averagePoseErrorNorm =
      totalPoseErrorNorm / static_cast<double>(recoveredPoseErrorNorms.size());

  bool allRankOne = true;
  for (double evr : recoveredPoseEVRs) {
    if (evr < kRankOneEigenRatioThreshold) {
      allRankOne = false;
      break;
    }
  }

  std::cout << "\n=== Chordal SDP ===" << std::endl;
  std::cout << "Chordal objective: " << chordalSdp.objectiveValue()
            << std::endl;
  std::cout << "Chordal average pose error norm: " << averagePoseErrorNorm
            << std::endl;
  std::cout << "Chordal all recovered blocks rank-1: "
            << (allRankOne ? "yes" : "no") << std::endl;
  for (size_t i = 0; i < gt.size(); ++i) {
    gt[i].print("Chordal ground-truth pose " + std::to_string(i));
  }
  for (size_t i = 0; i < recoveredPoses.size(); ++i) {
    recoveredPoses[i].print("Chordal recovered pose " + std::to_string(i));
  }
#else
  std::cerr << "This example requires GTSAM_USE_MOSEK." << std::endl;
  return 1;
#endif

  return 0;
}
