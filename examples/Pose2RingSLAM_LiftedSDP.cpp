/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2RingSLAM_LiftedSDP.cpp
 * @brief A Pose2 ring SLAM example solved with monolithic and chordal SDPs.
 */

#include <gtsam/certifiable/LiftedSDPProblem.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Pose2.h>
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

template <typename T, typename SdpProblem>
bool solveAndReport(const std::string& name, SdpProblem* sdp,
                    const std::vector<T>& gt,
                    const std::map<std::string, double>& mosekParams) {
  constexpr double kRankOneEigenRatioThreshold = 1e5;

  if (!sdp->solve(mosekParams)) {
    std::cerr << name
              << " MOSEK solve did not produce a readable primal solution."
              << std::endl;
    return false;
  }

  const auto recoveredPoses = ExtractQcqpValues<T, 1>(sdp->qcqpValues());
  if (recoveredPoses.size() != gt.size()) {
    std::cerr << "Recovered QCQP value count does not match ground truth."
              << std::endl;
    return false;
  }

  double totalPoseErrorNorm = 0.0;
  for (size_t i = 0; i < recoveredPoses.size(); ++i) {
    totalPoseErrorNorm +=
        gt[i].localCoordinates(recoveredPoses[i].second).norm();
  }
  const double averagePoseErrorNorm =
      totalPoseErrorNorm / static_cast<double>(recoveredPoses.size());

  bool allRankOne = true;
  for (double evr : sdp->variableEVRs()) {
    if (evr < kRankOneEigenRatioThreshold) {
      allRankOne = false;
      break;
    }
  }

  std::cout << "\n=== " << name << " SDP ===" << std::endl;
  std::cout << name << " objective: " << sdp->objectiveValue() << std::endl;
  std::cout << name << " average pose error norm: " << averagePoseErrorNorm
            << std::endl;
  std::cout << name
            << " all recovered blocks rank-1: " << (allRankOne ? "yes" : "no")
            << std::endl;
  for (size_t i = 0; i < gt.size(); ++i) {
    gt[i].print(name + " ground-truth pose " + std::to_string(i));
    recoveredPoses[i].second.print(name + " recovered pose " +
                                   std::to_string(i));
  }
  return true;
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
  constexpr double kMaxOptimizerTimeSeconds = 1500.0;
  constexpr size_t kPose2FrobeniusDimension = 9;

  NonlinearFactorGraph graph;

  const double yawStep = 2.0 * M_PI / static_cast<double>(N);
  const double translationStep = 2.0;
  const Pose2 initialPose;
  const Pose2 step(translationStep, 0.0, yawStep);
  const std::vector<Pose2> gt = createPoses(initialPose, N, step);

  auto odometryNoiseModel =
      noiseModel::Isotropic::Sigma(Pose2::dimension, kOdometryIsotropicNoise);
  auto sampledNoiseModel =
      noiseModel::Isotropic::Sigma(Pose2::dimension, kSampledOdometryNoise);
  Sampler odometrySampler(sampledNoiseModel, kNoiseSeed);

  auto exactPriorNoiseModel =
      noiseModel::Constrained::All(kPose2FrobeniusDimension);
  graph.emplace_shared<FrobeniusPrior<Pose2>>(0, gt[0].matrix(),
                                              exactPriorNoiseModel);

  for (size_t i = 0; i < N; ++i) {
    const size_t j = (i + 1) % N;
    const Pose2 T_ij = gt[i].between(gt[j]);
    const Pose2 T_ij_noisy = T_ij.retract(odometrySampler.sample());
    graph.emplace_shared<FrobeniusBetweenFactor<Pose2>>(i, j, T_ij_noisy,
                                                        odometryNoiseModel);
  }

  const std::map<std::string, double> mosekParams{
      {"intpntCoTolRelGap", 1e-10},
      {"optimizerMaxTime", kMaxOptimizerTimeSeconds},
  };

  const QcqpProblem problem(graph);
  MosekMonolithicSDP monolithicSdp(problem);
  if (!solveAndReport("Monolithic", &monolithicSdp, gt, mosekParams)) {
    return 1;
  }

  MosekChordalSDP chordalSdp(problem, ChordalOrderingType::Metis);
  if (!solveAndReport("Chordal", &chordalSdp, gt, mosekParams)) {
    return 1;
  }

  return 0;
}
