/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Rot2RingQcqpToMonolithicSDP.cpp
 * @brief Minimal Rot2 ring SLAM construction ending at a QcqpProblem.
 */

#include <gtsam/certifiable/LiftedSDPProblem.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using namespace gtsam;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

/**
 * Build the small Rot2 ring used by the QCQP tests. This intentionally uses the
 * existing Frobenius factor conversion path, then stops once QcqpProblem exists.
 */
NonlinearFactorGraph MakeRot2RingGraph(size_t numPoses, double delta) {
  NonlinearFactorGraph graph;
  const auto hardPriorNoise = noiseModel::Constrained::All(4);
  graph.emplace_shared<FrobeniusPrior<Rot2>>(
      Symbol('x', 0), Rot2::Identity().matrix(), hardPriorNoise);

  for (size_t i = 0; i < numPoses; ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        Symbol('x', i), Symbol('x', (i + 1) % numPoses),
        Rot2::fromAngle(delta));
  }
  return graph;
}

/**
 * Create feasible QCQP matrix values for reporting the constructed problem.
 * SDP construction should consume the QcqpProblem itself, not this graph.
 */
[[maybe_unused]] Values MakeRot2RingQcqpValues(size_t numPoses, double delta) {
  Values values;
  for (size_t i = 0; i < numPoses; ++i) {
    InsertQcqpValue<Rot2, 1>(Symbol('x', i), Rot2::fromAngle(i * delta),
                             &values);
  }
  return values;
}

[[maybe_unused]] void PrintKeys(const std::string& label, const KeySet& keys) {
  std::cout << label << ":";
  for (Key key : keys) {
    std::cout << " " << DefaultKeyFormatter(key);
  }
  std::cout << std::endl;
}

std::vector<Rot2> MakeRot2RingGroundTruth(size_t numPoses, double delta) {
  std::vector<Rot2> groundTruth;
  groundTruth.reserve(numPoses);
  for (size_t i = 0; i < numPoses; ++i) {
    groundTruth.push_back(Rot2::fromAngle(i * delta));
  }
  return groundTruth;
}

bool SolveRot2Ring(size_t numPoses) {
  const double delta = 2.0 * kPi / static_cast<double>(numPoses);
  const NonlinearFactorGraph graph = MakeRot2RingGraph(numPoses, delta);
  const QcqpProblem problem(graph);
  [[maybe_unused]] const Values qcqpValues =
      MakeRot2RingQcqpValues(numPoses, delta);
  //std::cout << "equality violation at feasible ring values: "
   //         << problem.eConstraints().violationNorm(qcqpValues) << std::endl;
  LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> sdp(problem);

  // # copied from gtsam-private
  if (!sdp.solve()) {
    std::cerr << "Monolithic MOSEK solve did not produce a readable primal "
              << "solution." << std::endl;
    return false;
  }

  sdp.recoverLiftedVectors();
  const std::vector<Rot2> groundTruth =
      MakeRot2RingGroundTruth(numPoses, delta);
  const auto recoveredPoses = sdp.getRecoveredPoses<Rot2>();
  const auto poseErrors = sdp.getRecoveredPoseErrorNorms(groundTruth);
  const auto& poseEVRs = sdp.getRecoveredVariableEVRs();
  const double averagePoseError =
      std::accumulate(poseErrors.begin(), poseErrors.end(), 0.0) /
      static_cast<double>(poseErrors.size());
  const double maximumPoseError =
      *std::max_element(poseErrors.begin(), poseErrors.end());
  const double minimumEVR = *std::min_element(poseEVRs.begin(), poseEVRs.end());
  const bool allRankOne = std::all_of(
      poseEVRs.begin(), poseEVRs.end(),
      [](double evr) { return evr >= 1e5; });

  std::cout << numPoses << "\t" << sdp.solveTimeSeconds() << "\t"
            << averagePoseError << "\t" << maximumPoseError << "\t"
            << minimumEVR << "\t" << (allRankOne ? "yes" : "no") << "\t"
            << sdp.objectiveValue() << std::endl;

  if (numPoses == 5) {
    std::cout << "pose\texpected angle\trecovered angle\terror" << std::endl;
    for (size_t index = 0; index < numPoses; ++index) {
      std::cout << index << "\t" << groundTruth[index].theta() << "\t"
                << recoveredPoses[index].theta() << "\t" << poseErrors[index]
                << std::endl;
    }
  }
  return true;
}

bool SolveChordalRot2Ring(size_t numPoses) {
  const double delta = 2.0 * kPi / static_cast<double>(numPoses);
  const NonlinearFactorGraph graph = MakeRot2RingGraph(numPoses, delta);
  const QcqpProblem problem(graph);
  LiftedSDPProblem<ChordalSDP, MosekSDPSolver> sdp(
      problem, ChordalOrderingType::Colamd);

  // # modified from gtsam-private
  if (!sdp.solve()) {
    std::cerr << "Chordal MOSEK solve did not produce a readable primal "
              << "solution." << std::endl;
    return false;
  }

  sdp.recoverLiftedVectors();
  const std::vector<Rot2> groundTruth =
      MakeRot2RingGroundTruth(numPoses, delta);
  const auto recoveredPoses = sdp.getRecoveredPoses<Rot2>();
  const auto poseErrors = sdp.getRecoveredPoseErrorNorms(groundTruth);
  const auto& poseEVRs = sdp.getRecoveredVariableEVRs();
  const double averagePoseError =
      std::accumulate(poseErrors.begin(), poseErrors.end(), 0.0) /
      static_cast<double>(poseErrors.size());
  const double maximumPoseError =
      *std::max_element(poseErrors.begin(), poseErrors.end());
  const double minimumEVR = *std::min_element(poseEVRs.begin(), poseEVRs.end());
  const bool allRankOne = std::all_of(
      poseEVRs.begin(), poseEVRs.end(),
      [](double evr) { return evr >= 1e5; });

  std::cout << "Chordal N=" << numPoses
            << "\tsolver time (s)=" << sdp.solveTimeSeconds()
            << "\taverage pose error=" << averagePoseError
            << "\tmaximum pose error=" << maximumPoseError
            << "\tminimum EVR=" << minimumEVR
            << "\trank-1=" << (allRankOne ? "yes" : "no")
            << "\tobjective=" << sdp.objectiveValue() << std::endl;

  std::cout << "pose\texpected angle\trecovered angle\terror" << std::endl;
  for (size_t index = 0; index < numPoses; ++index) {
    std::cout << index << "\t" << groundTruth[index].theta() << "\t"
              << recoveredPoses[index].theta() << "\t" << poseErrors[index]
              << std::endl;
  }

  return allRankOne;
}

}  // namespace

int main() {
  if (!SolveChordalRot2Ring(5)) {
    return 1;
  }

  std::cout << "N\tsolver time (s)\taverage pose error\tmaximum pose error\tminimum EVR\trank-1\tobjective"
            << std::endl;
  for (size_t numPoses : std::vector<size_t>{5, 25, 50, 100}) {
    if (!SolveRot2Ring(numPoses)) {
      return 1;
    }
  }
  return 0;
}
