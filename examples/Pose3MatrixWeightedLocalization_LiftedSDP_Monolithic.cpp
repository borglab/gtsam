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
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "MatrixWeightedLocalizationUtils.h"

using namespace gtsam;
using namespace gtsam::examples;

int main() {
  constexpr double kRankOneEigenRatioThreshold = 1e5;
  constexpr double kMaxOptimizerTimeSeconds = 600.0;

  const auto problem = loadMatrixWeightedLocalization(
      findExampleDataFile("matrix_weighted_localization_20.g2o"));

  GaussNewtonParams parameters;
  parameters.setMaxIterations(100);
  parameters.setRelativeErrorTol(1e-12);
  const Values localResult_kTws =
      GaussNewtonOptimizer(problem.graph, problem.initial_kTws, parameters)
          .optimize();

#ifdef GTSAM_USE_MOSEK
  const std::map<std::string, double> mosekParams{
      {"intpntCoTolRelGap", 1e-10},
      {"optimizerMaxTime", kMaxOptimizerTimeSeconds},
  };

  const QcqpProblem qcqp(problem.graph, 1);
  LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> monolithicSdp(qcqp);
  if (!monolithicSdp.solve(mosekParams)) {
    std::cerr << "Monolithic MOSEK solve did not produce a readable primal "
              << "solution." << std::endl;
    return 1;
  }

  const auto keyedRecovered_kTws =
      ExtractQcqpValues<Pose3, 1>(monolithicSdp.qcqpValues());
  const std::vector<double> recoveredPoseEVRs = monolithicSdp.variableEVRs();
  if (keyedRecovered_kTws.size() != recoveredPoseEVRs.size()) {
    std::cerr << "Recovered pose and eigenvalue-ratio counts differ."
              << std::endl;
    return 1;
  }

  Values recoveredValues_kTws;
  std::vector<Pose3> recovered_kTws;
  std::vector<Pose3> orderedGroundTruth_kTws;
  recovered_kTws.reserve(keyedRecovered_kTws.size());
  orderedGroundTruth_kTws.reserve(keyedRecovered_kTws.size());
  for (const auto& [k, recovered_kTw] : keyedRecovered_kTws) {
    recoveredValues_kTws.insert(k, recovered_kTw);
    recovered_kTws.push_back(recovered_kTw);
    orderedGroundTruth_kTws.push_back(problem.groundTruth_kTws.at<Pose3>(k));
  }
  const auto recoveredPoseErrors =
      poseErrorStatistics(orderedGroundTruth_kTws, recovered_kTws);
  const bool allRankOne =
      std::all_of(recoveredPoseEVRs.begin(), recoveredPoseEVRs.end(),
                  [](double eigenRatio) {
                    return eigenRatio >= kRankOneEigenRatioThreshold;
                  });

  std::cout << "Ground-truth graph error: "
            << problem.graph.error(problem.groundTruth_kTws) << '\n'
            << "Initial graph error: "
            << problem.graph.error(problem.initial_kTws) << '\n'
            << "Gauss-Newton graph error: "
            << problem.graph.error(localResult_kTws) << '\n'
            << "Monolithic SDP objective: " << monolithicSdp.objectiveValue()
            << '\n'
            << "Recovered feasible graph error: "
            << problem.graph.error(recoveredValues_kTws) << '\n'
            << "Recovered mean pose error: "
            << recoveredPoseErrors.meanPoseError << '\n'
            << "Recovered maximum pose error: "
            << recoveredPoseErrors.maximumPoseError << '\n'
            << "All recovered blocks rank one: " << (allRankOne ? "yes" : "no")
            << std::endl;
#else
  std::cerr << "This example requires GTSAM_USE_MOSEK." << std::endl;
  return 1;
#endif

  return 0;
}
