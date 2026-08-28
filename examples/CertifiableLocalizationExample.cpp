/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CertifiableLocalizationExample.cpp
 * @brief Compare local and certifiable solutions to landmark localization.
 * @author Avinash Subramanian
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */

#include <gtsam/certifiable/LiftedSDPProblem.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "CertifiableLocalizationUtils.h"

using namespace gtsam;
using namespace gtsam::examples;

#ifdef GTSAM_USE_MOSEK
namespace {

constexpr double kRankOneEigenRatioThreshold = 1e5;

template <class SdpProblem>
bool solveAndReportSdp(const std::string& name, SdpProblem& sdp,
                       const std::map<std::string, double>& mosekParams,
                       const CertifiableLocalizationProblem& problem) {
  if (!sdp.solve(mosekParams)) {
    std::cerr << name << " SDP did not produce a readable primal solution."
              << std::endl;
    return false;
  }

  const auto keyedRecovered_kTws =
      ExtractQcqpValues<Pose3, 1>(sdp.qcqpValues());
  const std::vector<double> recoveredPoseEVRs = sdp.variableEVRs();
  if (keyedRecovered_kTws.size() != recoveredPoseEVRs.size()) {
    std::cerr << name << " recovered pose and eigenvalue-ratio counts differ."
              << std::endl;
    return false;
  }

  Values recoveredValues_kTws;
  std::vector<Pose3> recovered_kTws, orderedGroundTruth_kTws;
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

  std::cout << name << " SDP objective: " << sdp.objectiveValue() << '\n'
            << name << " recovered feasible graph error: "
            << problem.graph.error(recoveredValues_kTws) << '\n'
            << name << " recovered mean pose error: "
            << recoveredPoseErrors.meanPoseError << '\n'
            << name << " recovered maximum pose error: "
            << recoveredPoseErrors.maximumPoseError << '\n'
            << name
            << " recovered blocks all rank one: " << (allRankOne ? "yes" : "no")
            << std::endl;
  return true;
}

}  // namespace
#endif

int main() {
  const auto problem = loadCertifiableLocalization(
      findExampleDataFile("known_landmark_localization_20.g2o"));

  GaussNewtonParams parameters;
  parameters.setMaxIterations(100);
  parameters.setRelativeErrorTol(1e-12);
  const Values localResult_kTws =
      GaussNewtonOptimizer(problem.graph, problem.initial_kTws, parameters)
          .optimize();

  const double groundTruthError = problem.graph.error(problem.groundTruth_kTws);
  const double initialError = problem.graph.error(problem.initial_kTws);
  const double finalError = problem.graph.error(localResult_kTws);
  const auto poseErrors = poseErrorStatistics(
      problem.groundTruth_kTws, localResult_kTws, problem.poseKeys);

  std::cout << "Ground-truth graph error: " << groundTruthError << '\n'
            << "Initial graph error: " << initialError << '\n'
            << "Gauss-Newton graph error: " << finalError << '\n'
            << "Gauss-Newton mean pose error: " << poseErrors.meanPoseError
            << '\n'
            << "Gauss-Newton maximum pose error: "
            << poseErrors.maximumPoseError << std::endl;

  constexpr double kMaximumPoseError = 0.05;
  if (finalError >= initialError || finalError >= groundTruthError ||
      poseErrors.maximumPoseError >= kMaximumPoseError) {
    std::cerr << "Certifiable localization did not improve the noisy "
                 "problem as expected."
              << std::endl;
    return 1;
  }

#ifdef GTSAM_USE_MOSEK
  constexpr double kMaxOptimizerTimeSeconds = 600.0;
  const std::map<std::string, double> mosekParams{
      {"intpntCoTolRelGap", 1e-10},
      {"optimizerMaxTime", kMaxOptimizerTimeSeconds},
  };

  const QcqpProblem qcqp(problem.graph, 1);
  LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> monolithicSdp(qcqp);
  if (!solveAndReportSdp("Monolithic", monolithicSdp, mosekParams, problem)) {
    return 1;
  }

  LiftedSDPProblem<ChordalSDP, MosekSDPSolver> chordalSdp(
      qcqp, ChordalOrderingType::Metis);
  if (!solveAndReportSdp("Chordal", chordalSdp, mosekParams, problem)) {
    return 1;
  }
#else
  std::cout << "GTSAM_USE_MOSEK is disabled; skipping the SDP comparisons."
            << std::endl;
#endif

  return 0;
}
