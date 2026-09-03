/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CertifiableMosek.h
 * @brief Shared MOSEK helpers for the certifiable application examples.
 */

#pragma once

#include <gtsam/certifiable/LiftedSDPProblem.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Rot2.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace gtsam::examples {

enum class CertifiableSolver {
  Staircase,
  MosekMonolithic,
  MosekChordal,
};

inline const char* certifiableSolverName(CertifiableSolver solver) {
  switch (solver) {
    case CertifiableSolver::Staircase:
      return "staircase";
    case CertifiableSolver::MosekMonolithic:
      return "mosek-monolithic";
    case CertifiableSolver::MosekChordal:
      return "mosek-chordal";
  }
  throw std::invalid_argument("Unknown certifiable solver");
}

inline CertifiableSolver parseCertifiableSolver(const std::string& name) {
  if (name == "staircase") return CertifiableSolver::Staircase;
  if (name == "mosek-monolithic") {
    return CertifiableSolver::MosekMonolithic;
  }
  if (name == "mosek-chordal") return CertifiableSolver::MosekChordal;
  throw std::invalid_argument("Unknown solver: " + name);
}

/** Fix one rotation and point to remove the global SE(d) gauge from an SDP. */
template <typename Rotation, typename Point>
void addPoseGauge(Key rotationKey, Key pointKey, QcqpProblem* problem) {
  if (!problem) {
    throw std::invalid_argument("addPoseGauge: problem is null.");
  }

  constexpr int kRotationDim = traits<Rotation>::QcqpVectorDim;
  constexpr int kPointDim = traits<Point>::QcqpVectorDim;
  constexpr int kDimension = Point::RowsAtCompileTime;

  Matrix rotationSelector;
  Vector identityVector;
  if constexpr (std::is_same_v<Rotation, Rot2>) {
    rotationSelector = Matrix::Zero(2, kRotationDim);
    rotationSelector.block<2, 2>(0, 1).setIdentity();
    identityVector = Vector2(1.0, 0.0);
  } else {
    rotationSelector = Matrix::Zero(kDimension * kDimension, kRotationDim);
    rotationSelector
        .block(0, 1, kDimension * kDimension, kDimension * kDimension)
        .setIdentity();
    const Matrix identity = Matrix::Identity(kDimension, kDimension);
    identityVector =
        Eigen::Map<const Vector>(identity.data(), kDimension * kDimension);
  }
  problem->addConstraint(LinearConstraint::Equal(
      JacobianFactor(rotationKey, rotationSelector, identityVector)));

  Matrix pointSelector = Matrix::Zero(kDimension, kPointDim);
  pointSelector.block(0, 1, kDimension, kDimension).setIdentity();
  problem->addConstraint(LinearConstraint::Equal(
      JacobianFactor(pointKey, pointSelector, Vector::Zero(kDimension))));
}

struct MosekExampleResult {
  bool solved = false;
  std::string status;
  std::string recoveryError;
  double objective = 0.0;
  double solveTimeSeconds = 0.0;
  double wallTimeSeconds = 0.0;
  Values qcqpValues;
  KeyVector orderedKeys;
  std::vector<double> variableEVRs;
};

inline void printMosekSummary(const MosekExampleResult& result,
                              CertifiableSolver solver,
                              double roundedObjective) {
  std::vector<double> sortedEVRs = result.variableEVRs;
  std::sort(sortedEVRs.begin(), sortedEVRs.end());
  const double minimumEVR = sortedEVRs.empty() ? 0.0 : sortedEVRs.front();
  const double medianEVR =
      sortedEVRs.empty() ? 0.0 : sortedEVRs[sortedEVRs.size() / 2];
  const double maximumEVR = sortedEVRs.empty() ? 0.0 : sortedEVRs.back();
  const size_t rankOneBlocks = static_cast<size_t>(std::count_if(
      sortedEVRs.begin(), sortedEVRs.end(),
      [](double evr) { return evr >= 1e6 || (std::isinf(evr) && evr > 0.0); }));
  const double relativeGap = (roundedObjective - result.objective) /
                             std::max(1.0, std::abs(roundedObjective));

  std::cout << "\n========== MOSEK D=1 Result ==========\n"
            << "Solver:            " << certifiableSolverName(solver) << "\n"
            << "Solved:            " << (result.solved ? "yes" : "no") << "\n"
            << "Status:            " << result.status << "\n"
            << "Recovery error:    "
            << (result.recoveryError.empty() ? "none" : result.recoveryError)
            << "\n"
            << "Relaxed objective: " << result.objective << "\n"
            << "Rounded objective: " << roundedObjective << "\n"
            << "Relative gap:      " << relativeGap << "\n"
            << "Variable blocks:   " << sortedEVRs.size() << "\n"
            << "Rank-one blocks:   " << rankOneBlocks << "\n"
            << "Minimum EVR:       " << minimumEVR << "\n"
            << "Median EVR:        " << medianEVR << "\n"
            << "Maximum EVR:       " << maximumEVR << "\n"
            << "MOSEK solve time:  " << result.solveTimeSeconds << " s\n"
            << "SDP wall time:     " << result.wallTimeSeconds << " s\n";
}

#ifdef GTSAM_USE_MOSEK
template <typename Solver>
MosekExampleResult solveMosek(Solver* solver) {
  if (!solver) throw std::invalid_argument("solveMosek: solver is null.");
  constexpr double kMaximumSolveTimeSeconds = 1500.0;
  const std::map<std::string, double> params{
      {"intpntCoTolRelGap", 1e-10},
      {"optimizerMaxTime", kMaximumSolveTimeSeconds},
  };

  MosekExampleResult result;
  result.solved = solver->solve(params);
  result.status = solver->problemStatus();
  result.objective = solver->objectiveValue();
  result.solveTimeSeconds = solver->solveTimeSeconds();
  result.orderedKeys = solver->orderedKeys();
  try {
    result.qcqpValues = solver->qcqpValues();
    result.variableEVRs = solver->variableEVRs();
  } catch (const std::runtime_error& error) {
    result.solved = false;
    result.recoveryError = error.what();
  }
  return result;
}

inline MosekExampleResult solveMosek(const QcqpProblem& problem,
                                     CertifiableSolver solver) {
  const auto start = std::chrono::steady_clock::now();
  MosekExampleResult result;
  if (solver == CertifiableSolver::MosekMonolithic) {
    MosekMonolithicSDP sdp(problem);
    result = solveMosek(&sdp);
  } else if (solver == CertifiableSolver::MosekChordal) {
    MosekChordalSDP sdp(problem, ChordalOrderingType::Metis);
    result = solveMosek(&sdp);
  } else {
    throw std::invalid_argument("solveMosek requires a MOSEK solver mode.");
  }
  result.wallTimeSeconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  return result;
}
#else
inline MosekExampleResult solveMosek(const QcqpProblem&, CertifiableSolver) {
  MosekExampleResult result;
  result.status = "MOSEK support unavailable";
  result.recoveryError =
      "This GTSAM build does not include the MOSEK SDP backend.";
  return result;
}
#endif

}  // namespace gtsam::examples
