/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CertifiablePoseGraphOptimization.cpp
 * @brief   Certifiable SE(d) synchronization, that is pose-graph
 *          optimization, via the Riemannian Staircase (Burer-Monteiro
 *          low-rank SDP + sparse-eigensolver certificate). Minimizes
 *          Sum_ij kappa_ij ||R_j - R_i R_ij||^2_F + tau_ij ||t_j - t_i - R_i t_ij||^2
 *          over R_i in O(d), t_i in R^d.
 * @author  Zhexin Xu
 *
 * Usage:
 *   ./CertifiablePoseGraphOptimization --data=<g2o file> --dim=<2|3>
 *       [--initialization=<fast-sync|g2o|random>]
 */

#include <gtsam/certifiable/RiemannianStaircaseOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FastSync.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/RelativeTranslationFactor.h>
#include <gtsam/slam/dataset.h>

#include <Eigen/SVD>

#include <chrono>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>

using namespace gtsam;
using RSO = RiemannianStaircaseOptimizer;

namespace {

constexpr unsigned int kSeed = 60;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTranslationInitSigma = 1.0;

enum class InitializationMethod { FastSync, G2o, Random };

const char* initializationName(InitializationMethod method) {
  switch (method) {
    case InitializationMethod::FastSync:
      return "fast-sync";
    case InitializationMethod::G2o:
      return "g2o";
    case InitializationMethod::Random:
      return "random";
  }
  throw std::invalid_argument("Unknown initialization method");
}

/// Rotation and translation are separate QCQP variables here, so each pose
/// index maps to two distinct keys rather than one.
Key rotationKey(Key poseIndex) { return Symbol('R', poseIndex); }
Key translationKey(Key poseIndex) { return Symbol('t', poseIndex); }

/// Certifiable methods optimize over O(d), which has two components, det = +1
/// and det = -1, so the rounded rotations can come out reflected.
template <int d>
void alignBlockDetSigns(Values& qcqpValues, const std::set<Key>& poseIndices) {
  size_t numNeg = 0, numBlocks = 0;
  for (Key poseIndex : poseIndices) {
    const Matrix M = qcqpValues.at<Matrix>(rotationKey(poseIndex));
    ++numBlocks;
    if (M.determinant() < 0) ++numNeg;
  }
  if (numBlocks == 0 || numNeg <= numBlocks / 2) return;
  for (const auto& [key, M] : qcqpValues.extract<Matrix>()) {
    Matrix flipped = M;
    flipped.col(M.cols() - 1) *= -1.0;
    qcqpValues.update(key, flipped);
  }
}

/// Initial values taken from the dataset's own vertex poses.
template <int d>
Values g2oInitial(const Values& poses, const std::set<Key>& poseIndices) {
  using PoseT = typename std::conditional<d == 2, Pose2, Pose3>::type;
  using RotT = typename std::conditional<d == 2, Rot2, Rot3>::type;
  using Point = Eigen::Matrix<double, d, 1>;
  Values initial;
  for (Key poseIndex : poseIndices) {
    if (!poses.exists(poseIndex)) {
      throw std::invalid_argument("initialization is missing a graph key");
    }
    const PoseT& pose = poses.at<PoseT>(poseIndex);
    InsertQcqpValue<RotT, d>(rotationKey(poseIndex), pose.rotation(), &initial);
    const Point t(pose.translation());
    initial.insert(translationKey(poseIndex), Matrix(t.transpose()));
  }
  return initial;
}

/// Reassemble PoseT from the rounded rank-d blocks.
template <int d>
Values recoverPoses(const Values& atRankD, const std::set<Key>& poseIndices) {
  using PoseT = typename std::conditional<d == 2, Pose2, Pose3>::type;
  using RotT = typename std::conditional<d == 2, Rot2, Rot3>::type;
  using Point = Eigen::Matrix<double, d, 1>;
  Values poses;
  for (Key poseIndex : poseIndices) {
    const RotT rotation = traits<RotT>::template FromQcqpValue<d>(
        atRankD.at<Matrix>(rotationKey(poseIndex)));
    const Point t(
        atRankD.at<Matrix>(translationKey(poseIndex)).row(0).transpose());
    poses.insert(poseIndex, PoseT(rotation, t));
  }
  return poses;
}

/// Random translation init: row i is a Gaussian-sampled `1 x d` row.
template <int d>
Values randomTranslationInitial(const std::set<Key>& poseIndices) {
  Values values;
  std::mt19937 rng(kSeed + 1);
  std::normal_distribution<double> gaussian(0.0, kTranslationInitSigma);
  for (Key poseIndex : poseIndices) {
    Matrix row = Matrix::Zero(1, d);
    for (int c = 0; c < d; ++c) row(0, c) = gaussian(rng);
    values.insert(translationKey(poseIndex), row);
  }
  return values;
}

template <int d>
Values randomInitial(const std::set<Key>& poseIndices) {
  using RotT = typename std::conditional<d == 2, Rot2, Rot3>::type;
  Values values;
  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> uniform(-kPi, kPi);
  for (Key poseIndex : poseIndices) {
    if constexpr (std::is_same_v<RotT, Rot2>) {
      InsertQcqpValue<Rot2, d>(
          rotationKey(poseIndex), Rot2::fromAngle(uniform(rng)), &values);
    } else {
      const Vector3 rpy(uniform(rng), uniform(rng), uniform(rng));
      InsertQcqpValue<Rot3, d>(rotationKey(poseIndex),
                               Rot3::RzRyRx(rpy(0), rpy(1), rpy(2)), &values);
    }
  }
  const Values translationInitial = randomTranslationInitial<d>(poseIndices);
  for (const auto& [key, row] : translationInitial.extract<Matrix>()) {
    values.insert(key, row);
  }
  return values;
}

template <int d>
int runCertifiablePGO(const std::string& dataPath,
                      InitializationMethod initializationMethod) {
  using PoseT = typename std::conditional<d == 2, Pose2, Pose3>::type;
  using RotT = typename std::conditional<d == 2, Rot2, Rot3>::type;
  constexpr bool kIs3D = (d == 3);
  using Point = Eigen::Matrix<double, d, 1>;

  // Build graph
  auto [poseGraph, g2oValues] = readG2o(dataPath, /*is3D=*/kIs3D);
  NonlinearFactorGraph graph;
  std::set<Key> poseIndices;
  size_t skipped = 0;
  for (const auto& f : *poseGraph) {
    const auto bf = std::dynamic_pointer_cast<BetweenFactor<PoseT>>(f);
    if (!bf) {
      ++skipped;
      continue;
    }

    const auto gaussian =
        std::dynamic_pointer_cast<noiseModel::Gaussian>(bf->noiseModel());

    // kappa: rotational precision. tau: translational precision.
    // Both are reduced to isotropic precisions.
    double kappa, tau;
    if constexpr (kIs3D) {
      const Matrix6 info =
          gaussian ? gaussian->information() : Matrix6::Identity();
      kappa = 3.0 / (2.0 * info.topLeftCorner<3, 3>().inverse().trace());
      tau = 3.0 / info.bottomRightCorner<3, 3>().inverse().trace();
    } else {
      const Matrix3 info =
          gaussian ? gaussian->information() : Matrix3::Identity();
      kappa = info(2, 2);
      tau = 2.0 / info.topLeftCorner<2, 2>().inverse().trace();
    }

    auto rotNoise =
        noiseModel::Isotropic::Variance(traits<RotT>::dimension, 1.0 / kappa);
    const Key ri = rotationKey(bf->key1()), rj = rotationKey(bf->key2());
    const Key ti = translationKey(bf->key1()), tj = translationKey(bf->key2());
    graph.emplace_shared<FrobeniusBetweenFactor<RotT>>(
        ri, rj, bf->measured().rotation(), rotNoise);
    graph.emplace_shared<RelativeTranslationFactor<d>>(
        ri, ti, tj, Point(bf->measured().translation()), tau);
    poseIndices.insert(bf->key1());
    poseIndices.insert(bf->key2());
  }

  if (graph.empty()) {
    std::cerr << "No BetweenFactor<" << (kIs3D ? "Pose3" : "Pose2")
              << "> found in " << dataPath
              << ". Wrong --dim, or wrong file format?\n";
    return 1;
  }
  std::cout << "Loaded " << poseIndices.size() << " poses, " << graph.size() / 2
            << " edges from " << dataPath << " (d=" << d << ")";
  if (skipped > 0) std::cout << "  (" << skipped << " non-Between skipped)";
  std::cout << "\n\n";

  RiemannianStaircaseParams params;
  params.pMin = d;
  params.pMax = 10;
  params.eta = 1e-3;
  params.verbose = true;
  params.almParams->maxIterations = 50;
  params.almParams->absoluteViolationTolerance = 1e-8;
  params.almParams->lmParams.maxIterations = 300;
  params.almParams->verbose = false;

  // Initialization
  const auto initializationStart = std::chrono::steady_clock::now();
  Values initial;
  switch (initializationMethod) {
    case InitializationMethod::FastSync: {
      // fastSync needs isotropic noise, which only the graph's reconstructed
      // rotation-only edges have (a real Pose's noise is not isotropic);
      // translation still needs a separate initial guess.
      initial = Values();
      const Values rotations = fastSync<RotT>(graph);
      for (const auto& [key, rotation] : rotations.extract<RotT>()) {
        InsertQcqpValue<RotT, d>(key, rotation, &initial);
      }
      const Values translationInitial =
          randomTranslationInitial<d>(poseIndices);
      for (const auto& [key, row] : translationInitial.extract<Matrix>()) {
        initial.insert(key, row);
      }
      break;
    }
    case InitializationMethod::G2o:
      initial = g2oInitial<d>(*g2oValues, poseIndices);
      break;
    case InitializationMethod::Random:
      initial = randomInitial<d>(poseIndices);
      break;
  }
  const double initializationSeconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    initializationStart).count();

  // Certifiable solver
  const auto solveStart = std::chrono::steady_clock::now();
  const auto result = RSO(graph, initial, params).optimize();
  const double solveSeconds = std::chrono::duration<double>(
                                  std::chrono::steady_clock::now() - solveStart)
                                  .count();

  // Rounding and extracting solution
  const auto roundingStart = std::chrono::steady_clock::now();
  Values rounded, poses;
  if (result.rounded) {
    Values atRankD = result.layout.unstack(result.rounded->Yd);
    alignBlockDetSigns<d>(atRankD, poseIndices);
    poses = recoverPoses<d>(atRankD, poseIndices);
    for (auto& [key, R] : ExtractQcqpValues<RotT, d>(atRankD)) {
      rounded.insert(key, R);
    }
    for (Key poseIndex : poseIndices) {
      const Matrix row = atRankD.at<Matrix>(translationKey(poseIndex));
      rounded.insert(translationKey(poseIndex), Point(row.row(0).transpose()));
    }
  }

  const double roundingSeconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    roundingStart).count();

  std::cout << "\n========== Result ==========\n"
            << "Certified:         " << (result.certified ? "yes" : "no")
            << "\n"
            << "Final rank:        " << result.finalRank << "\n";
  if (rounded.empty()) {
    std::cout << "Rounded objective: n/a\n";
  } else {
    std::cout << "Rounded objective: " << graph.error(rounded) << "\n";
  }
  const double buildSeconds =
      std::accumulate(result.qcqpBuildTimePerLevel.begin(),
                      result.qcqpBuildTimePerLevel.end(), 0.0);
  const double nlpSeconds = std::accumulate(result.nlpTimePerLevel.begin(),
                                            result.nlpTimePerLevel.end(), 0.0);
  const double verifySeconds = std::accumulate(
      result.verifyTimePerLevel.begin(), result.verifyTimePerLevel.end(), 0.0);
  std::cout << "Initialization:    " << initializationName(initializationMethod)
            << "\n"
            << "Initialization time: " << initializationSeconds << " s\n"
            << "QCQP build time:    " << buildSeconds << " s\n"
            << "Local solve time:  " << nlpSeconds << " s\n"
            << "Certificate time:  " << verifySeconds << " s\n"
            << "Rounding time:     " << roundingSeconds << " s\n"
            << "Total BM time:     " << result.totalTime << " s\n"
            << "Solve wall time:   " << solveSeconds << " s\n"
            << "End-to-end time:   " << initializationSeconds + solveSeconds
            << " s\n";

  return result.certified ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  std::string dataPath;
  std::string initialization = "fast-sync";
  int dim = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg.rfind("--data=", 0) == 0) {
      dataPath = arg.substr(7);
    } else if (arg.rfind("--dim=", 0) == 0) {
      dim = std::stoi(arg.substr(6));
    } else if (arg.rfind("--initialization=", 0) == 0) {
      initialization = arg.substr(17);
    }
  }
  InitializationMethod initializationMethod;
  if (initialization == "fast-sync") {
    initializationMethod = InitializationMethod::FastSync;
  } else if (initialization == "g2o") {
    initializationMethod = InitializationMethod::G2o;
  } else if (initialization == "random") {
    initializationMethod = InitializationMethod::Random;
  } else {
    std::cerr << "Unknown initialization method: " << initialization << "\n";
    return 2;
  }
  if (dataPath.empty() || (dim != 2 && dim != 3)) {
    std::cerr << "Usage: " << argv[0]
              << " --data=<g2o file> --dim=<2|3> "
                 "[--initialization=<fast-sync|g2o|random>]\n";
    return 2;
  }

  if (dim == 3) {
    return runCertifiablePGO<3>(dataPath, initializationMethod);
  } else {
    return runCertifiablePGO<2>(dataPath, initializationMethod);
  }
}
