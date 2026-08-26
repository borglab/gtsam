/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CertifiableLandmarkSLAM.cpp
 * @brief   Certifiable landmark SLAM via the Riemannian
 *          Staircase. Minimizes
 *          Sum_ij kappa_ij ||R_j - R_i R_ij||^2_F + tau_ij ||t_j - t_i - R_i t_ij||^2
 *          + Sum_ik nu_ik ||l_k - t_i - R_i v_ik||^2
 *          over R_i in O(d), t_i in R^d, l_k in R^d.
 * @author  Zhexin Xu
 *
 * Usage:
 *   ./CertifiableLandmarkSLAM --data=<g2o file> --dim=<2|3>
 *       [--initialization=<fast-sync|random>]
 */

#include <gtsam/certifiable/RiemannianStaircaseOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FastSync.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/RelativeTranslationFactor.h>

#include <Eigen/SVD>

#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace gtsam;
using RSO = RiemannianStaircaseOptimizer;

namespace {

constexpr unsigned int kSeed = 60;

enum class InitializationMethod { FastSync, Random };

const char* initializationName(InitializationMethod method) {
  switch (method) {
    case InitializationMethod::FastSync:
      return "fast-sync";
    case InitializationMethod::Random:
      return "random";
  }
  throw std::invalid_argument("Unknown initialization method");
}

Key rotationKey(Key poseIndex) { return Symbol('R', poseIndex); }
Key translationKey(Key poseIndex) { return Symbol('t', poseIndex); }
Key landmarkKey(Key landmarkIndex) { return Symbol('l', landmarkIndex); }

/// Certifiable methods optimize over O(d), which has two components, det = +1
/// and det = -1, so the rounded rotations can come out reflected.
template <int d>
void alignBlockDetSigns(Values& values, const std::set<Key>& poseIndices) {
  size_t negative = 0;
  for (Key poseIndex : poseIndices)
    if (values.at<Matrix>(rotationKey(poseIndex)).determinant() < 0) ++negative;
  if (negative <= poseIndices.size() / 2) return;
  for (const auto& [key, M] : values.extract<Matrix>()) {
    Matrix flipped = M;
    flipped.col(M.cols() - 1) *= -1.0;
    values.update(key, flipped);
  }
}

/// One odometry edge.
template <int d>
struct Odometry {
  using PoseT = typename std::conditional<d == 2, Pose2, Pose3>::type;
  Key i, j;
  PoseT measured;
  double kappa, tau;
};

/// One landmark observation.
template <int d>
struct Observation {
  Key i, l;
  Eigen::Matrix<double, d, 1> measured;
  double nu;
};

template <int d>
struct Dataset {
  using PoseT = typename std::conditional<d == 2, Pose2, Pose3>::type;
  std::vector<Odometry<d>> odometry;
  std::vector<Observation<d>> observations;
  std::set<Key> poseIds, landmarkIds;
};

/// Read a landmark-SLAM g2o file.
template <int d>
Dataset<d> readLandmarkG2o(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::invalid_argument("readLandmarkG2o: cannot open " + path);
  Dataset<d> data;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream stream(line);
    std::string tag;
    stream >> tag;

    if ((d == 2 && tag == "EDGE_SE2") ||
        (d == 3 && tag == "EDGE_SE3:QUAT")) {
      Odometry<d> edge;
      stream >> edge.i >> edge.j;
      constexpr int dim = d == 2 ? 3 : 6;
      Matrix information = Matrix::Identity(dim, dim);
      if constexpr (d == 2) {
        double x, y, theta;
        stream >> x >> y >> theta;
        edge.measured = Pose2(x, y, theta);
      } else {
        double x, y, z, qx, qy, qz, qw;
        stream >> x >> y >> z >> qx >> qy >> qz >> qw;
        edge.measured =
            Pose3(Rot3(Quaternion(qw, qx, qy, qz)), Point3(x, y, z));
      }
      for (int r = 0; r < dim; ++r)
        for (int c = r; c < dim; ++c) {
          double v = 0.0;
          if (!(stream >> v)) v = (r == c) ? 1.0 : 0.0;
          information(r, c) = v;
          information(c, r) = v;
        }
      // Isotropic reduction of each block, in the file's
      // (translation, rotation) order.
      if constexpr (d == 2) {
        edge.kappa = information(2, 2);
        edge.tau =
            2.0 / information.topLeftCorner<2, 2>().inverse().trace();
      } else {
        edge.kappa = 3.0 / (2.0 * information.bottomRightCorner<3, 3>()
                                      .inverse().trace());
        edge.tau =
            3.0 / information.topLeftCorner<3, 3>().inverse().trace();
      }
      data.poseIds.insert(edge.i);
      data.poseIds.insert(edge.j);
      data.odometry.push_back(edge);
    } else if (tag == "LANDMARK2") {
      // Planar landmark sightings only; the 3D datasets carry no landmarks.
      if constexpr (d == 2) {
        Observation<d> obs;
        double dx, dy, i11, i12, i22;
        if (!(stream >> obs.i >> obs.l >> dx >> dy >> i11 >> i12 >> i22))
          continue;
        obs.measured = Eigen::Matrix<double, d, 1>(dx, dy);
        Matrix2 information;
        information << i11, i12, i12, i22;
        obs.nu = 2.0 / information.inverse().trace();
        data.landmarkIds.insert(obs.l);
        data.observations.push_back(obs);
      }
    }
  }
  return data;
}

Matrix randomBlock(int rows, int p, std::mt19937& rng, bool orthonormal) {
  std::normal_distribution<double> normal(0.0, 1.0);
  Matrix X(rows, p);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < p; ++c) X(r, c) = normal(rng);
  if (!orthonormal) return X;
  Eigen::JacobiSVD<Matrix> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
  return svd.matrixU() * svd.matrixV().transpose();
}

template <int d>
int runCertifiableLandmarkSLAM(const std::string& path,
                               InitializationMethod initializationMethod) {
  using Point = Eigen::Matrix<double, d, 1>;
  using RotT = typename std::conditional<d == 2, Rot2, Rot3>::type;

  const Dataset<d> data = readLandmarkG2o<d>(path);

  // Build graph
  NonlinearFactorGraph graph;
  for (const auto& edge : data.odometry) {
    const Key ri = rotationKey(edge.i), rj = rotationKey(edge.j);
    graph.emplace_shared<FrobeniusBetweenFactor<RotT>>(
        ri, rj, edge.measured.rotation(),
        noiseModel::Isotropic::Variance(traits<RotT>::dimension,
                                        1.0 / edge.kappa));
    graph.emplace_shared<RelativeTranslationFactor<d>>(
        ri, translationKey(edge.i), translationKey(edge.j),
        Point(edge.measured.translation()), edge.tau);
  }
  for (const auto& obs : data.observations) {
    if (!data.poseIds.count(obs.i)) continue;
    graph.emplace_shared<RelativeTranslationFactor<d>>(
        rotationKey(obs.i), translationKey(obs.i), landmarkKey(obs.l),
        Point(obs.measured), obs.nu);
  }
  const std::set<Key>& poseIndices = data.poseIds;
  const std::set<Key>& landmarkIndices = data.landmarkIds;
  const size_t odometryEdges = data.odometry.size();
  const size_t landmarkEdges = data.observations.size();

  if (graph.empty()) {
    std::cerr << "no BetweenFactor edges in " << path << "\n";
    return 1;
  }

  std::cout << path.substr(path.find_last_of('/') + 1) << "  d=" << d
            << "  odometry edges=" << odometryEdges
            << "  landmark edges=" << landmarkEdges
            << "  poses=" << poseIndices.size()
            << "  landmarks=" << landmarkIndices.size()
            << "  init=" << initializationName(initializationMethod) << "\n";

  const int p = d;
  std::mt19937 rng(kSeed);

  // Initialization
  const auto initializationStart = std::chrono::steady_clock::now();
  Values initial;
  if (initializationMethod == InitializationMethod::FastSync) {
    // fastSync consumes the isotropic rotation edges; translations and
    // landmarks are unconstrained by it and still need a guess.
    NonlinearFactorGraph rotationGraph;
    for (const auto& factor : graph)
      if (std::dynamic_pointer_cast<FrobeniusBetweenFactor<RotT>>(factor))
        rotationGraph.push_back(factor);
    for (const auto& [key, rotation] :
         fastSync<RotT>(rotationGraph).template extract<RotT>()) {
      InsertQcqpValue<RotT, d>(key, rotation, &initial);
    }
    for (Key poseIndex : poseIndices)
      initial.insert(translationKey(poseIndex), randomBlock(1, p, rng, false));
  } else {
    for (Key poseIndex : poseIndices) {
      initial.insert(rotationKey(poseIndex), randomBlock(d, p, rng, true));
      initial.insert(translationKey(poseIndex), randomBlock(1, p, rng, false));
    }
  }

  for (Key landmarkIndex : landmarkIndices)
    initial.insert(landmarkKey(landmarkIndex), randomBlock(1, p, rng, false));
  const double initializationSeconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    initializationStart).count();

  // Certifiable solver
  RiemannianStaircaseParams params;
  params.pMin = d;
  params.pMax = 10;
  params.eta = 1e-4;
  params.verbose = true;
  params.almParams->maxIterations = 100;
  params.almParams->absoluteViolationTolerance = 1e-10;
  params.almParams->absoluteStationarityTolerance = 1e-8;
  params.almParams->lmParams.maxIterations = 1000;
  params.almParams->lmParams.relativeErrorTol = 1e-12;
  params.almParams->lmParams.absoluteErrorTol = 1e-12;
  params.almParams->verbose = false;

  const auto solveStart = std::chrono::steady_clock::now();
  const auto result = RSO(graph, initial, params).optimize();
  const double solveSeconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    solveStart).count();

  // Rounding and extracting solution
  const auto roundingStart = std::chrono::steady_clock::now();
  Values rounded;
  if (result.rounded) {
    Values atRankD = result.layout.unstack(result.rounded->Yd);
    alignBlockDetSigns<d>(atRankD, poseIndices);
    for (auto& [key, R] : ExtractQcqpValues<RotT, d>(atRankD))
      rounded.insert(key, R);
    for (Key poseIndex : poseIndices) {
      const Matrix row = atRankD.at<Matrix>(translationKey(poseIndex));
      rounded.insert(translationKey(poseIndex), Point(row.row(0).transpose()));
    }
    for (Key landmarkIndex : landmarkIndices) {
      const Matrix row = atRankD.at<Matrix>(landmarkKey(landmarkIndex));
      rounded.insert(landmarkKey(landmarkIndex), Point(row.row(0).transpose()));
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
  std::cout << "Initialization:    "
            << initializationName(initializationMethod) << "\n"
            << "Initialization time: " << initializationSeconds << " s\n"
            << "QCQP build time:    " << buildSeconds << " s\n"
            << "Local solve time:  " << nlpSeconds << " s\n"
            << "Certificate time:  " << verifySeconds << " s\n"
            << "Rounding time:     " << roundingSeconds << " s\n"
            << "Total BM time:     " << result.totalTime << " s\n"
            << "Solve wall time:   " << solveSeconds << " s\n"
            << "End-to-end time:   " << initializationSeconds + solveSeconds
            << " s\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string dataPath;
  std::string initialization = "random";
  int dim = 3;
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
  } else if (initialization == "random") {
    initializationMethod = InitializationMethod::Random;
  } else {
    std::cerr << "Unknown initialization method: " << initialization << "\n";
    return 2;
  }
  if (dataPath.empty() || (dim != 2 && dim != 3)) {
    std::cerr << "Usage: " << argv[0]
              << " --data=<g2o file> --dim=<2|3> "
                 "[--initialization=<fast-sync|random>]\n";
    return 2;
  }

  if (dim == 3) {
    return runCertifiableLandmarkSLAM<3>(
        dataPath, initializationMethod);
  } else {
    return runCertifiableLandmarkSLAM<2>(
        dataPath, initializationMethod);
  }
}
