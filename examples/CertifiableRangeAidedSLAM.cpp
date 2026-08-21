/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CertifiableRangeAidedSLAM.cpp
 * @brief   Certifiable range-aided SLAM via the Riemannian
 *          Staircase. Minimizes
 *          Sum_ij kappa_ij ||R_j - R_i R_ij||^2_F + tau_ij ||t_j - t_i - R_i t_ij||^2
 *          + Sum_m nu_m ||p_j - p_i - d_m u_m||^2
 *          over R_i in O(d), t_i, l_k in R^d, subject to ||u_m|| = 1.
 * @author  Zhexin Xu
 *
 * Usage:
 *   ./CertifiableRangeAidedSLAM --data=<pyfg file>
 *       [--initialization=<fast-sync|random>]
 */

#include <gtsam/certifiable/RiemannianStaircaseOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FastSync.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/sam/QuadraticRangeFactor.h>
#include <gtsam/slam/RelativeTranslationFactor.h>

#include <Eigen/SVD>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
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

Key rotationKey(size_t poseIndex) { return Symbol('R', poseIndex); }
Key translationKey(size_t poseIndex) { return Symbol('t', poseIndex); }
Key landmarkKey(size_t landmarkIndex) { return Symbol('l', landmarkIndex); }
/// One auxiliary per range measurement.
Key unitVectorKey(size_t measurementIndex) {
  return Symbol('u', measurementIndex);
}

template <int d>
struct Odometry {
  using PoseT = typename std::conditional<d == 2, Pose2, Pose3>::type;
  size_t from, to;
  PoseT measured;
  double kappa, tau;
};

struct Range {
  size_t poseIndex, targetIndex;
  bool targetIsLandmark;
  double range, weight;
};

/// Parsed .pyfg contents: pose variables, landmark variables, odometry, ranges.
template <int d>
struct Dataset {
  using PoseT = typename std::conditional<d == 2, Pose2, Pose3>::type;
  std::map<std::string, size_t> poseIds, landmarkIds;
  std::vector<Odometry<d>> odometry;
  std::vector<Range> ranges;
};

int detectDimension(const std::string& path) {
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind("VERTEX_SE3", 0) == 0 || line.rfind("VERTEX_XYZ", 0) == 0)
      return 3;
    if (line.rfind("VERTEX_SE2", 0) == 0 || line.rfind("VERTEX_XY", 0) == 0)
      return 2;
  }
  throw std::invalid_argument("detectDimension: no vertex tags in " + path);
}

template <int d>
Dataset<d> readPyfg(const std::string& path) {
  using PoseT = typename Dataset<d>::PoseT;
  std::ifstream in(path);
  if (!in) throw std::invalid_argument("cannot open " + path);
  Dataset<d> data;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream stream(line);
    std::string tag;
    stream >> tag;

    // Pose vertices carry a leading timestamp; landmark vertices do not.
    if ((d == 3 && tag == "VERTEX_SE3:QUAT") ||
        (d == 2 && tag == "VERTEX_SE2")) {
      double stamp;
      std::string name;
      stream >> stamp >> name;
      const size_t index = data.poseIds.size();
      data.poseIds.emplace(name, index);
    } else if ((d == 3 && tag == "VERTEX_XYZ") ||
               (d == 2 && tag == "VERTEX_XY")) {
      std::string name;
      stream >> name;
      const size_t index = data.landmarkIds.size();
      data.landmarkIds.emplace(name, index);
    } else if ((d == 3 && tag == "EDGE_SE3:QUAT") ||
               (d == 2 && tag == "EDGE_SE2")) {
      double stamp;
      std::string a, b;
      stream >> stamp >> a >> b;
      auto ia = data.poseIds.find(a), ib = data.poseIds.find(b);
      if (ia == data.poseIds.end() || ib == data.poseIds.end()) continue;
      constexpr int dim = d == 2 ? 3 : 6;
      PoseT measured;
      if constexpr (d == 2) {
        double x, y, theta;
        stream >> x >> y >> theta;
        measured = Pose2(x, y, theta);
      } else {
        double x, y, z, qx, qy, qz, qw;
        stream >> x >> y >> z >> qx >> qy >> qz >> qw;
        measured = Pose3(Rot3(Quaternion(qw, qx, qy, qz)), Point3(x, y, z));
      }
      // These entries are covariances, not information, which differs from g2o.
      Matrix covariance = Matrix::Identity(dim, dim);
      for (int r = 0; r < dim; ++r)
        for (int c = r; c < dim; ++c) {
          double v = 0.0;
          if (!(stream >> v)) v = (r == c) ? 1.0 : 0.0;
          covariance(r, c) = v;
          covariance(c, r) = v;
        }
      double kappa, tau;
      if constexpr (d == 2) {
        tau = 2.0 / covariance.topLeftCorner<2, 2>().trace();
        kappa = 1.0 / covariance(2, 2);
      } else {
        tau = 3.0 / covariance.topLeftCorner<3, 3>().trace();
        kappa = 3.0 / (2.0 * covariance.bottomRightCorner<3, 3>().trace());
      }
      data.odometry.push_back({ia->second, ib->second, measured, kappa, tau});
    } else if (tag == "EDGE_RANGE") {
      // The trailing field is a variance.
      double stamp, range, variance;
      std::string a, b;
      stream >> stamp >> a >> b >> range >> variance;
      const double sigma = std::sqrt(variance);
      auto ia = data.poseIds.find(a);
      if (ia == data.poseIds.end()) continue;
      auto lb = data.landmarkIds.find(b);
      auto pb = data.poseIds.find(b);
      if (lb != data.landmarkIds.end())
        data.ranges.push_back(
            {ia->second, lb->second, true, range, 1.0 / (sigma * sigma)});
      else if (pb != data.poseIds.end())
        data.ranges.push_back(
            {ia->second, pb->second, false, range, 1.0 / (sigma * sigma)});
    }
  }
  return data;
}

/// Certifiable methods optimize over O(d), which has two components, det = +1
/// and det = -1, so the rounded rotations can come out reflected.
template <int d>
void alignBlockDetSigns(Values& values, size_t numPoses) {
  size_t negative = 0;
  for (size_t i = 0; i < numPoses; ++i)
    if (values.at<Matrix>(rotationKey(i)).determinant() < 0) ++negative;
  if (negative <= numPoses / 2) return;
  for (const auto& [key, M] : values.extract<Matrix>()) {
    Matrix flipped = M;
    flipped.col(M.cols() - 1) *= -1.0;
    values.update(key, flipped);
  }
}

Matrix randomBlock(int rows, int p, std::mt19937& rng, bool unitRows) {
  std::normal_distribution<double> normal(0.0, 1.0);
  Matrix X(rows, p);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < p; ++c) X(r, c) = normal(rng);
  if (!unitRows) return X;
  Eigen::JacobiSVD<Matrix> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
  return svd.matrixU() * svd.matrixV().transpose();
}

template <int d>
int runCertifiableRangeAidedSLAM(const std::string& path,
                                 InitializationMethod initializationMethod) {
  using Point = Eigen::Matrix<double, d, 1>;
  using RotT = typename std::conditional<d == 2, Rot2, Rot3>::type;
  // The auxiliary direction has no analogue in the pose-only examples, so its
  // type is derived here rather than passed in.
  using DirT = typename std::conditional<d == 2, Rot2, Unit3>::type;
  constexpr int kDirectionRows = QuadraticRangeFactor<d>::kDirectionRows;

  const auto directionFrom = [](const Point& v) -> DirT {
    if constexpr (d == 2) return Rot2::atan2(v.y(), v.x());
    else return DirT(v);
  };

  const Dataset<d> data = readPyfg<d>(path);
  if (data.ranges.empty()) {
    std::cerr << "no range measurements in " << path << "\n";
    return 1;
  }

  // Build graph
  NonlinearFactorGraph graph;
  for (const auto& edge : data.odometry) {
    graph.emplace_shared<FrobeniusBetweenFactor<RotT>>(
        rotationKey(edge.from), rotationKey(edge.to), edge.measured.rotation(),
        noiseModel::Isotropic::Variance(traits<RotT>::dimension,
                                        1.0 / edge.kappa));
    graph.emplace_shared<RelativeTranslationFactor<d>>(
        rotationKey(edge.from), translationKey(edge.from),
        translationKey(edge.to), Point(edge.measured.translation()), edge.tau);
  }
  for (size_t m = 0; m < data.ranges.size(); ++m) {
    const auto& r = data.ranges[m];
    const Key target = r.targetIsLandmark ? landmarkKey(r.targetIndex)
                                          : translationKey(r.targetIndex);
    graph.emplace_shared<QuadraticRangeFactor<d>>(
        translationKey(r.poseIndex), target, unitVectorKey(m), r.range,
        r.weight);
  }
  std::cout << path.substr(path.find_last_of('/') + 1)
            << "  poses=" << data.poseIds.size()
            << "  landmarks=" << data.landmarkIds.size()
            << "  odometry=" << data.odometry.size()
            << "  ranges=" << data.ranges.size() << "\n";

  const int p = d;
  std::mt19937 rng(kSeed);

  // Initialization
  const auto initializationStart = std::chrono::steady_clock::now();
  Values initial;
  if (initializationMethod == InitializationMethod::FastSync) {
    // fastSync only does the rotation initialization here.
    NonlinearFactorGraph rotationGraph;
    for (const auto& edge : data.odometry) {
      rotationGraph.emplace_shared<FrobeniusBetweenFactor<RotT>>(
          rotationKey(edge.from), rotationKey(edge.to),
          edge.measured.rotation(),
          noiseModel::Isotropic::Variance(traits<RotT>::dimension,
                                          1.0 / edge.kappa));
    }
    const Values rotationEstimate = fastSync<RotT>(rotationGraph);
    for (const auto& [name, index] : data.poseIds) {
      RotT rotation;
      if (rotationEstimate.exists(rotationKey(index)))
        rotation = rotationEstimate.at<RotT>(rotationKey(index));
      InsertQcqpValue<RotT, d>(rotationKey(index), rotation, &initial);
      initial.insert(translationKey(index), randomBlock(1, p, rng, false));
    }
    for (const auto& [name, index] : data.landmarkIds)
      initial.insert(landmarkKey(index), randomBlock(1, p, rng, false));
    for (size_t m = 0; m < data.ranges.size(); ++m)
      initial.insert(unitVectorKey(m),
                     randomBlock(kDirectionRows, p, rng, true));
  } else {
    for (const auto& [name, index] : data.poseIds) {
      initial.insert(rotationKey(index), randomBlock(d, p, rng, true));
      initial.insert(translationKey(index), randomBlock(1, p, rng, false));
    }
    for (const auto& [name, index] : data.landmarkIds)
      initial.insert(landmarkKey(index), randomBlock(1, p, rng, false));
    for (size_t m = 0; m < data.ranges.size(); ++m)
      initial.insert(unitVectorKey(m),
                     randomBlock(kDirectionRows, p, rng, true));
  }
  const double initializationSeconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    initializationStart).count();

  // Certifiable solver
  RiemannianStaircaseParams params;
  params.pMin = d;
  params.pMax = 10;
  params.eta = 1e-3;
  params.verbose = true;
  params.almParams->maxIterations = 100;
  params.almParams->absoluteViolationTolerance = 1e-8;
  params.almParams->absoluteStationarityTolerance = 1e-8;
  params.almParams->lmParams.maxIterations = 500;
  params.almParams->lmParams.relativeErrorTol = 1e-8;
  params.almParams->lmParams.absoluteErrorTol = 1e-8;
  params.almParams->verbose = false;

  const auto solveStart = std::chrono::steady_clock::now();
  const auto result = RSO(graph, initial, params).optimize();
  const double solveSeconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    solveStart).count();

  // Rounding and extracting solution
  const auto recoverTyped = [&](const Values& atRankDInput) {
    Values atRankD = atRankDInput;
    alignBlockDetSigns<d>(atRankD, data.poseIds.size());
    Values typed;
    // ExtractQcqpValues selects by block shape, and a lifted Rot2 auxiliary
    // has the same shape as a lifted rotation, so select by key instead.
    for (size_t i = 0; i < data.poseIds.size(); ++i)
      typed.insert(rotationKey(i), traits<RotT>::template FromQcqpValue<d>(
                                       atRankD.at<Matrix>(rotationKey(i))));
    for (size_t i = 0; i < data.poseIds.size(); ++i)
      typed.insert(translationKey(i),
                   Point(atRankD.at<Matrix>(translationKey(i))
                             .row(0).transpose()));
    for (size_t k = 0; k < data.landmarkIds.size(); ++k)
      typed.insert(landmarkKey(k),
                   Point(atRankD.at<Matrix>(landmarkKey(k))
                             .row(0).transpose()));

    for (size_t m = 0; m < data.ranges.size(); ++m)
      typed.insert(unitVectorKey(m),
                   directionFrom(Point(atRankD.at<Matrix>(unitVectorKey(m))
                                           .row(0).head(d).transpose())));
    return typed;
  };

  const auto roundingStart = std::chrono::steady_clock::now();
  Values rounded;
  if (result.rounded) {
    rounded = recoverTyped(result.layout.unstack(result.rounded->Yd));
  }
  const double roundingSeconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    roundingStart).count();

  std::cout << "\n========== Result ==========\n"
            << "Certified:         " << (result.certified ? "yes" : "no")
            << "\n"
            << "Final rank:        " << result.finalRank << "\n";

  // Local refinement
  if (rounded.empty()) {
    std::cout << "Refined objective: n/a\n";
  } else {
    // Project the SDP solution to the feasible set and refine; see Alg. 2 in
    // CORA. The relaxation tightness does not always hold for RA-SLAM, so the
    // projection gives an approximate solution of the RA-SLAM problem, which
    // can then be refined using local optimization.
    const auto localStart = std::chrono::steady_clock::now();
    LevenbergMarquardtParams localParams;
    localParams.maxIterations = 300;
    localParams.relativeErrorTol = 1e-7;
    localParams.absoluteErrorTol = 1e-7;
    const Values localRefined =
        LevenbergMarquardtOptimizer(graph, rounded, localParams).optimize();
    const double localSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      localStart).count();
    std::cout << "Refined objective: " << graph.error(localRefined)
              << "   (" << localSeconds << " s)\n";
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
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg.rfind("--data=", 0) == 0) {
      dataPath = arg.substr(7);
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
  if (dataPath.empty()) {
    std::cerr << "Usage: " << argv[0]
              << " --data=<pyfg file> "
                 "[--initialization=<fast-sync|random>]\n";
    return 2;
  }

  if (detectDimension(dataPath) == 3) {
    return runCertifiableRangeAidedSLAM<3>(
        dataPath, initializationMethod);
  } else {
    return runCertifiableRangeAidedSLAM<2>(
        dataPath, initializationMethod);
  }
}
