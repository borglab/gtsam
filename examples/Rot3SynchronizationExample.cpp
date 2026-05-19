/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3SynchronizationExample.cpp
 * @brief   Run the Riemannian Staircase certifiable optimizer on the rotation
 *          subgraph of any 3D PGO dataset (g2o format, VERTEX_SE3:QUAT /
 *          EDGE_SE3:QUAT). The 3D analog of Rot2SynchronizationW100Example.cpp.
 *
 * Pipeline:
 *   1. Read a 3D PGO g2o file via gtsam::readG2o(file, is3D=true).
 *   2. Extract per-edge relative rotations (each EDGE_SE3:QUAT's Pose3
 *      .rotation()) and build a NonlinearFactorGraph of
 *      FrobeniusBetweenFactor<Rot3>.
 *   3. Initialize each pose's rotation per --init flag.
 *   4. Run the staircase with K (column dim) starting at 3 (the natural
 *      row-orthonormal Stiefel column count for SO(3)).
 *   5. Round the BM solution back to one Rot3 per pose via rank-d=3
 *      truncated SVD + per-block Rot3::ClosestTo projection.
 *
 * Usage:
 *   ./Rot3SynchronizationExample
 *       --data=<path>                      (required: 3D g2o file)
 *       [--init=<from_data|random>]        (default: from_data)
 *       [--verify=<lobpcg|dense>]          (default: lobpcg)
 *       [--seed=<N>]                       (default: 42; only used for random)
 */

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/RiemannianStaircaseOptimizer.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/dataset.h>

#include <Eigen/SVD>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace gtsam;

namespace {

enum class InitKind { FromData, Random };

struct CliOptions {
  std::string dataPath;
  InitKind init = InitKind::FromData;
  RiemannianStaircaseParams::VerificationMethod verify =
      RiemannianStaircaseParams::VerificationMethod::LOBPCG;
  unsigned int seed = 42;
  bool verbose = false;
};

bool startsWith(const std::string& s, const std::string& prefix) {
  return s.size() >= prefix.size() &&
         s.compare(0, prefix.size(), prefix) == 0;
}

CliOptions parseCli(int argc, char** argv) {
  CliOptions opts;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (startsWith(a, "--data=")) {
      opts.dataPath = a.substr(7);
    } else if (startsWith(a, "--init=")) {
      const std::string v = a.substr(7);
      if (v == "from_data") opts.init = InitKind::FromData;
      else if (v == "random") opts.init = InitKind::Random;
      else {
        std::cerr << "Unknown --init value: " << v
                  << " (from_data|random)\n";
        std::exit(2);
      }
    } else if (startsWith(a, "--verify=")) {
      const std::string v = a.substr(9);
      if (v == "lobpcg")
        opts.verify = RiemannianStaircaseParams::VerificationMethod::LOBPCG;
      else if (v == "dense")
        opts.verify = RiemannianStaircaseParams::VerificationMethod::DenseEigen;
      else {
        std::cerr << "Unknown --verify value: " << v << " (lobpcg|dense)\n";
        std::exit(2);
      }
    } else if (startsWith(a, "--seed=")) {
      opts.seed = static_cast<unsigned int>(std::stoul(a.substr(7)));
    } else if (startsWith(a, "--verbose=")) {
      opts.verbose = (a.substr(10) != "0");
    } else {
      std::cerr << "Unknown flag: " << a << "\n";
      std::exit(2);
    }
  }
  if (opts.dataPath.empty()) {
    std::cerr << "--data=<path> is required (3D g2o file).\n";
    std::exit(2);
  }
  return opts;
}

const char* initName(InitKind k) {
  switch (k) {
    case InitKind::FromData: return "from_data";
    case InitKind::Random:   return "random";
  }
  return "?";
}

const char* verifyName(RiemannianStaircaseParams::VerificationMethod m) {
  switch (m) {
    case RiemannianStaircaseParams::VerificationMethod::LOBPCG: return "lobpcg";
    case RiemannianStaircaseParams::VerificationMethod::DenseEigen:
      return "dense";
  }
  return "?";
}

/// Walk a Pose3 PGO graph, pull each edge's rotation component, and build a
/// new graph of FrobeniusBetweenFactor<Rot3>.
NonlinearFactorGraph buildRot3RotationGraph(const NonlinearFactorGraph& poseGraph,
                                            std::set<Key>* keysOut = nullptr) {
  NonlinearFactorGraph rotGraph;
  for (const auto& factor : poseGraph) {
    const auto between =
        std::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
    if (!between) continue;  // skip priors and other factor types
    rotGraph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(
        between->key1(), between->key2(), between->measured().rotation());
    if (keysOut) {
      keysOut->insert(between->key1());
      keysOut->insert(between->key2());
    }
  }
  return rotGraph;
}

/// Round the BM D=3 solution at rank p back to one Rot3 per pose: rank-d=3
/// truncated SVD + per-block SO(3) projection + global sign-gauge fix.
std::vector<std::pair<Key, Rot3>> roundToRot3(
    const Values& Y, const RiemannianStaircaseOptimizer::Layout& layout) {
  // 1. Assemble Yglobal of shape (totalDim x p).
  const Matrix Yglobal = layout.stack(Y);

  // 2. Rank-d=3 truncated SVD: collapse any extra (lifted) columns back to
  // the natural 3D rotation column count.
  Eigen::JacobiSVD<Matrix> svd(Yglobal,
                               Eigen::ComputeFullU | Eigen::ComputeFullV);
  const int d = 3;
  Matrix Y3 = svd.matrixU().leftCols(d) *
              svd.singularValues().head(d).asDiagonal();
  // Y3 is (totalDim x 3). Each 3-row block is approximately R^T.

  // 3. Global sign-gauge fix: the SVD truncation introduces an arbitrary
  // right-multiplication by an orthogonal V_d. If V_d is a reflection
  // (det=-1) every per-pose 3x3 block of Y3 is the reflection of the true
  // R^T (det=-1). Flipping ONE column of Y3 negates one column of each 3x3
  // block, which flips each block's determinant.
  size_t numNegDet = 0;
  for (const auto& [_, slice] : layout.slices) {
    const Matrix3 block = Y3.block(slice.offset, 0, 3, 3);
    if (block.determinant() < 0) ++numNegDet;
  }
  if (numNegDet > layout.size() / 2) Y3.col(2) *= -1.0;

  // 4. SO(3)-project each block. Block ≈ R^T, so R ≈ block^T.
  std::vector<std::pair<Key, Rot3>> rotations;
  rotations.reserve(layout.size());
  for (const auto& [key, slice] : layout.slices) {
    const Matrix3 block = Y3.block(slice.offset, 0, 3, 3);
    rotations.emplace_back(key, Rot3::ClosestTo(block.transpose()));
  }
  return rotations;
}

/// Build initial QCQP-D=3 (row-orthonormal R^T) values according to the
/// chosen strategy.
Values makeInitial(const std::set<Key>& keys, const Values& poseValues,
                   InitKind kind, unsigned int seed) {
  Values v;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> uni(-M_PI, M_PI);
  for (Key key : keys) {
    Rot3 R;
    switch (kind) {
      case InitKind::FromData: {
        if (!poseValues.exists(key)) {
          throw std::runtime_error(
              "Rot3SynchronizationExample: --init=from_data but the dataset "
              "has no pose for key " +
              std::to_string(key));
        }
        R = poseValues.at<Pose3>(key).rotation();
        break;
      }
      case InitKind::Random: {
        // Three random Tait-Bryan angles; not uniform on SO(3) but good
        // enough as a "bad init" probe.
        const Vector3 rpy(uni(rng), uni(rng), uni(rng));
        R = Rot3::RzRyRx(rpy(0), rpy(1), rpy(2));
        break;
      }
    }
    InsertQcqpValue<Rot3, 3>(key, R, &v);
  }
  return v;
}

/// Sniff the first non-comment line for VERTEX_SE3 or EDGE_SE3 to confirm
/// the file is 3D before calling readG2o(..., true).
bool isFile3D(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("isFile3D: cannot open " + path);
  std::string line;
  while (std::getline(in, line)) {
    size_t i = 0;
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i])))
      ++i;
    if (i == line.size()) continue;
    if (line[i] == '#') continue;
    auto starts = [&](const std::string& tag) {
      return line.compare(i, tag.size(), tag) == 0;
    };
    if (starts("VERTEX_SE3:QUAT") || starts("EDGE_SE3:QUAT")) return true;
    if (starts("VERTEX_SE2") || starts("EDGE_SE2") ||
        starts("VERTEX2") || starts("EDGE2")) return false;
  }
  throw std::runtime_error("isFile3D: " + path +
                           " contains no recognized vertex/edge tokens.");
}

}  // namespace

int main(int argc, char** argv) {
  const CliOptions opts = parseCli(argc, argv);
  std::cout << "data=" << opts.dataPath
            << "\ninit=" << initName(opts.init)
            << " verify=" << verifyName(opts.verify)
            << " seed=" << opts.seed << "\n\n";

  // -------------------------------------------------------------------
  // 1. Load 3D PGO graph.
  // -------------------------------------------------------------------
  if (!isFile3D(opts.dataPath)) {
    std::cerr << opts.dataPath
              << " is not a 3D PGO file (no VERTEX_SE3:QUAT/EDGE_SE3:QUAT). "
                 "Use Rot2SynchronizationW100Example for 2D data.\n";
    return 1;
  }
  NonlinearFactorGraph::shared_ptr poseGraph;
  Values::shared_ptr poseInitial;
  std::tie(poseGraph, poseInitial) = readG2o(opts.dataPath, /*is3D=*/true);
  std::cout << "Loaded: " << poseInitial->size() << " poses, "
            << poseGraph->size() << " factors\n";

  // -------------------------------------------------------------------
  // 2. Build rotation subgraph.
  // -------------------------------------------------------------------
  std::set<Key> keys;
  const NonlinearFactorGraph rotGraph =
      buildRot3RotationGraph(*poseGraph, &keys);
  std::cout << "Rotation subgraph: " << keys.size() << " vertices, "
            << rotGraph.size() << " between factors\n\n";

  // -------------------------------------------------------------------
  // 3. Initialize.
  // -------------------------------------------------------------------
  const Values initial = makeInitial(keys, *poseInitial, opts.init, opts.seed);

  // -------------------------------------------------------------------
  // 4. Run the staircase. D=3 is Rot3's natural matrix form (row-
  //    orthonormal R^T at the base of the ladder, p=d=3).
  // -------------------------------------------------------------------
  RiemannianStaircaseParams params;
  params.pMin = 3;
  params.pMax = 6;
  params.alpha = 1e-1;
  params.eta = 1e-3;
  params.verificationMethod = opts.verify;
  params.verbose = opts.verbose;
  params.almParams->maxIterations = 200;
  params.almParams->initialMuEq = 10.0;
  params.almParams->muEqIncreaseRate = 2.0;
  params.almParams->absoluteViolationTolerance = 1e-8;
  params.almParams->relativeViolationTolerance = 1e-8;

  const RiemannianStaircaseOptimizer rso(rotGraph, initial, params);
  const auto result = rso.optimize();

  // -------------------------------------------------------------------
  // 5. Report.
  // -------------------------------------------------------------------
  std::cout << "========== Result ==========\n";
  std::cout << "Certified:        " << (result.certified ? "yes" : "no")
            << "\n";
  std::cout << "Final rank:       " << result.finalRank << "\n";
  std::cout << "Min eig(S):       " << result.minEigenvalue << "\n";

  auto safeAt = [](const auto& v, size_t i, auto fallback) {
    return i < v.size() ? v[i] : fallback;
  };
  std::cout << "\nStaircase log:\n";
  std::cout << "  " << std::setw(4) << "lvl"
            << "  " << std::setw(5) << "p"
            << "  " << std::setw(13) << "cost"
            << "  " << std::setw(13) << "min_eig(S)"
            << "  " << std::setw(9) << "verified"
            << "  " << std::setw(9) << "alm_s"
            << "  " << std::setw(9) << "verify_s"
            << "  " << std::setw(9) << "lift_s" << "\n";
  std::cout << "  "
            << std::string(4 + 2 + 5 + 2 + 13 + 2 + 13 + 2 + 9 +
                               3 * (2 + 9),
                           '-')
            << "\n";
  for (size_t lvl = 0; lvl < result.ranksVisited.size(); ++lvl) {
    const double eig =
        safeAt(result.minEigenvaluePerLevel, lvl, std::nan(""));
    const bool cert = safeAt(result.certifiedPerLevel, lvl, false);
    const double almT = safeAt(result.almTimePerLevel, lvl, 0.0);
    const double verT = safeAt(result.verifyTimePerLevel, lvl, 0.0);
    const double liftT = safeAt(result.liftTimePerLevel, lvl, 0.0);
    std::cout << "  " << std::setw(4) << lvl
              << "  " << std::setw(5) << result.ranksVisited[lvl]
              << std::scientific << std::setprecision(4)
              << "  " << std::setw(13) << result.costPerLevel[lvl]
              << "  " << std::setw(13) << eig
              << "  " << std::setw(9) << (cert ? "yes" : "no")
              << std::fixed << std::setprecision(3)
              << "  " << std::setw(9) << almT
              << "  " << std::setw(9) << verT
              << "  " << std::setw(9) << (cert ? 0.0 : liftT)
              << "\n";
  }
  std::cout << "  Total staircase time : " << std::fixed
            << std::setprecision(3) << result.totalTime << " s\n";

  // -------------------------------------------------------------------
  // 6. Round to Rot3.
  // -------------------------------------------------------------------
  const auto layout =
      RiemannianStaircaseOptimizer::Layout::From(result.values);
  const auto roundStart = std::chrono::steady_clock::now();
  const auto recovered = roundToRot3(result.values, layout);
  const double roundTime = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - roundStart).count();
  std::cout << "  Rounding time        : " << std::fixed
            << std::setprecision(3) << roundTime << " s\n";
  if (recovered.empty()) return result.certified ? 0 : 1;

  // Final objective after rounding (in both conventions for clarity).
  {
    Values roundedValues;
    for (const auto& [key, R] : recovered) roundedValues.insert(key, R);
    const double roundedObj = rotGraph.error(roundedValues);
    std::cout << "\nObjective after SO(3) rounding:\n";
    std::cout << std::scientific << std::setprecision(4);
    std::cout << "  F (0.5*sum||...||^2)  : " << roundedObj << "\n";
  }

  return result.certified ? 0 : 1;
}
