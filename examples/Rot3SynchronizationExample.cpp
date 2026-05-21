/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3SynchronizationExample.cpp
 * @brief   Riemannian Staircase rotation-only synchronization on a 3D PGO
 *          g2o file. Random initial rotations, LOBPCG verification.
 * @author  Zhexin Xu  (xu.zhex@northeastern.edu)
 * @author  David M. Rosen
 *
 * Usage:
 *   ./Rot3SynchronizationExample --data=<path-to-3D-g2o-file>
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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <set>
#include <string>

using namespace gtsam;

namespace {

NonlinearFactorGraph buildRotationGraph(const NonlinearFactorGraph& poseGraph,
                                        std::set<Key>* keys) {
  NonlinearFactorGraph rotGraph;
  for (const auto& factor : poseGraph) {
    const auto between = std::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
    if (!between) continue;
    rotGraph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(
        between->key1(), between->key2(), between->measured().rotation());
    keys->insert(between->key1());
    keys->insert(between->key2());
  }
  return rotGraph;
}

/// Random Rot3 from uniform Euler angles (not Haar-uniform on SO(3)).
Values randomInitial(const std::set<Key>& keys, unsigned int seed) {
  Values v;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> uni(-M_PI, M_PI);
  for (Key key : keys) {
    const Vector3 rpy(uni(rng), uni(rng), uni(rng));
    InsertQcqpValue<Rot3, 3>(key, Rot3::RzRyRx(rpy(0), rpy(1), rpy(2)), &v);
  }
  return v;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 || std::string(argv[1]).substr(0, 7) != "--data=") {
    std::cerr << "Usage: " << argv[0] << " --data=<3D g2o file>\n";
    return 2;
  }
  const std::string dataPath = std::string(argv[1]).substr(7);

  auto [poseGraph, poseInitial] = readG2o(dataPath, /*is3D=*/true);
  if (!poseGraph) {
    std::cerr << "Failed to load " << dataPath << "\n";
    return 2;
  }

  std::set<Key> keys;
  const NonlinearFactorGraph graph = buildRotationGraph(*poseGraph, &keys);
  std::cout << "Loaded " << keys.size() << " poses, " << graph.size()
            << " rotation factors from " << dataPath << "\n\n";

  const Values initial = randomInitial(keys, /*seed=*/60);

  RiemannianStaircaseParams params;
  params.pMin = 3;  // Rot3 intrinsic row dim.
  params.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::LOBPCG;
  params.verbose = true;

  QcqpProblem qcqp(graph, params.pMin);
  RiemannianStaircaseOptimizer rso(qcqp, initial, params);

  const auto t0 = std::chrono::steady_clock::now();
  const auto result = rso.optimize<Rot3>();
  const double wallSeconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();

  // Paper convention F = ||residual||^2 = 2 * graph.error.
  double roundedObj = std::nan("");
  if (!result.rounded.empty()) roundedObj = 2.0 * graph.error(result.rounded);

  std::cout << "\n========== Result ==========\n"
            << "Certified:        " << (result.certified ? "yes" : "no") << "\n"
            << "Final rank:       " << result.finalRank << "\n"
            << "Min eig(S):       " << std::scientific << std::setprecision(4)
            << result.minEigenvalue << "\n"
            << "F (sum||...||^2): " << roundedObj << "\n"
            << "Wall time:        " << std::fixed << std::setprecision(3)
            << wallSeconds << " s\n";

  return result.certified ? 0 : 1;
}
