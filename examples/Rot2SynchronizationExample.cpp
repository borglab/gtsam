/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot2SynchronizationExample.cpp
 * @brief   Riemannian Staircase rotation-only synchronization on a 2D PGO
 *          file (g2o or TORO). Random initial rotations, LOBPCG verification.
 * @author  Zhexin Xu  (xu.zhex@northeastern.edu)
 * @author  David M. Rosen
 *
 * Usage:
 *   ./Rot2SynchronizationExample --data=<path-to-2D-pgo-file>
 */

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/RiemannianStaircaseOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/dataset.h>

#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <set>
#include <stdexcept>
#include <string>

using namespace gtsam;

namespace {

/// Pick a noise format from the first token; load2D's auto-detection fails
/// on g2o files with no VERTEX_SE2 priors.
NoiseFormat sniffNoiseFormat(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);
  std::string line;
  while (std::getline(in, line)) {
    size_t i = 0;
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i])))
      ++i;
    if (i == line.size() || line[i] == '#') continue;
    auto starts = [&](const std::string& tag) {
      return line.compare(i, tag.size(), tag) == 0 &&
             (line.size() == i + tag.size() ||
              std::isspace(static_cast<unsigned char>(line[i + tag.size()])));
    };
    if (starts("VERTEX_SE2") || starts("EDGE_SE2")) return NoiseFormatG2O;
    if (starts("VERTEX2") || starts("EDGE2")) return NoiseFormatAUTO;
  }
  throw std::runtime_error("no recognized PGO tokens in " + path);
}

/// One `FrobeniusBetweenFactor<Rot2>` per `BetweenFactor<Pose2>` edge.
NonlinearFactorGraph buildRotationGraph(const NonlinearFactorGraph& poseGraph,
                                        std::set<Key>* keys) {
  NonlinearFactorGraph rotGraph;
  for (const auto& factor : poseGraph) {
    const auto between = std::dynamic_pointer_cast<BetweenFactor<Pose2>>(factor);
    if (!between) continue;
    rotGraph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        between->key1(), between->key2(), between->measured().rotation());
    keys->insert(between->key1());
    keys->insert(between->key2());
  }
  return rotGraph;
}

/// Random Rot2 per key, uniform on the circle.
Values randomInitial(const std::set<Key>& keys, unsigned int seed) {
  Values v;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> uni(-M_PI, M_PI);
  for (Key key : keys) {
    InsertQcqpValue<Rot2, 2>(key, Rot2::fromAngle(uni(rng)), &v);
  }
  return v;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 || std::string(argv[1]).substr(0, 7) != "--data=") {
    std::cerr << "Usage: " << argv[0] << " --data=<2D PGO file>\n";
    return 2;
  }
  const std::string dataPath = std::string(argv[1]).substr(7);

  const NoiseFormat nf = sniffNoiseFormat(dataPath);
  auto [poseGraph, poseInitial] =
      load2D(dataPath, /*model=*/nullptr, /*maxIndex=*/0,
             /*addNoise=*/false, /*smart=*/true, nf);
  if (!poseGraph) {
    std::cerr << "Failed to load " << dataPath << "\n";
    return 2;
  }

  std::set<Key> keys;
  const NonlinearFactorGraph graph = buildRotationGraph(*poseGraph, &keys);
  std::cout << "Loaded " << keys.size() << " poses, " << graph.size()
            << " rotation factors from " << dataPath << "\n\n";

  const Values initial = randomInitial(keys, /*seed=*/42);

  RiemannianStaircaseParams params;
  params.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::LOBPCG;
  params.verbose = true;

  QcqpProblem qcqp(graph, params.pMin);
  RiemannianStaircaseOptimizer rso(qcqp, initial, params);

  const auto t0 = std::chrono::steady_clock::now();
  const auto result = rso.optimize<Rot2>();
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
