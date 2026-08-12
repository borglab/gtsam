/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LocationRecovery.cpp
 * @author Kathir Gounder, Frank Dellaert
 * @date April 2026
 * @brief Recover absolute Point3 locations from pairwise Unit3 direction
 *        measurements.
 */

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/LocationRecovery.h>
#include <gtsam/sfm/TranslationFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <random>
#include <set>

using namespace gtsam;
using namespace std;

namespace {
// In Wrappers we have no access to this so have a default ready.
std::mt19937 kPRNG(42);
}  // namespace

SharedNoiseModel LocationRecovery::convertNoiseModel(
    const SharedNoiseModel &unit3NoiseModel) {
  using noiseModel::Isotropic;
  if (auto isotropic =
          std::dynamic_pointer_cast<Isotropic>(unit3NoiseModel)) {
    return noiseModel::Isotropic::Sigma(3, isotropic->sigma());
  }
  if (auto robust = std::dynamic_pointer_cast<noiseModel::Robust>(
          unit3NoiseModel)) {
    return noiseModel::Robust::Create(robust->robust(),
                                      convertNoiseModel(robust->noise()));
  }
  throw std::runtime_error(
      "LocationRecovery::convertNoiseModel: only isotropic (optionally "
      "robust-wrapped) noise model supported.");
}

NonlinearFactorGraph LocationRecovery::buildGraph(
    const DirectionEdges &edges, bool bilinear) const {
  NonlinearFactorGraph graph;
  uint64_t i = 0;
  for (const auto &edge : edges) {
    auto model = convertNoiseModel(edge.noiseModel());
    if (bilinear) {
      graph.emplace_shared<BilinearAngleTranslationFactor>(
          edge.key1(), edge.key2(), Symbol('S', i), edge.measured(), model);
    } else {
      graph.emplace_shared<TranslationFactor>(edge.key1(), edge.key2(),
                                              edge.measured(), model);
    }
    i++;
  }
  return graph;
}

void LocationRecovery::addAnchorPrior(
    Key anchorKey, NonlinearFactorGraph *graph,
    const SharedNoiseModel &priorNoiseModel) const {
  graph->addPrior<Point3>(anchorKey, Point3(0, 0, 0), priorNoiseModel);
}

Values LocationRecovery::initializeRandomly(
    const std::set<Key> &keys, size_t numEdges, bool bilinear,
    std::mt19937 *rng, const Values &initialValues) const {
  uniform_real_distribution<double> randomVal(-1, 1);
  Values initial;

  auto insert = [&](Key j) {
    if (initial.exists(j)) return;
    if (initialValues.exists(j)) {
      initial.insert<Point3>(j, initialValues.at<Point3>(j));
    } else {
      initial.insert<Point3>(
          j, Point3(randomVal(*rng), randomVal(*rng), randomVal(*rng)));
    }
  };

  for (const Key &key : keys) {
    insert(key);
  }

  if (bilinear) {
    for (uint64_t i = 0; i < numEdges; i++) {
      initial.insert<Vector1>(Symbol('S', i), Vector1(1.0));
    }
  }

  return initial;
}

Values LocationRecovery::initializeRandomly(
    const std::set<Key> &keys, size_t numEdges, bool bilinear,
    const Values &initialValues) const {
  return initializeRandomly(keys, numEdges, bilinear, &kPRNG, initialValues);
}
