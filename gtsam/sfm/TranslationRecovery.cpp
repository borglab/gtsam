/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TranslationRecovery.cpp
 * @author Frank Dellaert, Akshay Krishnan
 * @date March 2020
 * @brief Source code for recovering translations when rotations are given
 */

#include <gtsam/base/DSFMap.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/TranslationRecovery.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <set>
#include <utility>

using namespace gtsam;
using namespace std;

namespace {
// In Wrappers we have no access to this so have a default ready.
std::mt19937 kPRNG(42);

// Some relative translations may be zero. We treat nodes that have a zero
// relativeTranslation as a single node.
// A DSFMap is used to find sets of nodes that have a zero relative
// translation. Add the nodes in each edge to the DSFMap, and merge nodes that
// are connected by a zero relative translation.
DSFMap<Key> getSameTranslationDSFMap(
    const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations) {
  DSFMap<Key> sameTranslationDSF;
  for (const auto &edge : relativeTranslations) {
    Key key1 = sameTranslationDSF.find(edge.key1());
    Key key2 = sameTranslationDSF.find(edge.key2());
    if (key1 != key2 && edge.measured().equals(Unit3(0.0, 0.0, 0.0))) {
      sameTranslationDSF.merge(key1, key2);
    }
  }
  return sameTranslationDSF;
}

// Removes zero-translation edges from measurements, and combines the nodes in
// these edges into a single node.
template <typename T>
std::vector<BinaryMeasurement<T>> removeSameTranslationNodes(
    const std::vector<BinaryMeasurement<T>> &edges,
    const DSFMap<Key> &sameTranslationDSFMap) {
  std::vector<BinaryMeasurement<T>> newEdges;
  for (const auto &edge : edges) {
    Key key1 = sameTranslationDSFMap.find(edge.key1());
    Key key2 = sameTranslationDSFMap.find(edge.key2());
    if (key1 == key2) continue;
    newEdges.emplace_back(key1, key2, edge.measured(), edge.noiseModel());
  }
  return newEdges;
}

// Adds nodes that were not optimized for because they were connected
// to another node with a zero-translation edge in the input.
Values addSameTranslationNodes(const Values &result,
                               const DSFMap<Key> &sameTranslationDSFMap) {
  Values final_result = result;
  // Nodes that were not optimized are stored in sameTranslationNodes_ as a map
  // from a key that was optimized to keys that were not optimized. Iterate over
  // map and add results for keys not optimized.
  for (const auto &optimizedAndDuplicateKeys : sameTranslationDSFMap.sets()) {
    Key optimizedKey = optimizedAndDuplicateKeys.first;
    std::set<Key> duplicateKeys = optimizedAndDuplicateKeys.second;
    // Add the result for the duplicate key if it does not already exist.
    for (const Key duplicateKey : duplicateKeys) {
      if (final_result.exists(duplicateKey)) continue;
      final_result.insert<Point3>(duplicateKey,
                                  final_result.at<Point3>(optimizedKey));
    }
  }
  return final_result;
}
}  // namespace

NonlinearFactorGraph TranslationRecovery::buildGraph(
    const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations) const {
  return LocationRecovery::buildGraph(relativeTranslations,
                                     use_bilinear_translation_factor_);
}

void TranslationRecovery::addPrior(
    const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations,
    const double scale,
    const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
    NonlinearFactorGraph *graph,
    const SharedNoiseModel &priorNoiseModel) const {
  auto edge = relativeTranslations.begin();
  if (edge == relativeTranslations.end()) return;
  graph->addPrior<Point3>(edge->key1(), Point3(0, 0, 0), priorNoiseModel);

  // Add a scale prior only if no other between factors were added.
  if (betweenTranslations.empty()) {
    auto model = convertNoiseModel(edge->noiseModel());
    graph->addPrior<Point3>(edge->key2(), scale * edge->measured(), model);
    return;
  }

  // Add between factors for optional relative translations.
  for (auto prior_edge : betweenTranslations) {
    graph->emplace_shared<BetweenFactor<Point3>>(
        prior_edge.key1(), prior_edge.key2(), prior_edge.measured(),
        prior_edge.noiseModel());
  }
}

Values TranslationRecovery::initializeRandomly(
    const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations,
    const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
    std::mt19937 *rng, const Values &initialValues) const {
  // Collect all keys from edges.
  std::set<Key> keys;
  for (const auto &edge : relativeTranslations) {
    keys.insert(edge.key1());
    keys.insert(edge.key2());
  }
  for (const auto &edge : betweenTranslations) {
    keys.insert(edge.key1());
    keys.insert(edge.key2());
  }

  return LocationRecovery::initializeRandomly(
      keys, relativeTranslations.size(), use_bilinear_translation_factor_, rng,
      initialValues);
}

Values TranslationRecovery::initializeRandomly(
    const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations,
    const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
    const Values &initialValues) const {
  return initializeRandomly(relativeTranslations, betweenTranslations, &kPRNG,
                            initialValues);
}

Values TranslationRecovery::run(
    const TranslationEdges &relativeTranslations, const double scale,
    const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
    const Values &initialValues) const {
  // Find edges that have a zero-translation, and recompute relativeTranslations
  // and betweenTranslations by retaining only one node for every zero-edge.
  DSFMap<Key> sameTranslationDSFMap =
      getSameTranslationDSFMap(relativeTranslations);
  const TranslationEdges nonzeroRelativeTranslations =
      removeSameTranslationNodes(relativeTranslations, sameTranslationDSFMap);
  const std::vector<BinaryMeasurement<Point3>> nonzeroBetweenTranslations =
      removeSameTranslationNodes(betweenTranslations, sameTranslationDSFMap);

  // Create graph of translation factors.
  NonlinearFactorGraph graph = buildGraph(nonzeroRelativeTranslations);

  // Add global frame prior and scale (either from betweenTranslations or
  // scale).
  addPrior(nonzeroRelativeTranslations, scale, nonzeroBetweenTranslations,
           &graph);

  // Uses initial values from params if provided.
  Values initial = initializeRandomly(
      nonzeroRelativeTranslations, nonzeroBetweenTranslations, initialValues);

  // If there are no valid edges, but zero-distance edges exist, initialize one
  // of the nodes in a connected component of zero-distance edges.
  if (initial.empty() && !sameTranslationDSFMap.sets().empty()) {
    for (const auto &optimizedAndDuplicateKeys : sameTranslationDSFMap.sets()) {
      Key optimizedKey = optimizedAndDuplicateKeys.first;
      initial.insert<Point3>(optimizedKey, Point3(0, 0, 0));
    }
  }

  LevenbergMarquardtOptimizer lm(graph, initial, lmParams_);
  Values result = lm.optimize();
  return addSameTranslationNodes(result, sameTranslationDSFMap);
}

TranslationRecovery::TranslationEdges TranslationRecovery::SimulateMeasurements(
    const Values &poses, const vector<KeyPair> &edges) {
  auto edgeNoiseModel = noiseModel::Isotropic::Sigma(2, 0.01);
  TranslationEdges relativeTranslations;
  for (const auto& [a, b] : edges) {
    const Pose3 wTa = poses.at<Pose3>(a), wTb = poses.at<Pose3>(b);
    const Point3 Ta = wTa.translation(), Tb = wTb.translation();
    const Unit3 w_aZb(Tb - Ta);
    relativeTranslations.emplace_back(a, b, w_aZb, edgeNoiseModel);
  }
  return relativeTranslations;
}
