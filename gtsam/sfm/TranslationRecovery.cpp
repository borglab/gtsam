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
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/TranslationFactor.h>
#include <gtsam/sfm/TranslationRecovery.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/expressions.h>

#include <set>
#include <utility>

using namespace gtsam;
using namespace std;

// In Wrappers we have no access to this so have a default ready.
static std::mt19937 kRandomNumberGenerator(42);

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

NonlinearFactorGraph TranslationRecovery::buildGraph(
    const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations) const {
  NonlinearFactorGraph graph;

  // Add translation factors for input translation directions.
  for (auto edge : relativeTranslations) {
    graph.emplace_shared<TranslationFactor>(edge.key1(), edge.key2(),
                                            edge.measured(), edge.noiseModel());
  }
  return graph;
}

void TranslationRecovery::addPrior(
    const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations,
    const double scale,
    const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
    NonlinearFactorGraph *graph,
    const SharedNoiseModel &priorNoiseModel) const {
  auto edge = relativeTranslations.begin();
  if (edge == relativeTranslations.end()) return;
  graph->emplace_shared<PriorFactor<Point3>>(edge->key1(), Point3(0, 0, 0),
                                             priorNoiseModel);

  // Add a scale prior only if no other between factors were added.
  if (betweenTranslations.empty()) {
    graph->emplace_shared<PriorFactor<Point3>>(
        edge->key2(), scale * edge->measured().point3(), edge->noiseModel());
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
  uniform_real_distribution<double> randomVal(-1, 1);
  // Create a lambda expression that checks whether value exists and randomly
  // initializes if not.
  Values initial;
  auto insert = [&](Key j) {
    if (initial.exists(j)) return;
    if (initialValues.exists(j)) {
      initial.insert<Point3>(j, initialValues.at<Point3>(j));
    } else {
      initial.insert<Point3>(
          j, Point3(randomVal(*rng), randomVal(*rng), randomVal(*rng)));
    }
    // Assumes all nodes connected by zero-edges have the same initialization.
  };

  // Loop over measurements and add a random translation
  for (auto edge : relativeTranslations) {
    insert(edge.key1());
    insert(edge.key2());
  }
  // There may be nodes in betweenTranslations that do not have a measurement.
  for (auto edge : betweenTranslations) {
    insert(edge.key1());
    insert(edge.key2());
  }
  return initial;
}

Values TranslationRecovery::initializeRandomly(
    const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations,
    const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
    const Values &initialValues) const {
  return initializeRandomly(relativeTranslations, betweenTranslations,
                            &kRandomNumberGenerator, initialValues);
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
  auto edgeNoiseModel = noiseModel::Isotropic::Sigma(3, 0.01);
  TranslationEdges relativeTranslations;
  for (auto edge : edges) {
    Key a, b;
    tie(a, b) = edge;
    const Pose3 wTa = poses.at<Pose3>(a), wTb = poses.at<Pose3>(b);
    const Point3 Ta = wTa.translation(), Tb = wTb.translation();
    const Unit3 w_aZb(Tb - Ta);
    relativeTranslations.emplace_back(a, b, w_aZb, edgeNoiseModel);
  }
  return relativeTranslations;
}
