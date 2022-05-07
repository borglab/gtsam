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

TranslationRecovery::TranslationRecovery(
    const TranslationRecovery::TranslationEdges &relativeTranslations,
    const TranslationRecoveryParams &params)
    : params_(params) {
  // Some relative translations may be zero. We treat nodes that have a zero
  // relativeTranslation as a single node.

  // A DSFMap is used to find sets of nodes that have a zero relative
  // translation. Add the nodes in each edge to the DSFMap, and merge nodes that
  // are connected by a zero relative translation.
  DSFMap<Key> sameTranslationDSF;
  for (const auto &edge : relativeTranslations) {
    Key key1 = sameTranslationDSF.find(edge.key1());
    Key key2 = sameTranslationDSF.find(edge.key2());
    if (key1 != key2 && edge.measured().equals(Unit3(0.0, 0.0, 0.0))) {
      sameTranslationDSF.merge(key1, key2);
    }
  }
  // Use only those edges for which two keys have a distinct root in the DSFMap.
  for (const auto &edge : relativeTranslations) {
    Key key1 = sameTranslationDSF.find(edge.key1());
    Key key2 = sameTranslationDSF.find(edge.key2());
    if (key1 == key2) continue;
    relativeTranslations_.emplace_back(key1, key2, edge.measured(),
                                       edge.noiseModel());
  }
  // Store the DSF map for post-processing results.
  sameTranslationNodes_ = sameTranslationDSF.sets();
}

NonlinearFactorGraph TranslationRecovery::buildGraph() const {
  NonlinearFactorGraph graph;

  // Add translation factors for input translation directions.
  for (auto edge : relativeTranslations_) {
    graph.emplace_shared<TranslationFactor>(edge.key1(), edge.key2(),
                                            edge.measured(), edge.noiseModel());
  }
  return graph;
}

void TranslationRecovery::addPrior(
    const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
    const double scale, const boost::shared_ptr<NonlinearFactorGraph> graph,
    const SharedNoiseModel &priorNoiseModel) const {
  auto edge = relativeTranslations_.begin();
  if (edge == relativeTranslations_.end()) return;
  graph->emplace_shared<PriorFactor<Point3>>(edge->key1(), Point3(0, 0, 0),
                                             priorNoiseModel);

  // Add between factors for optional relative translations.
  for (auto edge : betweenTranslations) {
    Key k1 = getUniqueKey(edge.key1()), k2 = getUniqueKey(edge.key2());
    if (k1 != k2) {
      graph->emplace_shared<BetweenFactor<Point3>>(k1, k2, edge.measured(),
                                                   edge.noiseModel());
    }
  }

  // Add a scale prior only if no other between factors were added.
  if (betweenTranslations.empty()) {
    graph->emplace_shared<PriorFactor<Point3>>(
        edge->key2(), scale * edge->measured().point3(), edge->noiseModel());
  }
}

Key TranslationRecovery::getUniqueKey(const Key i) const {
  for (const auto &optimizedAndDuplicateKeys : sameTranslationNodes_) {
    Key optimizedKey = optimizedAndDuplicateKeys.first;
    std::set<Key> duplicateKeys = optimizedAndDuplicateKeys.second;
    if (i == optimizedKey || duplicateKeys.count(i)) return optimizedKey;
  }
  // Unlikely case, when i is not in the graph.
  return i;
}

Values TranslationRecovery::initializeRandomly(std::mt19937 *rng) const {
  uniform_real_distribution<double> randomVal(-1, 1);
  // Create a lambda expression that checks whether value exists and randomly
  // initializes if not.
  Values initial;
  const Values inputInitial = params_.getInitialValues();
  auto insert = [&](Key j) {
    if (initial.exists(j)) return;
    if (inputInitial.exists(j)) {
      initial.insert<Point3>(j, inputInitial.at<Point3>(j));
    } else {
      initial.insert<Point3>(
          j, Point3(randomVal(*rng), randomVal(*rng), randomVal(*rng)));
    }
    // Assumes all nodes connected by zero-edges have the same initialization.
  };

  // Loop over measurements and add a random translation
  for (auto edge : relativeTranslations_) {
    insert(edge.key1());
    insert(edge.key2());
  }

  // If there are no valid edges, but zero-distance edges exist, initialize one
  // of the nodes in a connected component of zero-distance edges.
  if (initial.empty() && !sameTranslationNodes_.empty()) {
    for (const auto &optimizedAndDuplicateKeys : sameTranslationNodes_) {
      Key optimizedKey = optimizedAndDuplicateKeys.first;
      initial.insert<Point3>(optimizedKey, Point3(0, 0, 0));
    }
  }
  return initial;
}

Values TranslationRecovery::initializeRandomly() const {
  return initializeRandomly(&kRandomNumberGenerator);
}

Values TranslationRecovery::run(
    const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
    const double scale) const {
  boost::shared_ptr<NonlinearFactorGraph> graph_ptr =
      boost::make_shared<NonlinearFactorGraph>(buildGraph());
  addPrior(betweenTranslations, scale, graph_ptr);
  const Values initial = initializeRandomly();
  LevenbergMarquardtOptimizer lm(*graph_ptr, initial, params_.getLMParams());
  Values result = lm.optimize();
  return addSameTranslationNodes(result);
}

Values TranslationRecovery::addSameTranslationNodes(
    const Values &result) const {
  Values final_result = result;
  // Nodes that were not optimized are stored in sameTranslationNodes_ as a map
  // from a key that was optimized to keys that were not optimized. Iterate over
  // map and add results for keys not optimized.
  for (const auto &optimizedAndDuplicateKeys : sameTranslationNodes_) {
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
