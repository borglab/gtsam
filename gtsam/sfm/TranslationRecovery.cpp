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
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/TranslationFactor.h>
#include <gtsam/sfm/TranslationRecovery.h>
#include <gtsam/slam/PriorFactor.h>

#include <set>
#include <utility>

using namespace gtsam;
using namespace std;

TranslationRecovery::TranslationRecovery(
    const TranslationRecovery::TranslationEdges &relativeTranslations,
    const LevenbergMarquardtParams &lmParams)
    : params_(lmParams) {
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

  // Add all relative translation edges
  for (auto edge : relativeTranslations_) {
    graph.emplace_shared<TranslationFactor>(edge.key1(), edge.key2(),
                                            edge.measured(), edge.noiseModel());
  }

  return graph;
}

void TranslationRecovery::addPrior(
    const double scale, NonlinearFactorGraph *graph,
    const SharedNoiseModel &priorNoiseModel) const {
  auto edge = relativeTranslations_.begin();
  if (edge == relativeTranslations_.end()) return;
  graph->emplace_shared<PriorFactor<Point3> >(edge->key1(), Point3(0, 0, 0),
                                              priorNoiseModel);
  graph->emplace_shared<PriorFactor<Point3> >(
      edge->key2(), scale * edge->measured().point3(), edge->noiseModel());
}

Values TranslationRecovery::initalizeRandomly() const {
  // Create a lambda expression that checks whether value exists and randomly
  // initializes if not.
  Values initial;
  auto insert = [&initial](Key j) {
    if (!initial.exists(j)) {
      initial.insert<Point3>(j, Vector3::Random());
    }
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

Values TranslationRecovery::run(const double scale) const {
  auto graph = buildGraph();
  addPrior(scale, &graph);
  const Values initial = initalizeRandomly();
  LevenbergMarquardtOptimizer lm(graph, initial, params_);
  Values result = lm.optimize();

  // Nodes that were not optimized are stored in sameTranslationNodes_ as a map
  // from a key that was optimized to keys that were not optimized. Iterate over
  // map and add results for keys not optimized.
  for (const auto &optimizedAndDuplicateKeys : sameTranslationNodes_) {
    Key optimizedKey = optimizedAndDuplicateKeys.first;
    std::set<Key> duplicateKeys = optimizedAndDuplicateKeys.second;
    // Add the result for the duplicate key if it does not already exist.
    for (const Key duplicateKey : duplicateKeys) {
      if (result.exists(duplicateKey)) continue;
      result.insert<Point3>(duplicateKey, result.at<Point3>(optimizedKey));
    }
  }
  return result;
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
