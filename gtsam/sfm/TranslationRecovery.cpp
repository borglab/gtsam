/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TranslationRecovery.cpp
 * @author Frank Dellaert
 * @date March 2020
 * @brief Source code for recovering translations when rotations are given
 */

#include <gtsam/sfm/TranslationRecovery.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/TranslationFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;
using namespace std;

NonlinearFactorGraph TranslationRecovery::buildGraph() const {
  NonlinearFactorGraph graph;

  // Add all relative translation edges
  for (auto edge : relativeTranslations_) {
    graph.emplace_shared<TranslationFactor>(edge.key1(), edge.key2(),
                                            edge.measured(), edge.noiseModel());
  }

  return graph;
}

void TranslationRecovery::addPrior(const double scale,
                                   NonlinearFactorGraph *graph,
                                   const SharedNoiseModel &priorNoiseModel) const {
  auto edge = relativeTranslations_.begin();
  graph->emplace_shared<PriorFactor<Point3> >(edge->key1(), Point3(0, 0, 0), priorNoiseModel);
  graph->emplace_shared<PriorFactor<Point3> >(edge->key2(), scale * edge->measured().point3(),
                                              edge->noiseModel());
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
  return initial;
}

Values TranslationRecovery::run(const double scale) const {
  auto graph = buildGraph();
  addPrior(scale, &graph);
  const Values initial = initalizeRandomly();
  LevenbergMarquardtOptimizer lm(graph, initial, params_);
  Values result = lm.optimize();
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
