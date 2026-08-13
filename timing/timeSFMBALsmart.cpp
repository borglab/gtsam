/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMBALsmart.cpp
 * @brief   time SFM with BAL file,  SmartProjectionFactor
 * @author  Frank Dellaert
 * @date    Feb 26, 2016
 */

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/SmartProjectionFactor.h>

#include "timeSFMBAL.h"

using namespace std;
using namespace gtsam;
using symbol_shorthand::C;
namespace bal = gtsam::timing::bal;

typedef PinholeCamera<Cal3Bundler> Camera;
typedef SmartProjectionFactor<Camera> SfmFactor;

int main(int argc, char* argv[]) {
  // parse options and read BAL file
  bal::BalBenchmarkInput input = bal::parseSmallBenchmark(argc, argv);
  const SfmData& db = input.data;

  // Add smart factors to graph
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.numberTracks(); j++) {
    auto smartFactor =
        std::make_shared<SfmFactor>(input.config.projectionNoise);
    for (const SfmMeasurement& m : db.tracks[j].measurements) {
      size_t i = m.first;
      Point2 z = m.second;
      smartFactor->add(z, C(i));
    }
    graph.push_back(smartFactor);
  }

  Values initial;
  size_t i = 0;
  input.config.useSchur = false;
  for (const SfmCamera& camera : db.cameras) initial.insert(C(i++), camera);

  return bal::optimize(db, graph, initial, input.config);
}
