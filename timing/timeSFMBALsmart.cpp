/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
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

#include "timeSFMBAL.h"

#include <gtsam/slam/SmartProjectionFactor.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>

using namespace std;
using namespace gtsam;

typedef PinholeCamera<Cal3Bundler> Camera;
typedef SmartProjectionFactor<Camera> SfmFactor;

int main(int argc, char* argv[]) {
  // parse options and read BAL file
  SfmData db = preamble(argc, argv);

  // Add smart factors to graph
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.number_tracks(); j++) {
    auto smartFactor = boost::make_shared<SfmFactor>(gNoiseModel);
    for (const SfmMeasurement& m : db.tracks[j].measurements) {
      size_t i = m.first;
      Point2 z = m.second;
      smartFactor->add(z, C(i));
    }
    graph.push_back(smartFactor);
  }

  Values initial;
  size_t i = 0;
  gUseSchur = false;
  for (const SfmCamera& camera : db.cameras)
    initial.insert(C(i++), camera);

  return optimize(db, graph, initial);
}
