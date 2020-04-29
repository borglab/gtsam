/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMBAL.cpp
 * @brief   time SFM with BAL file,  conventional GeneralSFMFactor
 * @author  Frank Dellaert
 * @date    June 6, 2015
 */

#include "timeSFMBAL.h"

#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>


using namespace std;
using namespace gtsam;

typedef PinholeCamera<Cal3Bundler> Camera;
typedef GeneralSFMFactor<Camera, Point3> SfmFactor;

int main(int argc, char* argv[]) {
  // parse options and read BAL file
  SfmData db = preamble(argc, argv);

  // Build graph using conventional GeneralSFMFactor
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.number_tracks(); j++) {
    for (const SfmMeasurement& m: db.tracks[j].measurements) {
      size_t i = m.first;
      Point2 z = m.second;
      graph.emplace_shared<SfmFactor>(z, gNoiseModel, C(i), P(j));
    }
  }

  Values initial;
  size_t i = 0, j = 0;
  for (const SfmCamera& camera: db.cameras)
    initial.insert(C(i++), camera);
  for (const SfmTrack& track: db.tracks)
    initial.insert(P(j++), track.p);

  return optimize(db, graph, initial);
}
