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

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>

using namespace std;
using namespace gtsam;

typedef PinholeCamera<Cal3Bundler> Camera;
typedef GeneralSFMFactor<Camera, Point3> SfmFactor;

int main(int argc, char* argv[]) {
  // parse options and read BAL file
  SfmData db = preamble(argc, argv);

  // 1. Build graph using conventional GeneralSFMFactor
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.numberTracks(); j++) {
    for (const SfmMeasurement& m : db.tracks[j].measurements) {
      size_t i = m.first;
      Point2 z = m.second;
      graph.emplace_shared<SfmFactor>(z, gNoiseModel, C(i), P(j));
    }
  }
  Values initial;
  size_t i = 0, j = 0;
  for (const SfmCamera& camera : db.cameras) initial.insert(C(i++), camera);
  for (const SfmTrack& track : db.tracks) initial.insert(P(j++), track.p);

  {
    gttic_(regular);
    cout << "Optimizing Regular Graph..." << endl;
    optimize(db, graph, initial);
  }

  // 2. Build graph using BatchFactor
  // We batch by Point (Track). Each batch contains measurements from multiple
  // cameras for one point.
  NonlinearFactorGraph graphBatch;
  for (size_t j = 0; j < db.numberTracks(); j++) {
    std::map<Key, Point2> measurements;
    for (const SfmMeasurement& m : db.tracks[j].measurements) {
      measurements[C(m.first)] = m.second;
    }

    // BatchFactor(Vector)
    std::vector<SfmFactor, Eigen::aligned_allocator<SfmFactor>> factors;
    for (const auto& [key, z] : measurements) {
      // Correct order: (Camera, Point)
      factors.emplace_back(z, gNoiseModel, key, P(j));
    }
    auto batch =
        std::make_shared<BatchFactor<SfmFactor, 2>>(std::move(factors));
    // GTSAM_PRINT(*batch);
    // auto linearized = batch->linearize(initial);
    // cout << "Linearized batch factor " << j << endl;
    // GTSAM_PRINT(*linearized);

    graphBatch.add(batch);
  }

  {
    gttic_(batch);
    cout << "Optimizing Batch Graph..." << endl;
    optimize(db, graphBatch, initial);
  }
  return 0;
}
