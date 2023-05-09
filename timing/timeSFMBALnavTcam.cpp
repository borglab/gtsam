/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMBALnavTcam.cpp
 * @brief   time SFM with BAL file, expressions with navTcam pose
 * @author  Frank Dellaert
 * @date    July 5, 2015
 */

#include "timeSFMBAL.h"

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Point3.h>


using namespace std;
using namespace gtsam;

int main(int argc, char* argv[]) {
  // parse options and read BAL file
  SfmData db = preamble(argc, argv);

  // Build graph using conventional GeneralSFMFactor
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.number_tracks(); j++) {
    Point3_ nav_point_(P(j));
    for (const SfmMeasurement& m: db.tracks[j].measurements) {
      size_t i = m.first;
      Point2 z = m.second;
      Pose3_ navTcam_(C(i));
      Cal3Bundler_ calibration_(K(i));
      graph.addExpressionFactor(
          gNoiseModel, z,
          uncalibrate(calibration_,
                      project(transformTo(navTcam_, nav_point_))));
    }
  }

  Values initial;
  size_t i = 0, j = 0;
  for (const SfmCamera& camera: db.cameras) {
    initial.insert(C(i), camera.pose());
    initial.insert(K(i), camera.calibration());
    i += 1;
  }
  for (const SfmTrack& track: db.tracks)
    initial.insert(P(j++), track.p);

  bool separateCalibration = true;
  return optimize(db, graph, initial, separateCalibration);
}
