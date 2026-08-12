/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMBALcamTnav.cpp
 * @brief   time SFM with BAL file, expressions with camTnav pose
 * @author  Frank Dellaert
 * @date    July 5, 2015
 */

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/slam/expressions.h>

#include "timeSFMBAL.h"

using namespace std;
using namespace gtsam;
using symbol_shorthand::C;
using symbol_shorthand::K;
using symbol_shorthand::P;
namespace bal = gtsam::timing::bal;

int main(int argc, char* argv[]) {
  // parse options and read BAL file
  bal::BalBenchmarkInput input = bal::parseSmallBenchmark(argc, argv);
  const SfmData& db = input.data;

  // Build graph using conventional GeneralSFMFactor
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.numberTracks(); j++) {
    for (const SfmMeasurement& m : db.tracks[j].measurements) {
      size_t i = m.first;
      Point2 z = m.second;
      Pose3_ camTnav_(C(i));
      Cal3Bundler_ calibration_(K(i));
      Point3_ nav_point_(P(j));
      graph.addExpressionFactor(
          input.config.projectionNoise, z,
          uncalibrate(calibration_,  // now using transformFrom !!!:
                      project(transformFrom(camTnav_, nav_point_))));
    }
  }

  Values initial;
  size_t i = 0, j = 0;
  for (const SfmCamera& camera : db.cameras) {
    initial.insert(C(i), camera.pose().inverse());  // inverse !!!
    initial.insert(K(i), camera.calibration());
    i += 1;
  }
  for (const SfmTrack& track : db.tracks) initial.insert(P(j++), track.p);

  bool separateCalibration = true;
  return bal::optimize(db, graph, initial, input.config, separateCalibration);
}
