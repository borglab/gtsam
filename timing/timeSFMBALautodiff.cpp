/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMBALautodiff.cpp
 * @brief   time SFM with BAL file, Ceres autodiff version
 * @author  Frank Dellaert
 * @date    July 5, 2015
 */

#include <gtsam/3rdparty/ceres/example.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/AdaptAutoDiff.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <stddef.h>

#include <stdexcept>
#include <string>

#include "timeSFMBAL.h"

using namespace std;
using namespace gtsam;
using symbol_shorthand::C;
using symbol_shorthand::P;
namespace bal = gtsam::timing::bal;

// See http://www.cs.cornell.edu/~snavely/bundler/bundler-v0.3-manual.html
// as to why so much gymnastics is needed to massage the initial estimates and
// measurements: basically, Snavely does not use computer vision conventions
// but OpenGL conventions :-(

typedef PinholeCamera<Cal3Bundler> Camera;

int main(int argc, char* argv[]) {
  // parse options and read BAL file
  bal::BalBenchmarkInput input = bal::parseSmallBenchmark(argc, argv);
  const SfmData& db = input.data;

  AdaptAutoDiff<SnavelyProjection, 2, 9, 3> snavely;

  // Build graph
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.numberTracks(); j++) {
    for (const SfmMeasurement& m : db.tracks[j].measurements) {
      size_t i = m.first;
      Point2 z = m.second;
      Expression<Vector9> camera_(C(i));
      Expression<Vector3> point_(P(j));
      // Expects measurements in OpenGL format, with y increasing upwards
      graph.addExpressionFactor(input.config.projectionNoise,
                                Vector2(z.x(), -z.y()),
                                Expression<Vector2>(snavely, camera_, point_));
    }
  }

  Values initial;
  size_t i = 0, j = 0;
  for (const SfmCamera& camera : db.cameras) {
    // SfmData::FromBalFile converts to GTSAM format, so we need to convert back
    // !
    Pose3 openGLpose = gtsam2openGL(camera.pose());
    Vector9 v9;
    v9 << Pose3::Logmap(openGLpose), camera.calibration();
    initial.insert(C(i++), v9);
  }
  for (const SfmTrack& track : db.tracks) {
    Vector3 v3 = track.p;
    initial.insert(P(j++), v3);
  }

  return bal::optimize(db, graph, initial, input.config);
}
