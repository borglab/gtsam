/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMBALautodiff.cpp
 * @brief   time structure from motion with BAL file, Ceres autodiff version
 * @author  Frank Dellaert
 * @date    July 5, 2015
 */

#include "timeSFMBAL.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/AdaptAutoDiff.h>
#include <gtsam/3rdparty/ceres/example.h>

#include <boost/foreach.hpp>
#include <stddef.h>
#include <stdexcept>
#include <string>

using namespace std;
using namespace gtsam;

// See http://www.cs.cornell.edu/~snavely/bundler/bundler-v0.3-manual.html
// Special version of Cal3Bundler so that default constructor = 0,0,0
// This is only used in localCoordinates below
struct CeresCalibration : public Cal3Bundler {
  CeresCalibration(double f = 0, double k1 = 0, double k2 = 0, double u0 = 0,
                   double v0 = 0)
      : Cal3Bundler(f, k1, k2, u0, v0) {}
  CeresCalibration(const Cal3Bundler& cal) : Cal3Bundler(cal) {}
  CeresCalibration retract(const Vector& d) const {
    return CeresCalibration(fx() + d(0), k1() + d(1), k2() + d(2), u0(), v0());
  }
  Vector3 localCoordinates(const CeresCalibration& T2) const {
    return T2.vector() - vector();
  }
};

namespace gtsam {
template <>
struct traits<CeresCalibration> : public internal::Manifold<CeresCalibration> {
};
}

// With that, camera below behaves like Snavely's 9-dim vector
typedef PinholeCamera<CeresCalibration> Camera;

int main(int argc, char* argv[]) {
  // parse options and read BAL file
  SfM_data db = preamble(argc, argv);

  AdaptAutoDiff<SnavelyProjection, 2, 9, 3> snavely;

  // Build graph
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.number_tracks(); j++) {
    BOOST_FOREACH (const SfM_Measurement& m, db.tracks[j].measurements) {
      size_t i = m.first;
      Point2 z = m.second;
      Expression<Vector9> camera_(C(i));
      Expression<Vector3> point_(P(j));
      // Expects measurements in OpenGL format, with y increasing upwards
      graph.addExpressionFactor(gNoiseModel, Vector2(z.x(), -z.y()),
                                Expression<Vector2>(snavely, camera_, point_));
    }
  }

  Values initial;
  size_t i = 0, j = 0;
  BOOST_FOREACH (const SfM_Camera& camera, db.cameras) {
    // readBAL converts to GTSAM format, so we need to convert back !
    Camera ceresCamera(gtsam2openGL(camera.pose()), camera.calibration());
    Vector9 v9 = Camera().localCoordinates(ceresCamera);
    initial.insert(C(i++), v9);
  }
  BOOST_FOREACH (const SfM_Track& track, db.tracks) {
    Vector3 v3 = track.p.vector();
    initial.insert(P(j++), v3);
  }

  return optimize(db, graph, initial);
}
