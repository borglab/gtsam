/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMBAL.cpp
 * @brief   time structure from motion with BAL file
 * @author  Frank Dellaert
 * @date    June 6, 2015
 */

#include <boost/foreach.hpp>
#include <gtsam/base/timing.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <stddef.h>
#include <stdexcept>
#include <string>

using namespace std;
using namespace gtsam;

//#define TERNARY

int main(int argc, char *argv[]) {
  typedef GeneralSFMFactor<PinholeCamera<Cal3Bundler>, Point3> sfmFactor;
  using symbol_shorthand::P;

  string defaultFilename = findExampleDataFile("dubrovnik-3-7-pre");
  SfM_data db;
  bool success = readBAL(argc>1 ? argv[1] : defaultFilename, db);
  if (!success) throw runtime_error("Could not access file!");

  SharedNoiseModel unit2 = noiseModel::Unit::Create(2);
  NonlinearFactorGraph graph;

  for (size_t j = 0; j < db.number_tracks(); j++) {
    BOOST_FOREACH (const SfM_Measurement& m, db.tracks[j].measurements)
      graph.push_back(sfmFactor(m.second, unit2, m.first, P(j)));
  }

  Values initial = initialCamerasAndPointsEstimate(db);

  LevenbergMarquardtOptimizer lm(graph, initial);

  Values actual = lm.optimize();
  tictoc_print_();

  return 0;
}
