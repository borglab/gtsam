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

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/timing.h>

#include <boost/foreach.hpp>
#include <stddef.h>
#include <stdexcept>
#include <string>

using namespace std;
using namespace gtsam;

//#define TERNARY

int main(int argc, char* argv[]) {
  typedef GeneralSFMFactor<PinholeCamera<Cal3Bundler>, Point3> sfmFactor;
  using symbol_shorthand::P;

  // Load BAL file (default is tiny)
  string defaultFilename = findExampleDataFile("dubrovnik-3-7-pre");
  SfM_data db;
  bool success = readBAL(argc > 1 ? argv[1] : defaultFilename, db);
  if (!success) throw runtime_error("Could not access file!");

  // Build graph
  SharedNoiseModel unit2 = noiseModel::Unit::Create(2);
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.number_tracks(); j++) {
    BOOST_FOREACH (const SfM_Measurement& m, db.tracks[j].measurements)
      graph.push_back(sfmFactor(m.second, unit2, m.first, P(j)));
  }

  Values initial = initialCamerasAndPointsEstimate(db);

// Create Schur-complement ordering
#ifdef CCOLAMD
  vector<Key> pointKeys;
  for (size_t j = 0; j < db.number_tracks(); j++) pointKeys.push_back(P(j));
  Ordering ordering = Ordering::colamdConstrainedFirst(graph, pointKeys, true);
#else
  Ordering ordering;
  for (size_t j = 0; j < db.number_tracks(); j++) ordering.push_back(P(j));
  for (size_t i = 0; i < db.number_cameras(); i++) ordering.push_back(i);
#endif

  // Optimize
  LevenbergMarquardtParams params;
  params.setOrdering(ordering);
  params.setVerbosity("ERROR");
  params.setVerbosityLM("TRYLAMBDA");
  LevenbergMarquardtOptimizer lm(graph, initial, params);
  Values actual = lm.optimize();

  tictoc_finishedIteration_();
  tictoc_print_();

  return 0;
}
