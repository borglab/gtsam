/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample.cpp
 * @brief   Solve a structure-from-motion problem from a "Bundle Adjustment in the Large" file
 * @author  Frank Dellaert
 */

// For an explanation of headers, see SFMExample.cpp
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/dataset.h> // for loading BAL datasets !
#include <vector>

using namespace std;
using namespace gtsam;

// We will be using a projection factor that ties a SFM_Camera to a 3D point.
// An SFM_Camera is defined in datase.h as a camera with unknown Cal3Bundler calibration
// and has a total of 9 free parameters
typedef GeneralSFMFactor<SfM_Camera,Point3> MyFactor;

/* ************************************************************************* */
int main (int argc, char* argv[]) {

  // default file
  string filename = findExampleDataFile("dubrovnik-3-7-pre");

  // If an argument is given, try loading a file
  if (argc>1) filename = string(argv[1]);

  ///< The structure where we will save the SfM data
  SfM_data mydata;
  assert(readBAL(filename, mydata));

  // Create a factor graph
  NonlinearFactorGraph graph;

  // Define the camera observation noise model
  noiseModel::Isotropic::shared_ptr noise =
      noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Add measurements to the factor graph
  size_t j = 0;
  BOOST_FOREACH(const SfM_Track& track, mydata.tracks) {
    BOOST_FOREACH(const SfM_Measurement& measurement, track.measurements) {
      size_t i; Point2 uv;
      boost::tie(i, uv) = measurement;
      graph.push_back(MyFactor(uv, noise, Symbol('x', i), Symbol('p', j)));
    }
    j += 1;
  }

  // Add a prior on pose x1. This indirectly specifies where the origin is.
  graph.push_back(
      PriorFactor<SfM_Camera>(Symbol('x', 0), mydata.cameras[0],
          noiseModel::Isotropic::Sigma(9, 0.1)));

  // Add a prior on the position of the first landmark to fix the scale
  graph.push_back(
      PriorFactor<Point3>(Symbol('p', 0), mydata.tracks[0].p,
          noiseModel::Isotropic::Sigma(3, 0.1)));

  // Create the data structure to hold the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initial;
  size_t i = 0;
  BOOST_FOREACH(const SfM_Camera& camera, mydata.cameras)
    initial.insert(Symbol('x', i++), camera);
  j = 0;
  BOOST_FOREACH(const SfM_Track& track, mydata.tracks)
    initial.insert(Symbol('p', j++), track.p);

  /* Optimize the graph and print results */
  Values result;
  try {
    LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");
    LevenbergMarquardtOptimizer lm(graph, initial, params);
    result = lm.optimize();
  } catch (exception& e) {
    cout << e.what();
  }
  cout << "final error: " << graph.error(result) << endl;

  return 0;
}
/* ************************************************************************* */

