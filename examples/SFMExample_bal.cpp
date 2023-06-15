/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample_bal.cpp
 * @brief   Solve a structure-from-motion problem from a "Bundle Adjustment in the Large" file
 * @author  Frank Dellaert
 */

// For an explanation of headers, see SFMExample.cpp
#include <gtsam/sfm/SfmData.h> // for loading BAL datasets !
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <vector>

using namespace std;
using namespace gtsam;
using symbol_shorthand::C;
using symbol_shorthand::P;

// We will be using a projection factor that ties a SFM_Camera to a 3D point.
// An SFM_Camera is defined in datase.h as a camera with unknown Cal3Bundler calibration
// and has a total of 9 free parameters
typedef GeneralSFMFactor<SfmCamera,Point3> MyFactor;

/* ************************************************************************* */
int main (int argc, char* argv[]) {

  // Find default file, but if an argument is given, try loading a file
  string filename = findExampleDataFile("dubrovnik-3-7-pre");
  if (argc>1) filename = string(argv[1]);

  // Load the SfM data from file
  SfmData mydata = SfmData::FromBalFile(filename);
  cout << "read " << mydata.numberTracks() << " tracks on " << mydata.numberCameras() << " cameras" << endl;

  // Create a factor graph
  NonlinearFactorGraph graph;

  // We share *one* noiseModel between all projection factors
  auto noise =
      noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Add measurements to the factor graph
  size_t j = 0;
  for(const SfmTrack& track: mydata.tracks) {
    for (const auto& [i, uv] : track.measurements) {
      graph.emplace_shared<MyFactor>(uv, noise, C(i), P(j)); // note use of shorthand symbols C and P
    }
    j += 1;
  }

  // Add a prior on pose x1. This indirectly specifies where the origin is.
  // and a prior on the position of the first landmark to fix the scale
  graph.addPrior(C(0), mydata.cameras[0],  noiseModel::Isotropic::Sigma(9, 0.1));
  graph.addPrior(P(0), mydata.tracks[0].p, noiseModel::Isotropic::Sigma(3, 0.1));

  // Create initial estimate
  Values initial;
  size_t i = 0; j = 0;
  for(const SfmCamera& camera: mydata.cameras) initial.insert(C(i++), camera);
  for(const SfmTrack& track: mydata.tracks)    initial.insert(P(j++), track.p);

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

