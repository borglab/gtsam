/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample_SmartFactor_bal.cpp
 * @brief   Solve a BAL structure-from-motion problem using smart projection factors.
 */

#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/SmartProjectionFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <vector>

using namespace std;
using namespace gtsam;
using symbol_shorthand::C;

// Smart factor on full SfmCamera state (pose + calibration).
typedef SmartProjectionFactor<SfmCamera> SmartFactor;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  string filename = findExampleDataFile("dubrovnik-3-7-pre");
  if (argc > 1) filename = string(argv[1]);

  SfmData mydata = SfmData::FromBalFile(filename);
  cout << "read " << mydata.numberTracks() << " tracks on " << mydata.numberCameras() << " cameras" << endl;

  if (mydata.numberCameras() < 2) {
    cerr << "Need at least 2 cameras for this example." << endl;
    return 1;
  }

  NonlinearFactorGraph graph;
  auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

  for (const SfmTrack& track : mydata.tracks) {
    SmartFactor::shared_ptr smart(new SmartFactor(measurementNoise));
    for (const auto& [i, uv] : track.measurements) {
      smart->add(uv, C(i));
    }
    graph.push_back(smart);
  }

  // Fix gauge/scale with priors on two cameras.
  graph.addPrior(C(0), mydata.cameras[0], noiseModel::Isotropic::Sigma(9, 0.1));
  graph.addPrior(C(1), mydata.cameras[1], noiseModel::Isotropic::Sigma(9, 0.1));

  Values initial;
  size_t i = 0;
  for (const SfmCamera& camera : mydata.cameras) initial.insert(C(i++), camera);

  Values result;
  size_t iterations = 0;
  try {
    LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");
    LevenbergMarquardtOptimizer lm(graph, initial, params);
    result = lm.optimize();
    iterations = lm.iterations();
  } catch (exception& e) {
    cout << e.what() << endl;
    return 1;
  }

  cout << "final error: " << graph.error(result) << endl;
  cout << "iterations: " << iterations << endl;
  return 0;
}
/* ************************************************************************* */

