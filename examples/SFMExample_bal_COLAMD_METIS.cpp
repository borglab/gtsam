/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample_bal_COLAMD_METIS.cpp
 * @brief   This file is to compare the ordering performance for COLAMD vs METIS.
 * Example problem is to solve a structure-from-motion problem from a "Bundle Adjustment in the Large" file.
 * @author  Frank Dellaert, Zhaoyang Lv
 */

// For an explanation of headers, see SFMExample.cpp
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/dataset.h>  // for loading BAL datasets !

#include <gtsam/base/timing.h>

#include <vector>

using namespace std;
using namespace gtsam;
using symbol_shorthand::C;
using symbol_shorthand::P;

// We will be using a projection factor that ties a SFM_Camera to a 3D point.
// An SFM_Camera is defined in datase.h as a camera with unknown Cal3Bundler
// calibration and has a total of 9 free parameters
typedef GeneralSFMFactor<SfmCamera, Point3> MyFactor;


int main(int argc, char* argv[]) {
  // Find default file, but if an argument is given, try loading a file
  string filename = findExampleDataFile("dubrovnik-3-7-pre");
  if (argc > 1) filename = string(argv[1]);

  // Load the SfM data from file
  SfmData mydata;
  readBAL(filename, mydata);
  cout << boost::format("read %1% tracks on %2% cameras\n") %
              mydata.number_tracks() % mydata.number_cameras();

  // Create a factor graph
  NonlinearFactorGraph graph;

  // We share *one* noiseModel between all projection factors
  auto noise = noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

  // Add measurements to the factor graph
  size_t j = 0;
  for (const SfmTrack& track : mydata.tracks) {
    for (const SfmMeasurement& m : track.measurements) {
      size_t i = m.first;
      Point2 uv = m.second;
      graph.emplace_shared<MyFactor>(
          uv, noise, C(i), P(j));  // note use of shorthand symbols C and P
    }
    j += 1;
  }

  // Add a prior on pose x1. This indirectly specifies where the origin is.
  // and a prior on the position of the first landmark to fix the scale
  graph.addPrior(C(0), mydata.cameras[0], noiseModel::Isotropic::Sigma(9, 0.1));
  graph.addPrior(P(0), mydata.tracks[0].p,
                 noiseModel::Isotropic::Sigma(3, 0.1));

  // Create initial estimate
  Values initial;
  size_t i = 0;
  j = 0;
  for (const SfmCamera& camera : mydata.cameras) initial.insert(C(i++), camera);
  for (const SfmTrack& track : mydata.tracks) initial.insert(P(j++), track.p);

  /** ---------------  COMPARISON  -----------------------**/
  /** ----------------------------------------------------**/

  LevenbergMarquardtParams params_using_COLAMD, params_using_METIS;
  try {
    params_using_METIS.setVerbosity("ERROR");
    gttic_(METIS_ORDERING);
    params_using_METIS.ordering = Ordering::Create(Ordering::METIS, graph);
    gttoc_(METIS_ORDERING);

    params_using_COLAMD.setVerbosity("ERROR");
    gttic_(COLAMD_ORDERING);
    params_using_COLAMD.ordering = Ordering::Create(Ordering::COLAMD, graph);
    gttoc_(COLAMD_ORDERING);
  } catch (exception& e) {
    cout << e.what();
  }

  // expect they have different ordering results
  if (params_using_COLAMD.ordering == params_using_METIS.ordering) {
    cout << "COLAMD and METIS produce the same ordering. "
         << "Problem here!!!" << endl;
  }

  /* Optimize the graph with METIS and COLAMD and time the results */

  Values result_METIS, result_COLAMD;
  try {
    gttic_(OPTIMIZE_WITH_METIS);
    LevenbergMarquardtOptimizer lm_METIS(graph, initial, params_using_METIS);
    result_METIS = lm_METIS.optimize();
    gttoc_(OPTIMIZE_WITH_METIS);

    gttic_(OPTIMIZE_WITH_COLAMD);
    LevenbergMarquardtOptimizer lm_COLAMD(graph, initial, params_using_COLAMD);
    result_COLAMD = lm_COLAMD.optimize();
    gttoc_(OPTIMIZE_WITH_COLAMD);
  } catch (exception& e) {
    cout << e.what();
  }

  {  // printing the result

    cout << "COLAMD final error: " << graph.error(result_COLAMD) << endl;
    cout << "METIS final error: " << graph.error(result_METIS) << endl;

    cout << endl << endl;

    cout << "Time comparison by solving " << filename << " results:" << endl;
    cout << boost::format("%1% point tracks and %2% cameras\n") %
                mydata.number_tracks() % mydata.number_cameras()
         << endl;

    tictoc_print_();
  }

  return 0;
}

