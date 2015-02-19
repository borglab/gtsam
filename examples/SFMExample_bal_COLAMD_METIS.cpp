/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample.cpp
 * @brief   This file is to compare the ordering performance for COLAMD vs METIS.
 * Example problem is to solve a structure-from-motion problem from a "Bundle Adjustment in the Large" file.
 * @author  Frank Dellaert, Zhaoyang Lv
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
using symbol_shorthand::C;
using symbol_shorthand::P;

// We will be using a projection factor that ties a SFM_Camera to a 3D point.
// An SFM_Camera is defined in datase.h as a camera with unknown Cal3Bundler calibration
// and has a total of 9 free parameters
typedef GeneralSFMFactor<SfM_Camera,Point3> MyFactor;

/* ************************************************************************* */
int main (int argc, char* argv[]) {

  // Find default file, but if an argument is given, try loading a file
  string filename = findExampleDataFile("dubrovnik-3-7-pre");
  if (argc>1) filename = string(argv[1]);

  // Load the SfM data from file
  SfM_data mydata;
  readBAL(filename, mydata);
  cout << boost::format("read %1% tracks on %2% cameras\n") % mydata.number_tracks() % mydata.number_cameras();

  // Create a factor graph
  NonlinearFactorGraph graph;

  // We share *one* noiseModel between all projection factors
  noiseModel::Isotropic::shared_ptr noise =
      noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Add measurements to the factor graph
  size_t j = 0;
  BOOST_FOREACH(const SfM_Track& track, mydata.tracks) {
    BOOST_FOREACH(const SfM_Measurement& m, track.measurements) {
      size_t i = m.first;
      Point2 uv = m.second;
      graph.push_back(MyFactor(uv, noise, C(i), P(j))); // note use of shorthand symbols C and P
    }
    j += 1;
  }

  // Add a prior on pose x1. This indirectly specifies where the origin is.
  // and a prior on the position of the first landmark to fix the scale
  graph.push_back(PriorFactor<SfM_Camera>(C(0), mydata.cameras[0],  noiseModel::Isotropic::Sigma(9, 0.1)));
  graph.push_back(PriorFactor<Point3>    (P(0), mydata.tracks[0].p, noiseModel::Isotropic::Sigma(3, 0.1)));

  // Create initial estimate
  Values initial;
  size_t i = 0; j = 0;
  BOOST_FOREACH(const SfM_Camera& camera, mydata.cameras) initial.insert(C(i++), camera);
  BOOST_FOREACH(const SfM_Track& track, mydata.tracks)    initial.insert(P(j++), track.p);

  /** ---------------------------------------------------**/

  /* With COLAMD, optimize the graph and print the results */
  cout << "Optimize with COLAMD..." << endl;

  Values result_COLAMD;
  try {
    double tic_t = clock();

    LevenbergMarquardtParams params_using_COLAMD;
    params_using_COLAMD.setVerbosity("ERROR");
    params_using_COLAMD.ordering = Ordering::Create(Ordering::COLAMD, graph);

    double toc_t = (clock() - tic_t)/CLOCKS_PER_SEC;

    tic_t = clock();

    LevenbergMarquardtOptimizer lm(graph, initial, params_using_COLAMD);
    result_COLAMD = lm.optimize();

    tic_t = clock();

    cout << "Ordering: " << toc_t << "seconds" << endl;
    cout << "Solving: "  << (clock() - tic_t)/CLOCKS_PER_SEC << "seconds" << endl;

  } catch (exception& e) {
    cout << e.what();
  }

  // To see the error, check SFMExample_bal.cpp file
  //cout << "final error: " << graph.error(result_COLAMD) << endl;

  /** ---------------------------------------------------**/

  /* with METIS, optimize the graph and print the results */
  cout << "Optimize with METIS" << endl;

  Values results_METIS;
  try {
    double tic_t = clock();

    LevenbergMarquardtParams params_using_METIS;
    params_using_METIS.setVerbosity("ERROR");
    params_using_METIS.ordering = Ordering::Create(Ordering::METIS, graph);

    double toc_t = (clock() - tic_t)/CLOCKS_PER_SEC;

    tic_t = clock();

    LevenbergMarquardtOptimizer lm(graph, initial, params_using_METIS);
    results_METIS = lm.optimize();

    tic_t = clock();

    cout << "Ordering: " << toc_t << "seconds" << endl;
    cout << "Solving: "  << (clock() - tic_t)/CLOCKS_PER_SEC << "seconds" << endl;

  } catch (exception& e) {
    cout << e.what();
  }



  return 0;
}
/* ************************************************************************* */

