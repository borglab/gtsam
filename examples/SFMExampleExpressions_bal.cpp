/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExampleExpressions_bal.cpp
 * @brief   A structure-from-motion example done with Expressions
 * @author  Frank Dellaert
 * @date    January 2015
 */

/**
 * This is the Expression version of SFMExample
 * See detailed description of headers there, this focuses on explaining the AD part
 */

// The two new headers that allow using our Automatic Differentiation Expression framework
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

// Header order is close to far
#include <gtsam/sfm/SfmData.h>  // for loading BAL datasets !
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <boost/format.hpp>
#include <vector>

using namespace std;
using namespace gtsam;
using namespace noiseModel;
using symbol_shorthand::C;
using symbol_shorthand::P;

// An SfmCamera is defined in datase.h as a camera with unknown Cal3Bundler calibration
// and has a total of 9 free parameters

int main(int argc, char* argv[]) {
  // Find default file, but if an argument is given, try loading a file
  string filename = findExampleDataFile("dubrovnik-3-7-pre");
  if (argc > 1) filename = string(argv[1]);

  // Load the SfM data from file
  SfmData mydata = SfmData::FromBalFile(filename);
  cout << boost::format("read %1% tracks on %2% cameras\n") %
              mydata.numberTracks() % mydata.numberCameras();

  // Create a factor graph
  ExpressionFactorGraph graph;

  // Here we don't use a PriorFactor but directly the ExpressionFactor class
  // First, we create an expression to the pose from the first camera
  Expression<SfmCamera> camera0_(C(0));
  // Then, to get its pose:
  Pose3_ pose0_(&SfmCamera::getPose, camera0_);
  // Finally, we say it should be equal to first guess
  graph.addExpressionFactor(pose0_, mydata.cameras[0].pose(),
                            noiseModel::Isotropic::Sigma(6, 0.1));

  // similarly, we create a prior on the first point
  Point3_ point0_(P(0));
  graph.addExpressionFactor(point0_, mydata.tracks[0].p,
                            noiseModel::Isotropic::Sigma(3, 0.1));

  // We share *one* noiseModel between all projection factors
  auto noise = noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

  // Simulated measurements from each camera pose, adding them to the factor
  // graph
  size_t j = 0;
  for (const SfmTrack& track : mydata.tracks) {
    // Leaf expression for j^th point
    Point3_ point_('p', j);
    for (const auto& [i, uv] : track.measurements) {
      // Leaf expression for i^th camera
      Expression<SfmCamera> camera_(C(i));
      // Below an expression for the prediction of the measurement:
      Point2_ predict_ = project2<SfmCamera>(camera_, point_);
      // Again, here we use an ExpressionFactor
      graph.addExpressionFactor(predict_, uv, noise);
    }
    j += 1;
  }

  // Create initial estimate
  Values initial;
  size_t i = 0;
  j = 0;
  for (const SfmCamera& camera : mydata.cameras) initial.insert(C(i++), camera);
  for (const SfmTrack& track : mydata.tracks) initial.insert(P(j++), track.p);

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
