/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SmartProjectionFactorExample_kitti.cpp
 * @brief Example usage of SmartProjectionFactor using real dataset
 * @date August, 2013
 * @author Zsolt Kira
 */

// Both relative poses and recovered trajectory poses will be stored as Pose2 objects
#include <gtsam/geometry/Pose3.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/nonlinear/Symbol.h>

// We want to use iSAM2 to solve the range-SLAM problem incrementally
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// We will use a non-liear solver to batch-inituialize from the first 150 frames
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics SLAM problems.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/SmartProjectionFactor.h>

// Standard headers, added last, so we know headers above work on their own
#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;
namespace NM = gtsam::noiseModel;

using symbol_shorthand::X;
using symbol_shorthand::L;

typedef PriorFactor<Pose3> Pose3Prior;

static SharedNoiseModel prior_model(noiseModel::Diagonal::Sigmas(Vector_(6, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)));

/* ************************************************************************* */
Values::shared_ptr loadPoseValues(const string& filename, list<Key> keys) {
  Values::shared_ptr values(new Values());
  std::list<Key>::iterator kit;

  // read in camera poses
  string full_filename = filename;
  ifstream fin;
  fin.open(full_filename.c_str());

  int pose_id;
  while (fin >> pose_id) {
    double pose_matrix[16];
    for (int i = 0; i < 16; i++) {
      fin >> pose_matrix[i];
    }
    kit = find (keys.begin(), keys.end(), X(pose_id));
    if (kit != keys.end()) {
      cout << " Adding " << X(pose_id) << endl;
      values->insert(Symbol('x',pose_id), Pose3(Matrix_(4, 4, pose_matrix)));
    }
  }

  fin.close();
  return values;
}

// main
int main(int argc, char** argv) {

  string HOME = getenv("HOME");
  string input_dir = HOME + "/data/kitti/loop_closures_merged/";
  Cal3_S2::shared_ptr K(new Cal3_S2(718.856, 718.856, 0.0, 607.1928, 185.2157));

  typedef SmartProjectionFactor<Pose3, Point3, Cal3_S2> SmartFactor;

  // Read in kitti dataset
  // load stereo factors and initialize landmarks when first seen
  ifstream fin;
  fin.open((input_dir+"stereo_factors.txt").c_str());
  if(!fin) {
    cerr << "Could not open stereo_factors.txt" << endl;
    exit(1);
  }

  static SharedNoiseModel pixel_sigma(noiseModel::Unit::Create(3));
  static SharedNoiseModel prior_model(noiseModel::Diagonal::Sigmas(Vector_(6, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)));
  NonlinearFactorGraph graph;

  // read all measurements tracked by VO
  cout << "Loading stereo_factors.txt" << endl;
  int count = 0;
  int currentLandmark = 0;
  int numLandmarks = 0;
  Key r, l;
  double uL, uR, v, x, y, z;
  std::list<Key> allViews;
  std::vector<Key> views;
  std::vector<Point2> measurements;
  Values values;
  bool updateGraph = false;
  while (fin >> r >> l >> uL >> uR >> v >> x >> y >> z) {
    cout << "CurrentLandmark " << currentLandmark << " Landmark " << l << std::endl;

    if (currentLandmark != l && views.size() < 3) {
      // New landmark.  Not enough views for previous landmark so move on.
      cout << "New landmark " << l << " with not enough view for previous one" << std::endl;
      currentLandmark = l;
      views.clear();
      measurements.clear();
    } else if (currentLandmark != l) {
      // New landmark.  Add previous landmark and associated views to new factor
      cout << "New landmark " << l << " with "<< views.size() << " views for previous landmark " << currentLandmark << std::endl;

      cout << "Keys ";
      BOOST_FOREACH(const Key& k, views) {
        allViews.push_back(k);
        cout << k << " ";
      }
      cout << endl;

      cout << "Measurements ";
      BOOST_FOREACH(const Point2& p, measurements) {
         cout << p << " ";
      }
      cout << endl;

      SmartFactor::shared_ptr smartFactor(new SmartFactor(measurements, pixel_sigma, views, K));
      numLandmarks++;
      graph.push_back(smartFactor);

      if (numLandmarks > 4) {
        updateGraph = true;
      }

      currentLandmark = l;
      views.clear();
      measurements.clear();
    } else {
      // We have new view for current landmark, so add it to the list later
      cout << "New view for landmark " << l << " (" << views.size() << " total)" << std::endl;
    }
    views += X(r);
    measurements += Point2(uL,v);

    // Optimize
    if (updateGraph) {

      cout << "Optimizing... " << endl;

      // Get all view in the graph and populate poses from VO output
      // TODO: Handle loop closures properly
      allViews.unique();
      cout << "All Keys ";
      values = *loadPoseValues(input_dir+"camera_poses.txt", allViews);
      BOOST_FOREACH(const Key& k, allViews) {
        cout << k << " ";
      }
      cout << endl;

      // Optimize!
      LevenbergMarquardtParams params;
      params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
      params.verbosity = NonlinearOptimizerParams::ERROR;

      Values result;
      gttic_(SmartProjectionFactorExample_kitti);
      LevenbergMarquardtOptimizer optimizer(graph, values, params);
      result = optimizer.optimize();
      gttoc_(SmartProjectionFactorExample_kitti);
      tictoc_finishedIteration_();

      values.print("before optimization ");
      result.print("results of kitti optimization ");
      tictoc_print_();

      updateGraph = false;
      values.clear();
      allViews.clear();
      graph = NonlinearFactorGraph();
    }

    count++;
    if(count==100000) {
      cout << "Loading graph... " << graph.size() << endl;
      count=0;
    }
  }

  cout << "Graph size: " << graph.size() << endl;

  exit(0);
}

