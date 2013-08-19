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
#include <gtsam/inference/Symbol.h>

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
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/slam/SmartProjectionFactor.h>

// Standard headers, added last, so we know headers above work on their own
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace boost::assign;
namespace NM = gtsam::noiseModel;

using symbol_shorthand::X;
using symbol_shorthand::L;

typedef PriorFactor<Pose3> Pose3Prior;

//// Helper functions taken from VO code
// Loaded all pose values into list
Values::shared_ptr loadPoseValues(const string& filename) {
  Values::shared_ptr values(new Values());
  bool addNoise = false;

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));

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

    if (addNoise) {
      values->insert(Symbol('x',pose_id), Pose3(Matrix_(4, 4, pose_matrix)).compose(noise_pose));
    } else {
      values->insert(Symbol('x',pose_id), Pose3(Matrix_(4, 4, pose_matrix)));
    }
  }

  fin.close();
  return values;
}

// Loaded specific pose values that are in key list
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

// Load calibration info
Cal3_S2::shared_ptr loadCalibration(const string& filename) {
  string full_filename = filename;
  ifstream fin;
  fin.open(full_filename.c_str());

  // try loading from parent directory as backup
  if(!fin) {
    cerr << "Could not load " << full_filename;
    exit(1);
  }

  double fx, fy, s, u, v, b;
  fin >> fx >> fy >> s >> u >> v >> b;
  fin.close();

  Cal3_S2::shared_ptr K(new Cal3_S2(fx, fy, s, u, v));
  return K;
}

// main
int main(int argc, char** argv) {

  bool debug = false;

  // Set to true to use SmartProjectionFactor.  Otherwise GenericProjectionFactor will be used
  bool useSmartProjectionFactor = true;

  // Minimum number of views of a landmark before it is added to the graph (SmartProjectionFactor case only)
  unsigned int minimumNumViews = 1;

  string HOME = getenv("HOME");
  //string input_dir = HOME + "/data/kitti/loop_closures_merged/";
  string input_dir = HOME + "/data/KITTI_00_200/";

  typedef SmartProjectionFactor<Pose3, Point3, Cal3_S2> SmartFactor;
  typedef GenericProjectionFactor<Pose3, Point3, Cal3_S2> ProjectionFactor;
  static SharedNoiseModel pixel_sigma(noiseModel::Unit::Create(2));
  static SharedNoiseModel prior_model(noiseModel::Diagonal::Sigmas(Vector_(6, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)));
  NonlinearFactorGraph graph;

  // Load calibration
  //Cal3_S2::shared_ptr K(new Cal3_S2(718.856, 718.856, 0.0, 607.1928, 185.2157));
  boost::shared_ptr<Cal3_S2> K = loadCalibration(input_dir+"calibration.txt");
  K->print("Calibration");

  // Load values from VO camera poses output
  gtsam::Values::shared_ptr loaded_values = loadPoseValues(input_dir+"camera_poses.txt");
  graph.push_back(Pose3Prior(X(0),loaded_values->at<Pose3>(X(0)), prior_model));
  graph.push_back(Pose3Prior(X(1),loaded_values->at<Pose3>(X(1)), prior_model));
  //graph.print("thegraph");

  // Read in kitti dataset
  ifstream fin;
  fin.open((input_dir+"stereo_factors.txt").c_str());
  if(!fin) {
    cerr << "Could not open stereo_factors.txt" << endl;
    exit(1);
  }

  // read all measurements tracked by VO stereo
  cout << "Loading stereo_factors.txt" << endl;
  int count = 0;
  Key currentLandmark = 0;
  int numLandmarks = 0;
  Key r, l;
  double uL, uR, v, x, y, z;
  std::list<Key> allViews;
  std::vector<Key> views;
  std::vector<Point2> measurements;
  Values values;
  while (fin >> r >> l >> uL >> uR >> v >> x >> y >> z) {
    if (debug) cout << "CurrentLandmark " << currentLandmark << " Landmark " << l << std::endl;

    if (useSmartProjectionFactor == false) {
      if (loaded_values->exists<Point3>(L(l)) == boost::none) {
        Pose3 camera = loaded_values->at<Pose3>(X(r));
        Point3 worldPoint = camera.transform_from(Point3(x, y, z));
        loaded_values->insert(L(l), worldPoint); // add point;
      }

      ProjectionFactor::shared_ptr projectionFactor(new ProjectionFactor(Point2(uL,v), pixel_sigma, X(r), L(l), K));
      graph.push_back(projectionFactor);
    }

    if (currentLandmark != l && views.size() < minimumNumViews) {
      // New landmark.  Not enough views for previous landmark so move on.
      if (debug) cout << "New landmark " << l << " with not enough view for previous one" << std::endl;
      currentLandmark = l;
      views.clear();
      measurements.clear();
    } else if (currentLandmark != l) {
      // New landmark.  Add previous landmark and associated views to new factor
      if (debug) cout << "New landmark " << l << " with "<< views.size() << " views for previous landmark " << currentLandmark << std::endl;

      if (debug) cout << "Keys ";
      BOOST_FOREACH(const Key& k, views) {
        allViews.push_back(k);
        if (debug) cout << k << " ";
      }
      if (debug) cout << endl;

      if (debug) {
        cout << "Measurements ";
        BOOST_FOREACH(const Point2& p, measurements) {
           cout << p << " ";
        }
        cout << endl;
      }

      if (useSmartProjectionFactor) {
        SmartFactor::shared_ptr smartFactor(new SmartFactor(measurements, pixel_sigma, views, K));
        graph.push_back(smartFactor);
      }

      numLandmarks++;

      currentLandmark = l;
      views.clear();
      measurements.clear();
    } else {
      // We have new view for current landmark, so add it to the list later
      if (debug) cout << "New view for landmark " << l << " (" << views.size() << " total)" << std::endl;
    }

    // Add view for new landmark
    views += X(r);
    measurements += Point2(uL,v);

    count++;
    if(count==100000) {
      cout << "Loading graph... " << graph.size() << endl;
      count=0;
    }
  }
  cout << "Graph size: " << graph.size() << endl;

  /*

  // If using only subset of graph, only read in values for keys that are used

  // Get all view in the graph and populate poses from VO output
  // TODO: Handle loop closures properly
  cout << "All Keys (" << allViews.size() << ")" << endl;
  allViews.unique();
  cout << "All Keys (" << allViews.size() << ")" << endl;

  values = *loadPoseValues(input_dir+"camera_poses.txt", allViews);
  BOOST_FOREACH(const Key& k, allViews) {
    if (debug) cout << k << " ";
  }
  cout << endl;
  */

  cout << "Optimizing... " << endl;

  // Optimize!
  LevenbergMarquardtParams params;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;

  params.lambdaInitial = 1;
  params.lambdaFactor = 10;
  params.maxIterations = 100;
  params.relativeErrorTol = 1e-5;
  params.absoluteErrorTol = 1.0;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;
  params.linearSolverType = SuccessiveLinearizationParams::MULTIFRONTAL_CHOLESKY;

  LevenbergMarquardtOptimizer optimizer(graph, *loaded_values, params);

  Values result;
  gttic_(SmartProjectionFactorExample_kitti);
  result = optimizer.optimize();
  gttoc_(SmartProjectionFactorExample_kitti);
  tictoc_finishedIteration_();

  cout << "===================================================" << endl;
  loaded_values->print("before optimization ");
  result.print("results of kitti optimization ");
  tictoc_print_();
  cout << "===================================================" << endl;

  exit(0);
}
