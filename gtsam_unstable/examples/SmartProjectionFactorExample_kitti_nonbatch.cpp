/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SmartProjectionFactorExample_kitti.cpp
 * @brief Example usage of SmartProjectionFactor using real dataset in a non-batch fashion
 * @date August, 2013
 * @author Zsolt Kira
 */

// Use a map to store landmark/smart factor pairs
#include <gtsam/base/FastMap.h>

// Both relative poses and recovered trajectory poses will be stored as Pose3 objects
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
  std::cout << "PARAM Noise: " << addNoise << std::endl; 
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

// Load specific pose values that are in key list
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
      //cout << " Adding " << X(pose_id) << endl;
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

void writeValues(string directory_, const Values& values){
  string filename = directory_ + "camera_poses.txt";
  ofstream fout;
  fout.open(filename.c_str());
  fout.precision(20);

  // write out camera poses
  BOOST_FOREACH(Values::ConstFiltered<Pose3>::value_type key_value, values.filter<Pose3>()) {
    fout << Symbol(key_value.key).index();
    const gtsam::Matrix& matrix= key_value.value.matrix();
    for (size_t row=0; row < 4; ++row) {
      for (size_t col=0; col < 4; ++col) {
        fout << " " << matrix(row, col);
      }
    }
    fout << endl;
  }
  fout.close();

  if(values.filter<Point3>().size() > 0) {
    // write landmarks
    filename = directory_ + "landmarks.txt";
    fout.open(filename.c_str());

    BOOST_FOREACH(Values::ConstFiltered<Point3>::value_type key_value, values.filter<Point3>()) {
      fout << Symbol(key_value.key).index();
      fout << " " << key_value.value.x();
      fout << " " << key_value.value.y();
      fout << " " << key_value.value.z();
      fout << endl;
    }
    fout.close();

  }
}

// main
int main(int argc, char** argv) {

  bool debug = false;
  unsigned int maxNumLandmarks = 2000000;
  unsigned int maxNumPoses = 200000;

  // Set to true to use SmartProjectionFactor.  Otherwise GenericProjectionFactor will be used
  bool useSmartProjectionFactor = true;
  std::cout << "PARAM SmartFactor: " << useSmartProjectionFactor << std::endl;

  // Get home directory and dataset
  string HOME = getenv("HOME");
  //string input_dir = HOME + "/data/kitti/loop_closures_merged/";
  string input_dir = HOME + "/data/KITTI_00_200/";

  typedef SmartProjectionFactor<Pose3, Point3, Cal3_S2> SmartFactor;
  typedef GenericProjectionFactor<Pose3, Point3, Cal3_S2> ProjectionFactor;
  static SharedNoiseModel pixel_sigma(noiseModel::Unit::Create(2));
  static SharedNoiseModel prior_model(noiseModel::Diagonal::Sigmas(Vector_(6, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)));
  NonlinearFactorGraph graph;

  // Optimization parameters
  LevenbergMarquardtParams params;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;
  params.lambdaInitial = 1;
  params.lambdaFactor = 10;
  params.maxIterations = 100;
  //params.relativeErrorTol = 1e-5;
  params.absoluteErrorTol = 1.0;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;
  params.linearSolverType = SuccessiveLinearizationParams::MULTIFRONTAL_CHOLESKY;

  // Load calibration
  //Cal3_S2::shared_ptr K(new Cal3_S2(718.856, 718.856, 0.0, 607.1928, 185.2157));
  boost::shared_ptr<Cal3_S2> K = loadCalibration(input_dir+"calibration.txt");
  K->print("Calibration");

  // Read in kitti dataset
  ifstream fin;
  fin.open((input_dir+"stereo_factors.txt").c_str());
  if(!fin) {
    cerr << "Could not open stereo_factors.txt" << endl;
    exit(1);
  }

  // Load all values, add priors
  gtsam::Values::shared_ptr graphValues(new gtsam::Values());
  gtsam::Values::shared_ptr loadedValues = loadPoseValues(input_dir+"camera_poses.txt");
  graph.push_back(Pose3Prior(X(0),loadedValues->at<Pose3>(X(0)), prior_model));
  graph.push_back(Pose3Prior(X(1),loadedValues->at<Pose3>(X(1)), prior_model));

  // read all measurements tracked by VO stereo
  cout << "Loading stereo_factors.txt" << endl;
  unsigned int count = 0;
  Key currentLandmark = 0;
  unsigned int numLandmarks = 0, numPoses = 0;
  Key r, l;
  double uL, uR, v, x, y, z;
  std::vector<Key> views;
  std::vector<Point2> measurements;
  Values values;
  FastMap<Key, boost::shared_ptr<SmartProjectionFactorState> > smartFactorStates;
  FastMap<Key, boost::shared_ptr<SmartFactor> > smartFactors;
  Values result;
  while (fin >> r >> l >> uL >> uR >> v >> x >> y >> z) {
    fprintf(stderr,"Landmark %ld\n", l);
    fprintf(stderr,"Line %d: %d landmarks, (max landmarks %d), %d poses, max poses %d\n", count, numLandmarks, maxNumLandmarks, numPoses, maxNumPoses);

    if (currentLandmark != l && (numPoses > maxNumPoses || numLandmarks > maxNumLandmarks) ) { //numLandmarks > 3 && ) {
      cout << "Graph size: " << graph.size() << endl;
      graph.print("thegraph");
      std::cout << " OPTIMIZATION " << std::endl;
      LevenbergMarquardtOptimizer optimizer(graph, *graphValues, params);
      gttic_(SmartProjectionFactorExample_kitti);
      result = optimizer.optimize();
      gttoc_(SmartProjectionFactorExample_kitti);
      tictoc_finishedIteration_();

      // Only process first N measurements (for development/debugging)
      if ( (numPoses > maxNumPoses || numLandmarks > maxNumLandmarks) ) {
        fprintf(stderr,"BREAKING %d %d\n", count, maxNumLandmarks);
        break;
      }
    }

    // Check if landmark exists in mapping
    FastMap<Key, boost::shared_ptr<SmartProjectionFactorState> >::iterator fsit = smartFactorStates.find(L(l));
    FastMap<Key, boost::shared_ptr<SmartFactor> >::iterator fit = smartFactors.find(L(l));
    if (fsit != smartFactorStates.end() && fit != smartFactors.end()) {
      fprintf(stderr,"Adding measurement to existing landmark\n");

      // Add measurement to smart factor
      (*fit).second->add(Point2(uL,v), X(r));
      if (!graphValues->exists<Pose3>(X(r)) && loadedValues->exists<Pose3>(X(r))) {
        graphValues->insert(X(r), loadedValues->at<Pose3>(X(r)));
        numPoses++;
      }

      (*fit).second->print();
    } else {
      fprintf(stderr,"New landmark (%d,%d)\n", fsit != smartFactorStates.end(), fit != smartFactors.end());

      views += X(r);
      measurements += Point2(uL,v);

      // This is a new landmark, create a new factor and add to mapping
      boost::shared_ptr<SmartProjectionFactorState> smartFactorState(new SmartProjectionFactorState());
      SmartFactor::shared_ptr smartFactor(new SmartFactor(measurements, pixel_sigma, views, K));
      smartFactorStates.insert( make_pair(L(l), smartFactorState) );
      smartFactors.insert( make_pair(L(l), smartFactor) );
      graph.push_back(smartFactor);
      numLandmarks++;

      views.clear();
      measurements.clear();

      if (!graphValues->exists<Pose3>(X(r)) && loadedValues->exists<Pose3>(X(r))) {
        graphValues->insert(X(r), loadedValues->at<Pose3>(X(r)));
        numPoses++;
      }
    }

    fprintf(stderr,"%d %d\n", count, maxNumLandmarks);

    if (debug) cout << "CurrentLandmark " << currentLandmark << " Landmark " << l << std::endl;

    if (useSmartProjectionFactor == false) {
      // For projection factor, landmarks positions are used, but have to be transformed to world coordinates
      //if (loaded_values->exists<Point3>(L(l)) == boost::none) {
        //Pose3 camera = loaded_values->at<Pose3>(X(r));
        //Point3 worldPoint = camera.transform_from(Point3(x, y, z));
        //loaded_values->insert(L(l), worldPoint); // add point;
      //}

      ProjectionFactor::shared_ptr projectionFactor(new ProjectionFactor(Point2(uL,v), pixel_sigma, X(r), L(l), K));
      graph.push_back(projectionFactor);
    }

    currentLandmark = l;
    count++;
    if(count==100000) {
      cout << "Loading graph... " << graph.size() << endl;
    }
  }

  cout << "Graph size: " << graph.size() << endl;
  graph.print("thegraph");
  std::cout << " OPTIMIZATION " << std::endl;
  LevenbergMarquardtOptimizer optimizer(graph, *graphValues, params);
  gttic_(SmartProjectionFactorExample_kitti);
  result = optimizer.optimize();
  gttoc_(SmartProjectionFactorExample_kitti);
  tictoc_finishedIteration_();

  cout << "===================================================" << endl;
  graphValues->print("before optimization ");
  result.print("results of kitti optimization ");
  tictoc_print_();
  cout << "===================================================" << endl;
  writeValues("./", result);

  exit(0);
}
