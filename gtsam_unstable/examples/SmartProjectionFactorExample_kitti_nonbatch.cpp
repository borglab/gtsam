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
#include <gtsam/nonlinear/Values.h>

// We will use a non-linear solver to batch-initialize from the first 150 frames
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics SLAM problems.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/SmartProjectionFactorsCreator.h>
#include <gtsam_unstable/slam/GenericProjectionFactorsCreator.h>

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
typedef SmartProjectionFactorsCreator<Pose3, Point3, Cal3_S2> SmartFactorsCreator;
typedef GenericProjectionFactorsCreator<Pose3, Point3, Cal3_S2> ProjectionFactorsCreator;
typedef FastMap<Key, int> OrderingMap;

bool debug = false;

//// Helper functions taken from VO code
// Loaded all pose values into list
Values::shared_ptr loadPoseValues(const string& filename) {

  Values::shared_ptr values(new Values());
  bool addNoise = false;
  std::cout << "PARAM Noise: " << addNoise << std::endl; 
  // Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/100, 0., -M_PI/100), gtsam::Point3(0.3,0.1,0.3));

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

// Write key values to file
void writeValues(string directory_, const Values& values){
  string filename = directory_ + "out_camera_poses.txt";
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

void optimizeGraphLM(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr graphValues, Values &result, boost::shared_ptr<Ordering> &ordering) {
  // Optimization parameters
  LevenbergMarquardtParams params;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;
  params.lambdaInitial = 1;
  params.lambdaFactor = 10;
  // Profile a single iteration
//  params.maxIterations = 1;
  params.maxIterations = 100;
  std::cout << " LM max iterations: " << params.maxIterations << std::endl;
  // // params.relativeErrorTol = 1e-5;
  params.absoluteErrorTol = 1.0;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;
  params.linearSolverType = SuccessiveLinearizationParams::MULTIFRONTAL_CHOLESKY;

  cout << "Graph size: " << graph.size() << endl;
  cout << "Number of variables: " << graphValues->size() << endl;
  std::cout << " OPTIMIZATION " << std::endl;

  std::cout << "\n\n=================================================\n\n";
  if (debug) {
    graph.print("thegraph");
  }
  std::cout << "\n\n=================================================\n\n";

  if (ordering && ordering->size() > 0) {
    if (debug) {
      std::cout << "Have an ordering\n" << std::endl;
      BOOST_FOREACH(const Key& key, *ordering) {
        std::cout << key << " ";
      }
      std::cout << std::endl;
    }

    params.ordering = *ordering;

    //for (int i = 0; i < 3; i++) {
      LevenbergMarquardtOptimizer optimizer(graph, *graphValues, params);
      gttic_(GenericProjectionFactorExample_kitti);
      result = optimizer.optimize();
      gttoc_(GenericProjectionFactorExample_kitti);
      tictoc_finishedIteration_();
    //}
  } else {
    std::cout << "Using COLAMD ordering\n" << std::endl;
    //boost::shared_ptr<Ordering> ordering2(new Ordering()); ordering = ordering2;

    //for (int i = 0; i < 3; i++) {
      LevenbergMarquardtOptimizer optimizer(graph, *graphValues, params);
      params.ordering = Ordering::COLAMD(graph);
      gttic_(SmartProjectionFactorExample_kitti);
      result = optimizer.optimize();
      gttoc_(SmartProjectionFactorExample_kitti);
      tictoc_finishedIteration_();
    //}

    //*ordering = params.ordering;
    if (params.ordering) {
        std::cout << "Graph size: " << graph.size() << " ORdering: " << params.ordering->size() << std::endl;
        ordering = boost::make_shared<Ordering>(*(new Ordering()));
        *ordering = *params.ordering;
    } else {
        std::cout << "WARNING: NULL ordering!" << std::endl;
    }
  }
}

void optimizeGraphGN(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr graphValues, Values &result) {
  GaussNewtonParams params;
  //params.maxIterations = 1;
  params.verbosity = NonlinearOptimizerParams::DELTA;

  GaussNewtonOptimizer optimizer(graph, *graphValues, params);
  gttic_(SmartProjectionFactorExample_kitti);
  result = optimizer.optimize();
  gttoc_(SmartProjectionFactorExample_kitti);
  tictoc_finishedIteration_();

}

void optimizeGraphISAM2(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr graphValues, Values &result) {
  ISAM2 isam;
  gttic_(SmartProjectionFactorExample_kitti);
  isam.update(graph, *graphValues);
  result = isam.calculateEstimate();
  gttoc_(SmartProjectionFactorExample_kitti);
  tictoc_finishedIteration_();
}

// main
int main(int argc, char** argv) {

  unsigned int maxNumLandmarks = 389007; // 10000; //100000000; // 309393 // (loop_closure_merged) //37106 //(reduced kitti);
  unsigned int maxNumPoses = 1e+6;

  // Set to true to use SmartProjectionFactor. Otherwise GenericProjectionFactor will be used
  bool useSmartProjectionFactor = false;
  bool useLM = true; 

  double KittiLinThreshold = -1.0; // 0.005; //
  double KittiRankTolerance = 1.0;

  bool incrementalFlag = false;
  int optSkip = 200; // we optimize the graph every optSkip poses

  std::cout << "PARAM SmartFactor: " << useSmartProjectionFactor << std::endl;
  std::cout << "PARAM LM: " << useLM << std::endl;
  std::cout << "PARAM KittiLinThreshold (negative is disabled): " << KittiLinThreshold << std::endl;

  // Get home directory and dataset
  string HOME = getenv("HOME");
  //string input_dir = HOME + "/data/KITTI_00_200/";
  string input_dir = HOME + "/data/kitti/loop_closures_merged/"; // 399997 landmarks, 4541 poses
  //string input_dir = HOME + "/data/kitti_00_full_dirty/";

  static SharedNoiseModel pixel_sigma(noiseModel::Unit::Create(2));
  NonlinearFactorGraph graphSmart, graphProjection;

  // Load calibration
  boost::shared_ptr<Cal3_S2> K = loadCalibration(input_dir+"calibration.txt");
  K->print("Calibration");

  // Read in kitti dataset
  ifstream fin;
  fin.open((input_dir+"stereo_factors.txt").c_str());
  if(!fin) {
    cerr << "Could not open stereo_factors.txt" << endl;
    exit(1);
  }

  // Load all values
  gtsam::Values::shared_ptr graphSmartValues(new gtsam::Values());
  gtsam::Values::shared_ptr graphProjectionValues(new gtsam::Values());
  gtsam::Values::shared_ptr loadedValues = loadPoseValues(input_dir+"camera_poses.txt");

  // read all measurements tracked by VO stereo
  cout << "Loading stereo_factors.txt" << endl;
  unsigned int count = 0;
  Key currentLandmark = 0;
  unsigned int numLandmarks = 0, numPoses = 0;
  Key r, l;
  double uL, uR, v, x, y, z;
  std::vector<Key> landmarkKeys, cameraPoseKeys;
  Values values;
  Values result;
  bool optimized = false;
  boost::shared_ptr<Ordering> ordering(new Ordering());
  bool breakingCondition;
  SmartFactorsCreator smartCreator(pixel_sigma, K, KittiRankTolerance, KittiLinThreshold);
  ProjectionFactorsCreator projectionCreator(pixel_sigma, K);

  // main loop: reads measurements and adds factors (also performs optimization if desired)
  // r >> pose id
  // l >> landmark id
  // (uL >> uR) >> measurement (xaxis pixel measurement in left and right camera - since we do monocular, we only use uL)
  // v >> measurement (yaxis pixel measurement)
  // (x >> y >> z) 3D initialization for the landmark (not used in this code)
  while (fin >> r >> l >> uL >> uR >> v >> x >> y >> z) {
    if (debug) fprintf(stderr,"Landmark %ld\n", l);
    if (debug) fprintf(stderr,"Line %d: %d landmarks, (max landmarks %d), %d poses, max poses %d\n", count, numLandmarks, maxNumLandmarks, numPoses, maxNumPoses);

    // 1: add values and factors to the graph
    // 1.1: add factors
    // SMART FACTORS ..
    if (useSmartProjectionFactor) {

      smartCreator.add(L(l), X(r), Point2(uL,v), graphSmart);
      numLandmarks = smartCreator.getNumLandmarks();

      // Add initial pose value if pose does not exist
      if (!graphSmartValues->exists<Pose3>(X(r)) && loadedValues->exists<Pose3>(X(r))) {
        graphSmartValues->insert(X(r), loadedValues->at<Pose3>(X(r)));
        numPoses++;
        optimized = false;
      }

    } else {
      // or STANDARD PROJECTION FACTORS
      projectionCreator.add(L(l), X(r), Point2(uL,v), graphProjection);
      numLandmarks = projectionCreator.getNumLandmarks();
      optimized = false;
    }

    breakingCondition = currentLandmark != l && (numPoses > maxNumPoses || numLandmarks > maxNumLandmarks);

    // Optimize if have a certain number of poses/landmarks, or we want to do incremental inference
    if (breakingCondition || (incrementalFlag && !optimized && ((numPoses+1) % optSkip)==0) ) {

      if (debug) fprintf(stderr,"%d: %d > %d, %d > %d\n", count, numLandmarks, maxNumLandmarks, numPoses, maxNumPoses);
      if (debug) cout << "Adding triangulated landmarks, graph size: " << graphProjection.size() << endl;

      if (useSmartProjectionFactor == false) {
          projectionCreator.update(graphProjection, loadedValues, graphProjectionValues);
          ordering = projectionCreator.getOrdering();
      }

      if (debug) cout << "Adding triangulated landmarks, graph size after: " << graphProjection.size() << endl;
      if (debug) fprintf(stderr,"%d: %d > %d, %d > %d\n", count, numLandmarks, maxNumLandmarks, numPoses, maxNumPoses);

      if (useSmartProjectionFactor) {
          if (useLM)
              optimizeGraphLM(graphSmart, graphSmartValues, result, ordering);
          else
              optimizeGraphISAM2(graphSmart, graphSmartValues, result);
      } else {
          if (useLM)
              optimizeGraphLM(graphProjection, graphProjectionValues, result, ordering);
          else
              optimizeGraphISAM2(graphSmart, graphSmartValues, result);
      }
      if(incrementalFlag) *graphSmartValues = result; // we use optimized solution as initial guess for the next one

      optimized = true;

    }

    if (debug) fprintf(stderr,"%d %d\n", count, maxNumLandmarks);
    if (debug) cout << "CurrentLandmark " << currentLandmark << " Landmark " << l << std::endl;

    if (debug) fprintf(stderr,"%d: %d, %d > %d, %d > %d\n", count, currentLandmark != l, numLandmarks, maxNumLandmarks, numPoses, maxNumPoses);
    if(breakingCondition){ // reached desired number of landmarks/poses
      break;
    }

    currentLandmark = l;
    count++;
    if(count==100000) {
      cout << "Loading graph smart... " << graphSmart.size() << endl;
      cout << "Loading graph projection... " << graphProjection.size() << endl;
    }

  } // end of while

  if (1||debug) fprintf(stderr,"%d: %d > %d, %d > %d\n", count, numLandmarks, maxNumLandmarks, numPoses, maxNumPoses);

  cout << "===================================================" << endl;
  //graphSmartValues->print("before optimization ");
  //result.print("results of kitti optimization ");
  tictoc_print_();
  cout << "===================================================" << endl;
  writeValues("./", result);

  if (1||debug) fprintf(stderr,"%d: %d > %d, %d > %d\n", count, numLandmarks, maxNumLandmarks, numPoses, maxNumPoses);
  exit(0);
}
