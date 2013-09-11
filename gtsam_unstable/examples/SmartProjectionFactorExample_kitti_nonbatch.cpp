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
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>


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
typedef SmartProjectionFactor<Pose3, Point3, Cal3_S2> SmartFactor;
typedef GenericProjectionFactor<Pose3, Point3, Cal3_S2> ProjectionFactor;
typedef FastMap<Key, boost::shared_ptr<SmartProjectionFactorState> > SmartFactorToStateMap;
typedef FastMap<Key, boost::shared_ptr<SmartFactor> > SmartFactorMap;
typedef FastMap<Key, std::vector<boost::shared_ptr<ProjectionFactor> > > ProjectionFactorMap;

bool debug = false;

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

void addTriangulatedLandmarks(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr loadedValues,
    gtsam::Values::shared_ptr graphValues, boost::shared_ptr<Cal3_S2> K, ProjectionFactorMap &projectionFactors) {

  std::vector<boost::shared_ptr<ProjectionFactor> > projectionFactorVector;
  std::vector<boost::shared_ptr<ProjectionFactor> >::iterator vfit;
  std::vector<Key> keys;
  Point3 point;
  Pose3 cameraPose;

  ProjectionFactorMap::iterator pfit;

  // Iterate through all landmarks
  for (pfit = projectionFactors.begin(); pfit != projectionFactors.end(); pfit++) {
    projectionFactorVector = (*pfit).second;

    std::vector<Pose3> cameraPoses;
    std::vector<Point2> measured;

    // Iterate through projection factors
    for (vfit = projectionFactorVector.begin(); vfit != projectionFactorVector.end(); vfit++) {

      if (debug) std::cout << "ProjectionFactor: " << std::endl;
      if (debug) (*vfit)->print("ProjectionFactor");

      // Iterate through poses
      cameraPoses.push_back( loadedValues->at<Pose3>((*vfit)->key1() ) );
      measured.push_back( (*vfit)->measured() );

    }

    // Triangulate landmark based on set of poses and measurements
    if (debug) std::cout << "Triangulating: " << std::endl;
    try {
      point = triangulatePoint3(cameraPoses, measured, *K);
      if (1||debug) std::cout << "Triangulation succeeded: " << point << std::endl;
    } catch( TriangulationUnderconstrainedException& e) {
      if (1||debug) std::cout << "Triangulation failed because of unconstrained exception" << std::endl;
      BOOST_FOREACH(const Pose3& pose, cameraPoses) {
        std::cout << " Pose: " << pose << std::endl;
      }
      //exit(EXIT_FAILURE);
      continue;
    } catch( TriangulationCheiralityException& e) {
      if (1||debug) std::cout << "Triangulation failed because of cheirality exception" << std::endl;
      BOOST_FOREACH(const Pose3& pose, cameraPoses) {
        std::cout << " Pose: " << pose << std::endl;
      }
      //exit(EXIT_FAILURE);
      continue;
    }

    // Add projection factors and pose values
    for (vfit = projectionFactorVector.begin(); vfit != projectionFactorVector.end(); vfit++) {
      if (debug) std::cout << "Adding factor " << std::endl;
      if (debug) (*vfit)->print("Projection Factor");
      graph.push_back( (*vfit) );

      // TODO: Add poses to values here!
      if (!graphValues->exists<Pose3>( (*vfit)->key1()) && loadedValues->exists<Pose3>((*vfit)->key1())) {
        graphValues->insert((*vfit)->key1(), loadedValues->at<Pose3>((*vfit)->key1()));
      }
    }

    // Add landmark value
    if (debug) std::cout << "Adding value " << std::endl;
    graphValues->insert( projectionFactorVector[0]->key2(), point); // add point;


  }
}

void optimizeGraphLM(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr graphValues, Values &result) {
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

  cout << "Graph size: " << graph.size() << endl;
  std::cout << " OPTIMIZATION " << std::endl;

  std::cout << "\n\n=================================================\n\n";
  if (debug) {
     graph.print("thegraph");
   }
  std::cout << "\n\n=================================================\n\n";

  //for (int i = 0; i < 3; i++) {
    LevenbergMarquardtOptimizer optimizer(graph, *graphValues, params);
    gttic_(SmartProjectionFactorExample_kitti);
    result = optimizer.optimize();
    gttoc_(SmartProjectionFactorExample_kitti);
    tictoc_finishedIteration_();
  //}

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
// main
int main(int argc, char** argv) {

  unsigned int maxNumLandmarks = 37106; //1000000;
  unsigned int maxNumPoses = 200;

  // Set to true to use SmartProjectionFactor.  Otherwise GenericProjectionFactor will be used
  bool useSmartProjectionFactor = true;
  bool useTriangulation = false;
  bool useLM = true;

  std::cout << "PARAM SmartFactor: " << useSmartProjectionFactor << std::endl;
  std::cout << "PARAM Triangulation: " << useTriangulation << std::endl;
  std::cout << "PARAM LM: " << useLM << std::endl;

  // Get home directory and dataset
  string HOME = getenv("HOME");
  //string input_dir = HOME + "/data/kitti/loop_closures_merged/";
  string input_dir = HOME + "/data/KITTI_00_200/";

  static SharedNoiseModel pixel_sigma(noiseModel::Unit::Create(2));
  static SharedNoiseModel prior_model(noiseModel::Diagonal::Sigmas(Vector_(6, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)));
  NonlinearFactorGraph graph;

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
  SmartFactorToStateMap smartFactorStates;
  SmartFactorMap smartFactors;
  ProjectionFactorMap projectionFactors;
  Values result;
  while (fin >> r >> l >> uL >> uR >> v >> x >> y >> z) {
    if (debug) fprintf(stderr,"Landmark %ld\n", l);
    if (debug) fprintf(stderr,"Line %d: %d landmarks, (max landmarks %d), %d poses, max poses %d\n", count, numLandmarks, maxNumLandmarks, numPoses, maxNumPoses);

    // Optimize if have a certain number of poses/landmarks
    if (currentLandmark != l && (numPoses > maxNumPoses || numLandmarks > maxNumLandmarks) ) {

      if (1||debug) fprintf(stderr,"%d: %d > %d, %d > %d\n", count, numLandmarks, maxNumLandmarks, numPoses, maxNumPoses);

      cout << "Adding triangulated landmarks: " << graph.size() << endl;
      if (useTriangulation) {
        addTriangulatedLandmarks(graph, loadedValues, graphValues, K, projectionFactors);
      }
      cout << "Adding triangulated landmarks: " << graph.size() << endl;

      if (useLM)
        optimizeGraphLM(graph, graphValues, result);
      else
        optimizeGraphGN(graph, graphValues, result);

      // Only process first N measurements (for development/debugging)
      if ( (numPoses > maxNumPoses || numLandmarks > maxNumLandmarks) ) {
        if (1||debug) fprintf(stderr,"%d: BREAKING %d > %d, %d > %d\n", count, numLandmarks, maxNumLandmarks, numPoses, maxNumPoses);
        break;
      }
      break;
    }

    if (useSmartProjectionFactor) {

      // Check if landmark exists in mapping
      SmartFactorToStateMap::iterator fsit = smartFactorStates.find(L(l));
      SmartFactorMap::iterator fit = smartFactors.find(L(l));
      if (fsit != smartFactorStates.end() && fit != smartFactors.end()) {
        if (debug) fprintf(stderr,"Adding measurement to existing landmark\n");

        // Add measurement to smart factor
        (*fit).second->add(Point2(uL,v), X(r));

        if (debug) (*fit).second->print();
      } else {
        if (debug) fprintf(stderr,"New landmark (%d,%d)\n", fsit != smartFactorStates.end(), fit != smartFactors.end());

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

      }

      // Add initial pose value if pose does not exist
      if (!graphValues->exists<Pose3>(X(r)) && loadedValues->exists<Pose3>(X(r))) {
        graphValues->insert(X(r), loadedValues->at<Pose3>(X(r)));
        numPoses++;
      }

    } else {

      // Create projection factor
      ProjectionFactor::shared_ptr projectionFactor(new ProjectionFactor(Point2(uL,v), pixel_sigma, X(r), L(l), K));

      // Check if landmark exists in mapping
      ProjectionFactorMap::iterator pfit = projectionFactors.find(L(l));
      if (pfit != projectionFactors.end()) {
        if (debug) fprintf(stderr,"Adding measurement to existing landmark\n");

        // Add projection factor to list of projection factors associated with this landmark
        (*pfit).second.push_back(projectionFactor);

      } else {
        if (debug) fprintf(stderr,"New landmark (%d)\n", pfit != projectionFactors.end());

        // Create a new vector of projection factors
        std::vector<ProjectionFactor::shared_ptr> projectionFactorVector;
        projectionFactorVector.push_back(projectionFactor);

        // Insert projection factor to NEW list of projection factors associated with this landmark
        projectionFactors.insert( make_pair(L(l), projectionFactorVector) );

        // Add projection factor to graph
        //graph.push_back(projectionFactor);

        // We have a new landmark
        numLandmarks++;
      }

      // Add landmark if triangulation is not being used to initialize them
      if (!useTriangulation) {
        // For projection factor, landmarks positions are used, but have to be transformed to world coordinates
        if (graphValues->exists<Point3>(L(l)) == boost::none) {
          Pose3 camera = loadedValues->at<Pose3>(X(r));
          Point3 worldPoint = camera.transform_from(Point3(x, y, z));
          graphValues->insert(L(l), worldPoint); // add point;
        }

        // Add initial pose value if pose does not exist
        // Only do this if triangulation is not used.  Otherwise, it depends what projection factors are added
        // based on triangulation success
        if (!graphValues->exists<Pose3>(X(r)) && loadedValues->exists<Pose3>(X(r))) {
          graphValues->insert(X(r), loadedValues->at<Pose3>(X(r)));
          numPoses++;
        }

        // Add projection factor to graph
        graph.push_back(projectionFactor);


      } else {

        // Alternatively: Triangulate similar to how SmartProjectionFactor does it
        // We only do this at the end, when all of the camera poses are available
        // Note we do not add anything to the graph until then, since in some cases
        // of triangulation failure we cannot add the landmark to the graph

      }

    }

    if (debug) fprintf(stderr,"%d %d\n", count, maxNumLandmarks);
    if (debug) cout << "CurrentLandmark " << currentLandmark << " Landmark " << l << std::endl;

    currentLandmark = l;
    count++;
    if(count==100000) {
      cout << "Loading graph... " << graph.size() << endl;
    }
  }
 
  cout << "===================================================" << endl;
  graphValues->print("before optimization ");
  result.print("results of kitti optimization ");
  tictoc_print_();
  cout << "===================================================" << endl;
  writeValues("./", result);

  exit(0);
}
