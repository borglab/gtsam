/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SmartProjectionFactorTesting.cpp
 * @brief Example usage of SmartProjectionFactor using real datasets
 * @date August, 2013
 * @author Luca Carlone
 */

// Use a map to store landmark/smart factor pairs
#include <gtsam/base/FastMap.h>

// Both relative poses and recovered trajectory poses will be stored as Pose3 objects
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>

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
#include <gtsam/slam/dataset.h>
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

#define USE_BUNDLER

using symbol_shorthand::X;
using symbol_shorthand::L;

typedef PriorFactor<Pose3> Pose3Prior;
typedef FastMap<Key, int> OrderingMap;

#ifdef USE_BUNDLER
typedef SmartProjectionFactorsCreator<Pose3, Point3, Cal3Bundler> SmartFactorsCreator;
typedef GenericProjectionFactorsCreator<Pose3, Point3, Cal3Bundler> ProjectionFactorsCreator;
#else
typedef SmartProjectionFactorsCreator<Pose3, Point3, Cal3_S2> SmartFactorsCreator;
typedef GenericProjectionFactorsCreator<Pose3, Point3, Cal3_S2> ProjectionFactorsCreator;
#endif

bool debug = false;

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
  } // end of if on landmarks

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

  if (debug) {
    std::cout << "\n\n=================================================\n\n";
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

    LevenbergMarquardtOptimizer optimizer(graph, *graphValues, params);
    params.ordering = Ordering::COLAMD(graph);
    gttic_(SmartProjectionFactorExample_kitti);
    result = optimizer.optimize();
    gttoc_(SmartProjectionFactorExample_kitti);
    tictoc_finishedIteration_();

    std::cout << "Number of outer LM iterations: " << optimizer.state().iterations << std::endl;
    std::cout << "Total number of LM iterations (inner and outer): " << optimizer.getInnerIterations() << std::endl;

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

  // Set to true to use SmartProjectionFactor. Otherwise GenericProjectionFactor will be used
  bool useSmartProjectionFactor = true;
  bool doTriangulation = true; // we read points initial guess from file or we triangulate

  bool useLM = true; 
  bool addNoise = false;

  // Smart factors settings
  double linThreshold = -1.0; // negative is disabled
  double rankTolerance = 1.0;

  // Get home directory and default dataset
  string HOME = getenv("HOME");
  string datasetFile = HOME + "/data/SfM/BAL/Ladybug/problem-1031-110968-pre.txt";

  // --------------- READ USER INPUTS (main arguments) ------------------------------------
  if(argc>1){ // if we have any input arguments
    // Arg1: "smart" or "standard" (select if to use smart factors or standard projection factors)
    // Arg2: "triangulation=0" or "triangulation=1" (select whether to initialize initial guess for points using triangulation)
    // Arg3: name of the dataset, e.g., /home/aspn/data/SfM/BAL/Ladybug/problem-1031-110968-pre.txt
    string useSmartArgument = argv[1];
    string useTriangulationArgument = argv[2];
    datasetFile = argv[3];

    if(useSmartArgument.compare("smart")==0){
      useSmartProjectionFactor=true;
    } else{
      if(useSmartArgument.compare("standard")==0){
        useSmartProjectionFactor=false;
      }else{
        cout << "Selected wrong option for input argument - useSmartProjectionFactor" << endl;
        exit(1);
      }
    }
    if(useTriangulationArgument.compare("triangulation=1")==0){
      doTriangulation=true;
    } else{
      if(useTriangulationArgument.compare("triangulation=0")==0){
        doTriangulation=false;
      }else{
        cout << "Selected wrong option for input argument - doTriangulation - not important for SmartFactors" << endl;
      }
    }
  }

  // --------------- PRINT USER's CHOICE  ------------------------------------
  std::cout << "- useSmartFactor: " << useSmartProjectionFactor << std::endl;
  std::cout << "- doTriangulation: " << doTriangulation << std::endl;
  std::cout << "- datasetFile: " << datasetFile << std::endl;

  if (linThreshold >= 0)
    std::cout << "PARAM linThreshold (negative is disabled): " << linThreshold << std::endl;

  if(addNoise)
    std::cout << "PARAM Noise: " << addNoise << std::endl;

  // --------------- READ INPUT DATA  ----------------------------------------
  SfM_data inputData;
  readBAL(datasetFile, inputData);
  std::cout << "read data from file... " << std::endl;

  // --------------- CREATE FACTOR GRAPH  ------------------------------------
  static SharedNoiseModel pixel_sigma(noiseModel::Unit::Create(2));
  boost::shared_ptr<Ordering> ordering(new Ordering());

  NonlinearFactorGraph graphSmart;
  gtsam::Values::shared_ptr graphSmartValues(new gtsam::Values());

  NonlinearFactorGraph graphProjection;
  gtsam::Values::shared_ptr graphProjectionValues(new gtsam::Values());

#ifdef USE_BUNDLER
  std::vector< boost::shared_ptr<Cal3Bundler> > K_cameras;
  boost::shared_ptr<Cal3Bundler> K(new Cal3Bundler());
#else
  std::vector< boost::shared_ptr<Cal3_S2> > K_cameras;
  Cal3_S2::shared_ptr K(new Cal3_S2());
#endif

  SmartFactorsCreator smartCreator(pixel_sigma, K, rankTolerance, linThreshold); // this initial K is not used
  ProjectionFactorsCreator projectionCreator(pixel_sigma, K);  // this initial K is not used
  int numLandmarks=0;

  if(debug){
    std::cout << "Constructors for factor creators " << std::endl;
    std::cout << "inputData.number_cameras() " << inputData.number_cameras()  << std::endl;
    std::cout << "inputData.number_tracks() " << inputData.number_tracks()  << std::endl;
  }

  // Load graph values
  gtsam::Values::shared_ptr loadedValues(new gtsam::Values()); // values we read from file
  for (size_t i = 0; i < inputData.number_cameras(); i++){ // for each camera
    Pose3 cameraPose =  inputData.cameras[i].pose();
    if(addNoise){
      Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/100, 0., -M_PI/100), gtsam::Point3(0.3,0.1,0.3));
      cameraPose = cameraPose.compose(noise_pose);
    }
    loadedValues->insert(X(i), cameraPose);
    graphSmartValues->insert(X(i), cameraPose);
  }
  if(debug) std::cout << "Initialized values " << std::endl;

  for (size_t j = 0; j < inputData.number_tracks(); j++){ // for each 3D point
    Point3 point = inputData.tracks[j].p;
    loadedValues->insert(L(j), point);
  }
  if(debug) std::cout << "Initialized points " << std::endl;

  // Create the graph
  for (size_t j = 0; j < inputData.number_tracks(); j++){ // for each 3D point
    SfM_Track track  = inputData.tracks[j];
    Point3 point = track.p;

    for (size_t k = 0; k < track.number_measurements(); k++){ // for each measurement of the point
      SfM_Measurement measurement = track.measurements[k];
      int    i = measurement.first;
      double u = measurement.second.x();
      double v = measurement.second.y();
      boost::shared_ptr<Cal3Bundler> Ki(new Cal3Bundler(inputData.cameras[i].calibration()));
      //boost::shared_ptr<Cal3_S2> Ki(new Cal3_S2());

      // insert data in a format that can be understood
      if (useSmartProjectionFactor) {
        // Use smart factors
        smartCreator.add(L(j), X(i), Point2(u,v), pixel_sigma, Ki, graphSmart);
        numLandmarks = smartCreator.getNumLandmarks();
      } else {
        // or STANDARD PROJECTION FACTORS
        projectionCreator.add(L(j), X(i), Point2(u,v), pixel_sigma, Ki, graphProjection);
        numLandmarks = projectionCreator.getNumLandmarks();
      }
    }
  }
  if(debug) std::cout << "Included measurements in the graph " << std::endl;

  cout << "Number of landmarks  " << numLandmarks << endl;

  cout << "Before call to update: ------------------ " << endl;
  cout << "Poses in SmartGraph values: "<< graphSmartValues->size() << endl;
  Values valuesProjPoses = graphProjectionValues->filter<Pose3>();
  cout << "Poses in ProjectionGraph values: "<< valuesProjPoses.size() << endl;
  Values valuesProjPoints = graphProjectionValues->filter<Point3>();
  cout << "Points in ProjectionGraph values: "<< valuesProjPoints.size() << endl;
  cout << "---------------------------------------------------------- " << endl;

  if (!useSmartProjectionFactor) {
    projectionCreator.update(graphProjection, loadedValues, graphProjectionValues, doTriangulation);
    // graphProjectionValues = loadedValues;
    ordering = projectionCreator.getOrdering();
  }

  cout << "After call to update: ------------------ " << endl;
  cout << "Poses in SmartGraph values: "<< graphSmartValues->size() << endl;
  valuesProjPoses = graphProjectionValues->filter<Pose3>();
  cout << "Poses in ProjectionGraph values: "<< valuesProjPoses.size() << endl;
  valuesProjPoints = graphProjectionValues->filter<Point3>();
  cout << "Points in ProjectionGraph values: "<< valuesProjPoints.size() << endl;
  cout << "---------------------------------------------------------- " << endl;

  Values result;
  if (useSmartProjectionFactor) {
    if (useLM)
      optimizeGraphLM(graphSmart, graphSmartValues, result, ordering);
    else
      optimizeGraphISAM2(graphSmart, graphSmartValues, result);
    cout << "Final reprojection error (smart): " << graphSmart.error(result);
  } else {
    if (useLM)
      optimizeGraphLM(graphProjection, graphProjectionValues, result, ordering);
    else
      optimizeGraphISAM2(graphProjection, graphProjectionValues, result); // ?
    cout << "Final reprojection error (standard): " << graphProjection.error(result);
  }

  cout << "===================================================" << endl;
  tictoc_print_();
  cout << "===================================================" << endl;
  writeValues("./", result);

  exit(0);
}

