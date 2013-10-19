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

using symbol_shorthand::X;
using symbol_shorthand::L;

typedef PriorFactor<Pose3> Pose3Prior;
typedef FastMap<Key, int> OrderingMap;

typedef SmartProjectionFactorsCreator<Pose3, Point3, Cal3Bundler> SmartFactorsCreator;
typedef GenericProjectionFactorsCreator<Pose3, Point3, Cal3Bundler> ProjectionFactorsCreator;

bool debug = false;


void optimizeGraphLM(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr graphValues, Values &result, boost::shared_ptr<Ordering> &ordering) {
  // Optimization parameters
  LevenbergMarquardtParams params;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;
  params.lambdaInitial = 1;
  // Other parameters: if needed
  // params.lambdaFactor = 10;
  // Profile a single iteration
  // params.maxIterations = 1;
  // params.relativeErrorTol = 1e-5;
  // params.absoluteErrorTol = 1.0;
  cout << "==================== Optimization ==================" << endl;
  cout << "- Number of factors: " << graph.size() << endl;
  cout << "- Number of variables: " << graphValues->size() << endl;

  params.print("PARAMETERS FOR LM: \n");

  if (debug) {
    cout << "\n\n===============================================\n\n";
    graph.print("thegraph");
  }
  cout << "-----------------------------------------------------" << endl;

  if (ordering && ordering->size() > 0) {
    std::cout << "Starting graph optimization with user specified ordering" << std::endl;
    params.ordering = *ordering;
    LevenbergMarquardtOptimizer optimizer(graph, *graphValues, params);
    gttic_(GenericProjectionFactorExample_kitti);
    result = optimizer.optimize();
    gttoc_(GenericProjectionFactorExample_kitti);
    tictoc_finishedIteration_();
    cout << "-----------------------------------------------------" << endl;
    std::cout << "Number of outer LM iterations: " << optimizer.state().iterations << std::endl;
    std::cout << "Total number of LM iterations (inner and outer): " << optimizer.getInnerIterations() << std::endl;

  } else {
    std::cout << "Starting graph optimization with COLAMD ordering" << std::endl;
    LevenbergMarquardtOptimizer optimizer(graph, *graphValues, params);
    params.ordering = Ordering::COLAMD(graph);
    gttic_(smartProjectionFactorExample);
    result = optimizer.optimize();
    gttoc_(smartProjectionFactorExample);
    tictoc_finishedIteration_();
    cout << "-----------------------------------------------------" << endl;
    std::cout << "Number of outer LM iterations: " << optimizer.state().iterations << std::endl;
    std::cout << "Total number of LM iterations (inner and outer): " << optimizer.getInnerIterations() << std::endl;

    //*ordering = params.ordering;
    if (params.ordering) {
      if(debug) std::cout << "Graph size: " << graph.size() << " Ordering: " << params.ordering->size() << std::endl;
      ordering = boost::make_shared<Ordering>(*(new Ordering()));
      *ordering = *params.ordering;
    } else {
      std::cout << "WARNING: NULL ordering!" << std::endl;
    }
  }
  cout << "======================================================" << endl;
}

void optimizeGraphGN(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr graphValues, Values &result) {
  GaussNewtonParams params;
  //params.maxIterations = 1;
  params.verbosity = NonlinearOptimizerParams::DELTA;

  GaussNewtonOptimizer optimizer(graph, *graphValues, params);
  gttic_(smartProjectionFactorExample);
  result = optimizer.optimize();
  gttoc_(smartProjectionFactorExample);
  tictoc_finishedIteration_();
}

void optimizeGraphISAM2(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr graphValues, Values &result) {
  ISAM2 isam;
  gttic_(smartProjectionFactorExample);
  isam.update(graph, *graphValues);
  result = isam.calculateEstimate();
  gttoc_(smartProjectionFactorExample);
  tictoc_finishedIteration_();
}

// ************************************************************************************************
// ************************************************************************************************
// main
int main(int argc, char** argv) {

  bool useSmartProjectionFactor = true; // default choice is to use the smart projection factors
  bool doTriangulation = true; // default choice is to initialize points from triangulation (only for standard projection factors)

  bool addNoise = false; // add (fixed) noise to the initial guess of camera poses
  bool useLM = true; 

  // Smart factors settings
  double linThreshold = -1.0; // negative is disabled
  double rankTolerance = 1.0;

  // Get home directory and default dataset
  string HOME = getenv("HOME");
  string datasetFile = HOME + "/data/SfM/BAL/Ladybug/problem-1031-110968-pre.txt";

  // --------------- READ USER INPUTS (main arguments) ------------------------------------
  // COMMAND TO RUN (EXAMPLE): ./SmartProjectionFactorExampleBAL smart triangulation=0 /home/aspn/data/SfM/BAL/Ladybug/problem-1031-110968-pre.txt
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
    std::cout << "- linThreshold (negative is disabled): " << linThreshold << std::endl;

  if(addNoise)
    std::cout << "- Noise: " << addNoise << std::endl;

  // --------------- READ INPUT DATA  ----------------------------------------
  std::cout << "- reading dataset from file... " << std::endl;
  SfM_data inputData;
  readBAL(datasetFile, inputData);

  // --------------- CREATE FACTOR GRAPH  ------------------------------------
  std::cout << "- creating factor graph... " << std::endl;
  static SharedNoiseModel pixel_sigma(noiseModel::Unit::Create(2)); // pixel noise
  boost::shared_ptr<Ordering> ordering(new Ordering());

  NonlinearFactorGraph graphSmart;
  gtsam::Values::shared_ptr graphSmartValues(new gtsam::Values());

  NonlinearFactorGraph graphProjection;
  gtsam::Values::shared_ptr graphProjectionValues(new gtsam::Values());

  std::vector< boost::shared_ptr<Cal3Bundler> > K_cameras;
  boost::shared_ptr<Cal3Bundler> K(new Cal3Bundler());

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
    loadedValues->insert(X(i), cameraPose); // this will be used for the graphProjection
    graphSmartValues->insert(X(i), cameraPose); // we insert the value for the graphSmart that only contains poses
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
      int    i = measurement.first; // camera id
      double u = measurement.second.x();
      double v = measurement.second.y();
      boost::shared_ptr<Cal3Bundler> Ki(new Cal3Bundler(inputData.cameras[i].calibration()));

      if (useSmartProjectionFactor) {
        // use SMART PROJECTION FACTORS
        smartCreator.add(L(j), X(i), Point2(u,v), pixel_sigma, Ki, graphSmart);
        numLandmarks = smartCreator.getNumLandmarks();
      } else {
        // or STANDARD PROJECTION FACTORS
        projectionCreator.add(L(j), X(i), Point2(u,v), pixel_sigma, Ki, graphProjection);
        numLandmarks = projectionCreator.getNumLandmarks();
      }
    }
  }
  if(debug){
    cout << "Included measurements in the graph " << endl;
    cout << "Number of landmarks  " << numLandmarks << endl;
    cout << "Before call to update: ------------------ " << endl;
    cout << "Poses in SmartGraph values: "<< graphSmartValues->size() << endl;
    Values valuesProjPoses = graphProjectionValues->filter<Pose3>();
    cout << "Poses in ProjectionGraph values: "<< valuesProjPoses.size() << endl;
    Values valuesProjPoints = graphProjectionValues->filter<Point3>();
    cout << "Points in ProjectionGraph values: "<< valuesProjPoints.size() << endl;
    cout << "---------------------------------------------------------- " << endl;
  }

  if (!useSmartProjectionFactor) {
    projectionCreator.update(graphProjection, loadedValues, graphProjectionValues, doTriangulation);
    ordering = projectionCreator.getOrdering();
  }

  if(debug) {
    cout << "After call to update: ------------------ " << endl;
    cout << "--------------------------------------------------------- " << endl;
    cout << "Poses in SmartGraph values: "<< graphSmartValues->size() << endl;
    Values valuesProjPoses = graphProjectionValues->filter<Pose3>();
    cout << "Poses in ProjectionGraph values: "<< valuesProjPoses.size() << endl;
    Values valuesProjPoints = graphProjectionValues->filter<Point3>();
    cout << "Points in ProjectionGraph values: "<< valuesProjPoints.size() << endl;
    cout << "---------------------------------------------------------- " << endl;
  }

  Values result;
  if (useSmartProjectionFactor) {
    if (useLM)
      optimizeGraphLM(graphSmart, graphSmartValues, result, ordering);
    else
      optimizeGraphISAM2(graphSmart, graphSmartValues, result);
    cout << "Initial reprojection error (smart): " << graphSmart.error(*graphSmartValues) << endl;;
    cout << "Final reprojection error (smart): " << graphSmart.error(result) << endl;;
  } else {
    if (useLM)
      optimizeGraphLM(graphProjection, graphProjectionValues, result, ordering);
    else
      optimizeGraphISAM2(graphProjection, graphProjectionValues, result);
    cout << "Initial reprojection error (standard): " << graphProjection.error(*graphProjectionValues) << endl;;
    cout << "Final reprojection error (standard): " << graphProjection.error(result) << endl;;
  }

  tictoc_print_();
  cout << "===================================================" << endl;

  // --------------- WRITE OUTPUT TO BAL FILE  ----------------------------------------
  if(useSmartProjectionFactor){
    smartCreator.computePoints(result);
  }

  cout << "- writing results to (BAL) file... " << endl;
  std::size_t stringCut1 = datasetFile.rfind("/");
  std::size_t stringCut2 = datasetFile.rfind(".txt");
  string outputFile;

  if(useSmartProjectionFactor){
    outputFile = "." + datasetFile.substr(stringCut1, stringCut2-stringCut1) + "-optimized-smart.txt";
  }else{
    if(doTriangulation){
      outputFile = "." + datasetFile.substr(stringCut1, stringCut2-stringCut1) + "-optimized-standard-triangulation.txt";
    }else{
      outputFile = "." + datasetFile.substr(stringCut1, stringCut2-stringCut1) + "-optimized-standard.txt";
    }
  }



  if(debug) cout << outputFile << endl;
  writeBALfromValues(outputFile, inputData, result);
  cout << "- mission accomplished! " << endl;
  exit(0);
}

