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
//typedef SmartProjectionFactorsCreator<Pose3, Point3, Cal3Bundler> SmartFactorsCreator;
//typedef GenericProjectionFactorsCreator<Pose3, Point3, Cal3Bundler> ProjectionFactorsCreator;
typedef SmartProjectionFactorsCreator<Pose3, Point3, Cal3_S2> SmartFactorsCreator;
typedef GenericProjectionFactorsCreator<Pose3, Point3, Cal3_S2> ProjectionFactorsCreator;
typedef FastMap<Key, int> OrderingMap;

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

    //for (int i = 0; i < 3; i++) {
      LevenbergMarquardtOptimizer optimizer(graph, *graphValues, params);
      params.ordering = Ordering::COLAMD(graph);
      gttic_(SmartProjectionFactorExample_kitti);
      result = optimizer.optimize();
      gttoc_(SmartProjectionFactorExample_kitti);
      tictoc_finishedIteration_();

      std::cout << "Total number of LM iterations: " << optimizer.state().iterations << std::endl;
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

  // Set to true to use SmartProjectionFactor. Otherwise GenericProjectionFactor will be used
  bool useSmartProjectionFactor = true;
  bool doTriangulation = true; // we read points initial guess from file or we triangulate

  bool useLM = true; 
  bool addNoise = false;

  // Smart factors settings
  double linThreshold = -1.0; // negative is disabled
  double rankTolerance = 1.0;

  // bool incrementalFlag = false;
  // int optSkip = 200; // we optimize the graph every optSkip poses

  std::cout << "PARAM SmartFactor: " << useSmartProjectionFactor << std::endl;
  std::cout << "PARAM LM: " << useLM << std::endl;
  std::cout << "PARAM linThreshold (negative is disabled): " << linThreshold << std::endl;
  if(addNoise)
    std::cout << "PARAM Noise: " << addNoise << std::endl;

  // Get home directory and dataset
  string HOME = getenv("HOME");
  string datasetFile = HOME + "/data/SfM/BAL/Ladybug/problem-1031-110968-pre.txt";
  //  string datasetFile = HOME + "/data/SfM/BAL/Ladybug/problem-1723-156502-pre.txt";
  //  string datasetFile = HOME + "/data/SfM/BAL/Final/problem-961-187103-pre.txt";
  //  string datasetFile = HOME + "/data/SfM/BAL/Final/problem-1936-649673-pre.txt";

  //  string datasetFile = HOME + "/data/SfM/BAL/Final/problem-3068-310854-pre.txt";
  //  string datasetFile = HOME + "/data/SfM/BAL/Final/problem-4585-1324582-pre.txt";
  //  13682    4456117     28987644   problem-13682-4456117-pre.txt.bz2

  static SharedNoiseModel pixel_sigma(noiseModel::Unit::Create(2));
  NonlinearFactorGraph graphSmart, graphProjection;

  gtsam::Values::shared_ptr graphSmartValues(new gtsam::Values());
  gtsam::Values::shared_ptr graphProjectionValues(new gtsam::Values());
  gtsam::Values::shared_ptr loadedValues(new gtsam::Values()); // values we read from file
  Values result;

  // Read in kitti dataset
  ifstream fin;
  fin.open((datasetFile).c_str());
  if(!fin) {
    cerr << "Could not open dataset" << endl;
    exit(1);
  }

  // read all measurements
  cout << "Reading dataset... " << endl;
  unsigned int numLandmarks = 0, numPoses = 0;
  Key r, l;
  double u, v;
  double x, y, z, rotx, roty, rotz, f, k1, k2;
  std::vector<Key> landmarkKeys, cameraPoseKeys;

  bool optimized = false;
  boost::shared_ptr<Ordering> ordering(new Ordering());

//   std::vector< boost::shared_ptr<Cal3Bundler> > K_cameras; // TODO: uncomment

//  boost::shared_ptr<Cal3Bundler> K(new Cal3Bundler()); // TODO: uncomment
  Cal3_S2::shared_ptr K(new Cal3_S2());

  SmartFactorsCreator smartCreator(pixel_sigma, K, rankTolerance, linThreshold); // this initial K is not used
  ProjectionFactorsCreator projectionCreator(pixel_sigma, K);  // this initial K is not used

  // main loop: reads measurements and adds factors (also performs optimization if desired)
  // r >> pose id
  // l >> landmark id
  // (u >> u) >> measurement
  unsigned int totNumLandmarks=0, totNumPoses=0, totNumMeasurements=0;
  fin >> totNumPoses >> totNumLandmarks >> totNumMeasurements;

  cout << "Dataset: #poses: " << totNumPoses << " #points: " << totNumLandmarks << " #measurements: " << totNumMeasurements << " " << endl;

  std::vector<double> vector_u;
  std::vector<double> vector_v;
  std::vector<int> vector_r;
  std::vector<int> vector_l;

  // read measurements
  for(unsigned int i = 0; i < totNumMeasurements; i++){
    fin >> r >> l >> u >> v;
    vector_u.push_back(u);
    vector_v.push_back(v);
    vector_r.push_back(r);
    vector_l.push_back(l);
  }

  cout << "last measurement: " << r << " " << l << " " << u << " " << v << endl;

  std::vector< boost::shared_ptr<Cal3_S2> > K_cameras;

  // create values
  for(unsigned int i = 0; i < totNumPoses; i++){
    // R,t,f,k1 and k2.
    fin >> rotx >> roty >> rotz  >> x >> y >> z >> f >> k1 >> k2;
//    boost::shared_ptr<Cal3Bundler> Kbundler(new Cal3Bundler(f, k1, k2, 0.0, 0.0)); //TODO: uncomment
//    K_cameras.push_back(Kbundler); //TODO: uncomment
    boost::shared_ptr<Cal3_S2> K_S2(new Cal3_S2(f, f, 0.0, 0.0, 0.0));
    // cout << "f "<< f << endl;
    K_cameras.push_back(K_S2);
    Vector3 rotVect(rotx,roty,rotz);
    // FORMAT CONVERSION!! R -> R'
    Rot3 R = Rot3::Expmap(rotVect);
    Matrix3  R_bal_gtsam_mat = Matrix3::Zero(3,3);
    R_bal_gtsam_mat(0,0) = 1.0;  R_bal_gtsam_mat(1,1) = -1.0; R_bal_gtsam_mat(2,2) = -1.0;
    Rot3 R_bal_gtsam_ = Rot3(R_bal_gtsam_mat);
    Pose3 CameraPose((R.inverse()).compose(R_bal_gtsam_), - R.unrotate(Point3(x,y,z)));

    if(addNoise){
      Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/100, 0., -M_PI/100), gtsam::Point3(0.3,0.1,0.3));
      CameraPose = CameraPose.compose(noise_pose);
    }
    loadedValues->insert(X(i), CameraPose );
  }

  cout << "last pose: " << x << " " << y << " " << z << " " << rotx << " "
      << roty << " " << rotz << " " << f << " " << k1 << " " << k2 << endl;

  // add landmarks in standard projection factors
  if(!useSmartProjectionFactor){
    for(unsigned int i = 0; i < totNumLandmarks; i++){
      fin >> x >> y >> z;
      // FORMAT CONVERSION!! XPOINT
      loadedValues->insert(L(i), Point3(x,y,z) );
    }
  }

  cout << "last point: " << x << " " << y << " " << z << endl;

  // 1: add values and factors to the graph
  // 1.1: add factors
  // SMART FACTORS ..
  for(size_t i = 0; i < vector_u.size(); i++){

    l = vector_l.at(i);
    r = vector_r.at(i);
    // FORMAT CONVERSION!! XPOINT
    u = vector_u.at(i);
    // FORMAT CONVERSION!! XPOINT
    v = - vector_v.at(i);

    if (useSmartProjectionFactor) {

      smartCreator.add(L(l), X(r), Point2(u,v), pixel_sigma, K_cameras.at(r), graphSmart);
      numLandmarks = smartCreator.getNumLandmarks();

      // Add initial pose value if pose does not exist
      if (!graphSmartValues->exists<Pose3>(X(r)) && loadedValues->exists<Pose3>(X(r))) {
        graphSmartValues->insert(X(r), loadedValues->at<Pose3>(X(r)));
        numPoses++;
        optimized = false;
      }

    } else {
      // or STANDARD PROJECTION FACTORS
      projectionCreator.add(L(l), X(r), Point2(u,v), pixel_sigma, K_cameras.at(r), graphProjection);
      numLandmarks = projectionCreator.getNumLandmarks();
      optimized = false;
    }
  }

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

  optimized = true;

  cout << "===================================================" << endl;
  tictoc_print_();
  cout << "===================================================" << endl;
  writeValues("./", result);

  if (debug) cout << numLandmarks << " " <<  numPoses << " " << optimized << endl;

  exit(0);
}
