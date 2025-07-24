/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file RangeISAMExample_plaza2.cpp
 * @brief A 2D Range SLAM example
 * @date June 20, 2013
 * @author Frank Dellaert
 */

// Both relative poses and recovered trajectory poses will be stored as Pose2.
#include <gtsam/geometry/Pose3.h>
using gtsam::Pose2;

// gtsam::Vectors are dynamic Eigen vectors, Vector3 is statically sized.
#include <gtsam/base/Vector.h>
using gtsam::Vector;
using gtsam::Vector2;
using gtsam::Vector3;

// Unknown landmarks are of type Point2 (which is just a 2D Eigen vector).
#include <gtsam/geometry/Point2.h>
using gtsam::Point2;

// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols.
#include <gtsam/inference/Symbol.h>
using gtsam::Symbol;

// We want to use iSAM2 to solve the range-SLAM problem incrementally.
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a
// factor graph, and initial guesses for any new variables in the added factors.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// We will use a non-linear solver to batch-initialize from the first 150 frames
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

// Measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics SLAM problems:
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

// Timing, with functions below, provides nice facilities to benchmark.
#include <gtsam/base/timing.h>
using gtsam::tictoc_print_;

#include <gtsam/slam/StereoFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

// Standard headers, added last, so we know headers above work on their own.
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace gtsam;
namespace NM = gtsam::noiseModel;

bool USE_MOTION_PRIOR = true;  // if false, system may not be observable
bool USE_MEASUREMENTS = true;
size_t MAX_POSES = 1000;  // maximum number of poses to process

// This dataset is...

template <typename MatrixType>
MatrixType readMatrix(std::istringstream& iss, size_t rows, size_t cols) {
  MatrixType matrix(rows, cols);
  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      iss >> matrix(i, j);
    }
  }
  return matrix;
}

// load the odometry
// DR: Odometry Input (delta distance traveled and delta heading change)
//    Time (sec)  omega v
using TimedInput = std::pair<double, Vector6>;
void readInputs(std::vector<TimedInput>& inputs) {
  std::string data_file = findExampleDataFile("starryNightInput.txt");
  std::ifstream is(data_file.c_str());
  while (is) {
    double t;
    std::string line;
    if (!std::getline(is, line)) break;
    std::istringstream iss(line);
    iss >> t;
    // template is varpi = [omega; v]
    inputs.push_back(TimedInput(t, readMatrix<Vector6>(iss, 6, 1)));
  }
  is.clear(); /* clears the end-of-file and error flags */
}

// load camera calibration (intrinsics and extrinsics) and noise params
std::tuple<Cal3_S2Stereo::shared_ptr, Pose3, NM::Diagonal::shared_ptr, NM::Diagonal::shared_ptr>
getMetadata() {
  std::string data_file = findExampleDataFile("starryNightMetadata.txt");
  std::ifstream is(data_file.c_str());
  std::string line;

  // first line contains the intrinsics
  std::getline(is, line);
  std::istringstream iss(line);
  double fx, fy, s, u0, v0, b;
  iss >> fx >> fy >> s >> u0 >> v0 >> b;
  Cal3_S2Stereo::shared_ptr Cal(new Cal3_S2Stereo(fx, fy, s, u0, v0, b));

  // second line contains the extrinsics (camera to vehicle transform) rotation
  std::getline(is, line);
  std::istringstream iss2(line);
  auto C_cv = readMatrix<Matrix3>(iss2, 3, 3);

  // third line contains the extrinsics (camera to vehicle transform) translation
  std::getline(is, line);
  std::istringstream iss3(line);
  auto t_cv = readMatrix<Vector3>(iss3, 3, 1);

  auto T_cv = Pose3(Rot3(C_cv), Point3(t_cv));

  // fourth line contains odom noise
  std::getline(is, line);
  std::istringstream iss4(line);
  auto odomNoise = NM::Diagonal::Sigmas(readMatrix<Vector6>(iss4, 6, 1).cwiseSqrt());

  // fifth line contains stereo noise
  std::getline(is, line);
  std::istringstream iss5(line);
  auto stereoNoise = NM::Diagonal::Sigmas(readMatrix<Vector3>(iss5, 3, 1).cwiseSqrt());

  is.clear();
  return std::tuple<Cal3_S2Stereo::shared_ptr, Pose3, NM::Diagonal::shared_ptr, NM::Diagonal::shared_ptr> (Cal, T_cv, odomNoise, stereoNoise);
}

// load the stereo measurements
//    Pose ID  /  Landmark ID  /  Stereo
using MeasTriple = std::tuple<size_t, size_t, StereoPoint2>;
void readMeasurements(std::list<MeasTriple>& measTripleList, size_t numLandmarks) {
  std::string data_file = findExampleDataFile("starryNightMeas.txt");
  std::ifstream is(data_file.c_str());
  size_t poseID = 0;
  while (is) {
    std::string line;
    if (!std::getline(is, line)) break;
    std::istringstream iss(line);
    for (size_t landmarkID = 0; landmarkID < numLandmarks; ++landmarkID) {
      double ul, vl, ur, vr;
      iss >> ul >> vl >> ur >> vr;  // vr is not used for now - Todo fix this
      double vlr = (vl + vr) / 2.0;  // average vertical coordinate
      if (ul > -0.5) { // -1 is used to indicate no measurement
        measTripleList.emplace_back(poseID, landmarkID, StereoPoint2(ul, ur, vlr));
      }
    }
    poseID++;
  }
  is.clear();
}

// load groundtruth poses and landmarks
void readGroundTruth(std::vector<Pose3>& poses, std::vector<Point3>& landmarks) {
  // groundtruth poses in world frame
  std::string data_file = findExampleDataFile("starryNightGroundtruth.txt");
  std::ifstream is(data_file.c_str());
  std::string line;
  while (std::getline(is, line)) {
    std::istringstream iss(line);
    poses.push_back(Pose3(readMatrix<Matrix4>(iss, 4, 4)));
  }
  is.clear();

  // landmarks
  data_file = findExampleDataFile("starryNightLandmarkPos.txt");
  std::ifstream is2(data_file.c_str());
  while (std::getline(is2, line)) {
    std::istringstream iss(line);
    auto landmark = Point3(readMatrix<Vector3>(iss, 3, 1));
    std::cout << "Landmark " << landmarks.size() << ": " << landmark.transpose() << std::endl;
    landmarks.push_back(landmark);
  }
  is2.clear();
}

// --------------------------------------------------------------------------
int main(int argc, char** argv) {
  // Parse command line arguments
  if (argc > 1) {
    for (int i = 1; i < argc; ++i) {
      std::string arg(argv[i]);
      if (arg == "--no-motion-prior") {
        USE_MOTION_PRIOR = false;
      } else if (arg == "--no-measurements") {
        USE_MEASUREMENTS = false;
      } else if (arg == "--max-poses") {
        if (i + 1 < argc) {
          ++i;  // Move to the next argument
          try {
            MAX_POSES = std::stoul(argv[i]);
          } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid value for --max-poses: " << argv[i] << std::endl;
            return 1;
          }
        } else {
          std::cerr << "--max-poses requires an argument." << std::endl;
          return 1;
        }
      } else if (arg == "--help" || arg == "-h") {
        std::cout << "Usage: " << argv[0] << " [--no-motion-prior] [--no-measurements] [--max-poses <number>]" << std::endl;
        std::cout << "Options:" << std::endl;
        std::cout << "  --no-motion-prior       Disable the motion prior (odometry factors)." << std::endl;
        std::cout << "  --no-measurements        Disable the range measurements." << std::endl;
        std::cout << "  --max-poses <number>  Set the maximum number of poses to process (default: 1000)." << std::endl;
        std::cout << "  --help, -h              Show this help message." << std::endl;
        return 0;
      } else {
        std::cerr << "Unknown argument: " << arg << std::endl;
        std::cerr << "Usage: " << argv[0] << " [--no-motion-prior] [--no-measurements]" << std::endl;
        return 1;
      }
    }
  }

  // load Plaza2 data
  std::vector<TimedInput> inputs;
  readInputs(inputs);
  std::cout << "Loaded " << inputs.size() << " inputs." << std::endl;
  const auto [Cal, T_cv, odomNoise, stereoNoise] = getMetadata();
  std::cout << "Transform from camera to vehicle frame: " << T_cv << std::endl;
  std::cout << "Camera calibration: " << *Cal << std::endl;
  std::cout << "Odometry noise: " << odomNoise->sigmas() << std::endl;
  std::cout << "Stereo noise: " << stereoNoise->sigmas() << std::endl;
  std::cout << "Loaded stereo camera calibration and noise params." << std::endl;
  std::vector<Pose3> gtPoses;
  std::vector<Point3> gtLandmarks;
  readGroundTruth(gtPoses, gtLandmarks);
  std::cout << "Loaded " << gtPoses.size() << " ground truth poses and " << gtLandmarks.size() << " landmarks." << std::endl;
  std::list<MeasTriple> measTripleList;
  readMeasurements(measTripleList, gtLandmarks.size());
  std::cout << "Loaded " << measTripleList.size() << " range measurements." << std::endl;

  NonlinearFactorGraph graph;
  Values initialEstimate;

  // Add a prior on the first pose
  auto priorNoise = NM::Diagonal::Sigmas((Vector(6)<<0.3,0.3,0.3,0.1,0.1,0.1).finished());
  // get a prior noise model for the first pose, using the first dt
  double dt_1 = inputs[1].first - inputs[0].first;
  auto odomNoiseDiscrete = NM::Diagonal::Sigmas(odomNoise->sigmas() * dt_1);
  graph.addPrior(Symbol('x', 0), gtPoses[0], priorNoise);
  initialEstimate.insert(Symbol('x', 0), gtPoses[0]);

  size_t numPoses;
  if (USE_MOTION_PRIOR) {
    // Add odometry factors
    numPoses = std::min(inputs.size(), MAX_POSES);
    auto currPredictedPose = gtPoses[0];
    for (size_t poseID = 0; poseID < numPoses - 1; ++poseID) {
      // get time difference
      double dt = inputs[poseID + 1].first - inputs[poseID].first;
      // Create a BetweenFactor for the odometry
      Vector6 odometry = inputs[poseID].second; // v, omega
      Pose3 odometryPose = Pose3::Expmap(odometry*dt);
      auto odomNoiseDiscrete = NM::Diagonal::Sigmas(odomNoise->sigmas() * dt);
      graph.emplace_shared<BetweenFactor<Pose3> >(Symbol('x', poseID), Symbol('x', poseID + 1), odometryPose, odomNoiseDiscrete);
      // Insert the initial estimate for the next pose
      currPredictedPose = currPredictedPose.compose(odometryPose);
      initialEstimate.insert(Symbol('x', poseID + 1), currPredictedPose);
    }
  } else {
    // If no motion prior, just add the ground truth poses as initial estimates
    numPoses = gtPoses.size();
    for (size_t poseID = 1; poseID < numPoses; ++poseID) {
      initialEstimate.insert(Symbol('x', poseID), gtPoses[poseID]);
    }
  }

  // tried to use Expression<StereoPoint2> but it does not work
  // auto prediction = StereoPoint2_(&project2_static, T_cv_, Point3_(landmark), Cal_);


  // for localization, add all groundtruth landmark positions
  size_t numLandmarks = gtLandmarks.size();
  for (size_t landmarkID = 0; landmarkID < numLandmarks; ++landmarkID) {
    graph.add(NonlinearEquality<Point3>(Symbol('l', landmarkID), gtLandmarks[landmarkID]));
    initialEstimate.insert(Symbol('l', landmarkID), gtLandmarks[landmarkID]);
  }
  if (USE_MEASUREMENTS) {    
    // Add stereo measurement factors
    for (const auto& measTriple : measTripleList) {
      auto [poseID, landmarkID, measurement] = measTriple;
      if (poseID < MAX_POSES && landmarkID < numLandmarks) {
        // Add a stereo factor for the measurement
        // std::cout << "Measurement for pose " << poseID << " and landmark " << landmarkID << ": " << measurement << std::endl;
        // test out using StereoCamera
        auto T_iv = initialEstimate.at<Pose3>(Symbol('x', poseID));
        auto T_ic = T_iv.compose(T_cv);
        auto cam = StereoCamera(T_ic, Cal);
        StereoPoint2 prediction = cam.project(gtLandmarks[landmarkID]);
        std::cout << "Predicted measurement for pose " << poseID << " and landmark " << landmarkID << ": " << prediction << std::endl;
        graph.emplace_shared<GenericStereoFactor<Pose3, Point3> >(
          measurement, stereoNoise, Symbol('x', poseID), Symbol('l', landmarkID), Cal, T_cv);
      }
    }
  }
  
  // graph.print("\nFactor Graph:\n");
  initialEstimate.print("\nInitial Estimate:\n");

  GaussNewtonParams parameters;
  parameters.relativeErrorTol = 1e-5;
  parameters.maxIterations = 100;
  parameters.setVerbosity("ERROR");
  GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and save marginal covariances to file for all poses
  std::cout.precision(3);
  Marginals marginals(graph, result);
  std::ofstream marginalsFile("starry_night_marginals.txt");
  if (!marginalsFile.is_open()) {
    std::cerr << "Error opening file for writing marginals" << std::endl;
    return 1;
  }
  // marginalsFile << "PoseID\tCovariance\n";
  for (size_t poseID = 0; poseID < numPoses; ++poseID) {
    Symbol poseSymbol('x', poseID);
    if (result.exists(poseSymbol)) {
      const Matrix& covariance = marginals.marginalCovariance(poseSymbol);
      marginalsFile << covariance.reshaped(1, covariance.size()) << "\n";
      // std::cout << "Pose " << poseID << " covariance:\n" << covariance << std::endl;
    } else {
      std::cout << "Pose " << poseID << " does not exist in the result." << std::endl;
    }
  }
  marginalsFile.close();
  // std::cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 0)) << std::endl;

  // Write to a .dot file
  graph.saveGraph("graph.dot", result);
  writeG2o(graph, initialEstimate, "starry_night_initial.g2o");
  writeG2o(graph, result, "starry_night_optimized.g2o");

  exit(0);
}
