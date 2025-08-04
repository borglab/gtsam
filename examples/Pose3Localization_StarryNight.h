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
#include <gtsam/slam/WNOAFactors.h>
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
#include <functional>

using namespace gtsam;
using TimedInput = std::pair<double, Vector6>;
// stereo measurements
//    Pose ID  /  Landmark ID  /  Stereo
using MeasTriple = std::tuple<size_t, size_t, StereoPoint2>;

namespace NM = gtsam::noiseModel;
class FileUtils {
private:
FileUtils() = delete; // Prevent instantiation of static class
template <typename MatrixType>
static MatrixType readMatrix(std::istringstream& iss, size_t rows, size_t cols) {
  MatrixType matrix(rows, cols);
  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      iss >> matrix(i, j);
    }
  }
  return matrix;
}

public:
// load the odometry
// DR: Odometry Input (delta distance traveled and delta heading change)
//    Time (sec)  omega v
static void readInputs(std::vector<TimedInput>& inputs) {
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
static getMetadata() {
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

static void readMeasurements(std::vector<MeasTriple>& measTripleVector, size_t numLandmarks) {
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
        measTripleVector.emplace_back(poseID, landmarkID, StereoPoint2(ul, ur, vlr));
      }
    }
    poseID++;
  }
  is.clear();
}

// load groundtruth poses and landmarks
static void readGroundTruth(std::vector<Pose3>& poses, std::vector<Point3>& landmarks) {
  // groundtruth poses in world frame
  std::string data_file = findExampleDataFile("starryNightGroundtruth.txt");
  std::ifstream is(data_file.c_str());
  std::string line;
  while (std::getline(is, line)) {
    std::istringstream iss(line);
    poses.push_back(Pose3(readMatrix<Matrix4>(iss, 4, 4)));
    if (poses.size() == 1) {
      std::cout << "Ground truth pose 0: " << poses[0].translation().transpose() << ", "
                << poses[0].rotation().yaw() << ", " << poses[0].rotation().pitch() << ", "
                << poses[0].rotation().roll() << std::endl;
      std::cout << "Ground truth pose matrix: " << poses[0].matrix() << std::endl;
    }
  }
  is.clear();

  // landmarks
  data_file = findExampleDataFile("starryNightLandmarkPos.txt");
  std::ifstream is2(data_file.c_str());
  while (std::getline(is2, line)) {
    std::istringstream iss(line);
    auto landmark = Point3(readMatrix<Vector3>(iss, 3, 1));
    // std::cout << "Landmark " << landmarks.size() << ": " << landmark.transpose() << std::endl;
    landmarks.push_back(landmark);
  }
  is2.clear();
}

// --- Save Poses to CSV ---
static void savePosesToFile(Values& result,
                                size_t numPoses,
                                const std::string& filename,
                                size_t poseInterval = 1) {

  // open file, print header
  std::ofstream poses_file(filename + ".csv");
  if (poses_file.is_open()) {
    poses_file
        << "key,x,y,z,yaw,pitch,roll\n";  // Header for Pose3
    // get poses
    for (size_t poseID = 0; poseID < numPoses; poseID += poseInterval) {
      Symbol poseSymbol('x', poseID);
      if (result.exists(poseSymbol)) {
        auto pose = result.at<Pose3>(poseSymbol);
        auto t = pose.translation();
        auto r = pose.rotation();
        poses_file << poseSymbol.string() << "," << t.x() << "," << t.y() << ","
                 << t.z() << "," << r.yaw() << "," << r.pitch() << "," << r.roll() << "\n";
      } else {
        std::cout << "Pose " << poseID << " does not exist in the result." << std::endl;
      }
    }
    poses_file.close();
  } else {
    std::cerr << "Error opening file" << std::endl;
  }
}

// --- Save Landmarks to CSV ---
static void saveLandmarksToFile(Values& result,
                         size_t numLandmarks,
                         const std::string& filename) {
  // open file, print header
  std::ofstream landmarks_file(filename + ".csv");
  if (landmarks_file.is_open()) {
    landmarks_file << "key,x,y,z\n";  // Header for Point3
    // get landmarks
    for (size_t landmarkID = 0; landmarkID < numLandmarks; ++landmarkID) {
      Symbol landmarkSymbol('l', landmarkID);
      if (result.exists(landmarkSymbol)) {
        auto landmark = result.at<Point3>(landmarkSymbol);
        landmarks_file << landmarkSymbol.string() << "," << landmark.x() << ","
                       << landmark.y() << "," << landmark.z() << "\n";
      } else {
        std::cout << "Landmark " << landmarkID << " does not exist in the result." << std::endl;
      }
    }
    landmarks_file.close();
  } else {
    std::cerr << "Error opening file" << std::endl;
  }
}

static void saveMarginalsToFile(const Marginals& marginals, size_t numPoses, const std::string& filename, size_t poseInterval = 1) {
  std::ofstream marginalsFile(filename + ".csv");
  if (!marginalsFile.is_open()) {
    std::cerr << "Error opening file for writing marginals" << std::endl;
    return;
  }
  // marginalsFile << "PoseID\tCovariance\n";
  for (size_t poseID = 0; poseID < numPoses; poseID += poseInterval) {
    Symbol poseSymbol('x', poseID);
    const Matrix& covariance = marginals.marginalCovariance(poseSymbol);
    marginalsFile << covariance.reshaped(1, covariance.size()) << "\n";
  }
  marginalsFile.close();
  
}

};

