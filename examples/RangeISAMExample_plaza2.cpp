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
#include <gtsam/geometry/Pose2.h>
using gtsam::Pose2;

// gtsam::Vectors are dynamic Eigen vectors, Vector3 is statically sized.
#include <gtsam/base/Vector.h>
using gtsam::Vector;
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

// Measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics SLAM problems:
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

// Timing, with functions below, provides nice facilities to benchmark.
#include <gtsam/base/timing.h>
using gtsam::tictoc_print_;

// Standard headers, added last, so we know headers above work on their own.
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace NM = gtsam::noiseModel;

// Data is second UWB ranging dataset, B2 or "plaza 2", from
// "Navigating with Ranging Radios: Five Data Sets with Ground Truth"
// by Joseph Djugash, Bradley Hamner, and Stephan Roth
// https://www.ri.cmu.edu/pub_files/2009/9/Final_5datasetsRangingRadios.pdf

// load the odometry
// DR: Odometry Input (delta distance traveled and delta heading change)
//    Time (sec)  Delta Distance Traveled (m) Delta Heading (rad)
using TimedOdometry = std::pair<double, Pose2>;
std::list<TimedOdometry> readOdometry() {
  std::list<TimedOdometry> odometryList;
  std::string data_file = gtsam::findExampleDataFile("Plaza2_DR.txt");
  std::ifstream is(data_file.c_str());

  while (is) {
    double t, distance_traveled, delta_heading;
    is >> t >> distance_traveled >> delta_heading;
    odometryList.emplace_back(t, Pose2(distance_traveled, 0, delta_heading));
  }
  is.clear(); /* clears the end-of-file and error flags */
  return odometryList;
}

// load the ranges from TD
//    Time (sec)  Sender / Antenna ID Receiver Node ID  Range (m)
using RangeTriple = boost::tuple<double, size_t, double>;
std::vector<RangeTriple> readTriples() {
  std::vector<RangeTriple> triples;
  std::string data_file = gtsam::findExampleDataFile("Plaza2_TD.txt");
  std::ifstream is(data_file.c_str());

  while (is) {
    double t, range, sender, receiver;
    is >> t >> sender >> receiver >> range;
    triples.emplace_back(t, receiver, range);
  }
  is.clear(); /* clears the end-of-file and error flags */
  return triples;
}

// main
int main(int argc, char** argv) {
  // load Plaza2 data
  std::list<TimedOdometry> odometry = readOdometry();
  size_t M = odometry.size();
  std::cout << "Read " << M << " odometry entries." << std::endl;

  std::vector<RangeTriple> triples = readTriples();
  size_t K = triples.size();
  std::cout << "Read " << K << " range triples." << std::endl;

  // parameters
  size_t minK =
      150;  // minimum number of range measurements to process initially
  size_t incK = 25;  // minimum number of range measurements to process after
  bool robust = true;

  // Set Noise parameters
  Vector priorSigmas = Vector3(1, 1, M_PI);
  Vector odoSigmas = Vector3(0.05, 0.01, 0.1);
  double sigmaR = 100;        // range standard deviation
  const NM::Base::shared_ptr  // all same type
      priorNoise = NM::Diagonal::Sigmas(priorSigmas),  // prior
      looseNoise = NM::Isotropic::Sigma(2, 1000),      // loose LM prior
      odoNoise = NM::Diagonal::Sigmas(odoSigmas),      // odometry
      gaussian = NM::Isotropic::Sigma(1, sigmaR),      // non-robust
      tukey = NM::Robust::Create(NM::mEstimator::Tukey::Create(15),
                                 gaussian),  // robust
      rangeNoise = robust ? tukey : gaussian;

  // Initialize iSAM
  gtsam::ISAM2 isam;

  // Add prior on first pose
  Pose2 pose0 = Pose2(-34.2086489999201, 45.3007639991120, M_PI - 2.021089);
  gtsam::NonlinearFactorGraph newFactors;
  newFactors.addPrior(0, pose0, priorNoise);
  gtsam::Values initial;
  initial.insert(0, pose0);

  // We will initialize landmarks randomly, and keep track of which landmarks we
  // already added with a set.
  std::mt19937_64 rng;
  std::normal_distribution<double> normal(0.0, 100.0);
  std::set<Symbol> initializedLandmarks;

  // set some loop variables
  size_t i = 1;  // step counter
  size_t k = 0;  // range measurement counter
  bool initialized = false;
  Pose2 lastPose = pose0;
  size_t countK = 0;

  // Loop over odometry
  gttic_(iSAM);
  for (const TimedOdometry& timedOdometry : odometry) {
    //--------------------------------- odometry loop --------------------------
    double t;
    Pose2 odometry;
    boost::tie(t, odometry) = timedOdometry;

    // add odometry factor
    newFactors.emplace_shared<gtsam::BetweenFactor<Pose2>>(i - 1, i, odometry,
                                                           odoNoise);

    // predict pose and add as initial estimate
    Pose2 predictedPose = lastPose.compose(odometry);
    lastPose = predictedPose;
    initial.insert(i, predictedPose);

    // Check if there are range factors to be added
    while (k < K && t >= boost::get<0>(triples[k])) {
      size_t j = boost::get<1>(triples[k]);
      Symbol landmark_key('L', j);
      double range = boost::get<2>(triples[k]);
      newFactors.emplace_shared<gtsam::RangeFactor<Pose2, Point2>>(
          i, landmark_key, range, rangeNoise);
      if (initializedLandmarks.count(landmark_key) == 0) {
        std::cout << "adding landmark " << j << std::endl;
        double x = normal(rng), y = normal(rng);
        initial.insert(landmark_key, Point2(x, y));
        initializedLandmarks.insert(landmark_key);
        // We also add a very loose prior on the landmark in case there is only
        // one sighting, which cannot fully determine the landmark.
        newFactors.emplace_shared<gtsam::PriorFactor<Point2>>(
            landmark_key, Point2(0, 0), looseNoise);
      }
      k = k + 1;
      countK = countK + 1;
    }

    // Check whether to update iSAM 2
    if ((k > minK) && (countK > incK)) {
      if (!initialized) {  // Do a full optimize for first minK ranges
        std::cout << "Initializing at time " << k << std::endl;
        gttic_(batchInitialization);
        gtsam::LevenbergMarquardtOptimizer batchOptimizer(newFactors, initial);
        initial = batchOptimizer.optimize();
        gttoc_(batchInitialization);
        initialized = true;
      }
      gttic_(update);
      isam.update(newFactors, initial);
      gttoc_(update);
      gttic_(calculateEstimate);
      gtsam::Values result = isam.calculateEstimate();
      gttoc_(calculateEstimate);
      lastPose = result.at<Pose2>(i);
      newFactors = gtsam::NonlinearFactorGraph();
      initial = gtsam::Values();
      countK = 0;
    }
    i += 1;
    //--------------------------------- odometry loop --------------------------
  }  // end for
  gttoc_(iSAM);

  // Print timings
  tictoc_print_();

  // Print optimized landmarks:
  gtsam::Values finalResult = isam.calculateEstimate();
  for (auto&& landmark_key : initializedLandmarks) {
    Point2 p = finalResult.at<Point2>(landmark_key);
    std::cout << landmark_key << ":" << p.transpose() << "\n";
  }

  exit(0);
}
