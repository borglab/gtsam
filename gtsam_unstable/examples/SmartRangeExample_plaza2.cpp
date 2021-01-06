/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SmartRangeExample_plaza2.cpp
 * @brief A 2D Range SLAM example
 * @date June 20, 2013
 * @author FRank Dellaert
 */

// Both relative poses and recovered trajectory poses will be stored as Pose2 objects
#include <gtsam/geometry/Pose2.h>

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
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>

// To find data files, we can use `findExampleDataFile`, declared here:
#include <gtsam/slam/dataset.h>

// Standard headers, added last, so we know headers above work on their own
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;
namespace NM = gtsam::noiseModel;

// data available at http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/
// Datafile format (from http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/log.html)

// load the odometry
// DR: Odometry Input (delta distance traveled and delta heading change)
//    Time (sec)  Delta Dist. Trav. (m) Delta Heading (rad)
typedef pair<double, Pose2> TimedOdometry;
list<TimedOdometry> readOdometry() {
  list<TimedOdometry> odometryList;
  string drFile = findExampleDataFile("Plaza2_DR.txt");
  ifstream is(drFile);
  if (!is) throw runtime_error("Plaza2_DR.txt file not found");

  while (is) {
    double t, distance_traveled, delta_heading;
    is >> t >> distance_traveled >> delta_heading;
    odometryList.push_back(
        TimedOdometry(t, Pose2(distance_traveled, 0, delta_heading)));
  }
  is.clear(); /* clears the end-of-file and error flags */
  return odometryList;
}

// load the ranges from TD
//    Time (sec)  Sender / Antenna ID Receiver Node ID  Range (m)
typedef boost::tuple<double, size_t, double> RangeTriple;
vector<RangeTriple> readTriples() {
  vector<RangeTriple> triples;
  string tdFile = findExampleDataFile("Plaza2_TD.txt");
  ifstream is(tdFile);
  if (!is) throw runtime_error("Plaza2_TD.txt file not found");

  while (is) {
    double t, sender, receiver, range;
    is >> t >> sender >> receiver >> range;
    triples.push_back(RangeTriple(t, receiver, range));
  }
  is.clear(); /* clears the end-of-file and error flags */
  return triples;
}

// main
int main(int argc, char** argv) {

  // load Plaza1 data
  list<TimedOdometry> odometry = readOdometry();
//  size_t M = odometry.size();

  vector<RangeTriple> triples = readTriples();
  size_t K = triples.size();

  // parameters
  size_t incK = 50; // minimum number of range measurements to process after
  bool robust = false;

  // Set Noise parameters
  Vector priorSigmas = Vector3(0.01, 0.01, 0.01);
  Vector odoSigmas = Vector3(0.05, 0.01, 0.2);
  double sigmaR = 100; // range standard deviation
  const NM::Base::shared_ptr // all same type
  priorNoise = NM::Diagonal::Sigmas(priorSigmas), //prior
  odoNoise = NM::Diagonal::Sigmas(odoSigmas), // odometry
  gaussian = NM::Isotropic::Sigma(1, sigmaR), // non-robust
  tukey = NM::Robust::Create(NM::mEstimator::Tukey::Create(15), gaussian), //robust
  rangeNoise = robust ? tukey : gaussian;

  // Initialize iSAM
  ISAM2 isam;

  // Add prior on first pose
  Pose2 pose0 = Pose2(-34.2086489999201, 45.3007639991120, -2.02108900000000);
  NonlinearFactorGraph newFactors;
  newFactors.addPrior(0, pose0, priorNoise);

  //  initialize points (Gaussian)
  Values initial;
  initial.insert(symbol('L', 1), Point2(-68.9265, 18.3778));
  initial.insert(symbol('L', 6), Point2(-37.5805, 69.2278));
  initial.insert(symbol('L', 0), Point2(-33.6205, 26.9678));
  initial.insert(symbol('L', 5), Point2(1.7095, -5.8122));
  Values landmarkEstimates = initial; // copy landmarks
  initial.insert(0, pose0);

  // set some loop variables
  size_t i = 1; // step counter
  size_t k = 0; // range measurement counter
  Pose2 lastPose = pose0;
  size_t countK = 0;

  // Loop over odometry
  gttic_(iSAM);
  for(const TimedOdometry& timedOdometry: odometry) {
    //--------------------------------- odometry loop -----------------------------------------
    double t;
    Pose2 odometry;
    boost::tie(t, odometry) = timedOdometry;

    // add odometry factor
    newFactors.push_back(
        BetweenFactor<Pose2>(i - 1, i, odometry,
            NM::Diagonal::Sigmas(odoSigmas)));

    // predict pose and add as initial estimate
    Pose2 predictedPose = lastPose.compose(odometry);
    lastPose = predictedPose;
    initial.insert(i, predictedPose);
    landmarkEstimates.insert(i, predictedPose);

    // Check if there are range factors to be added
    while (k < K && t >= boost::get<0>(triples[k])) {
      size_t j = boost::get<1>(triples[k]);
      double range = boost::get<2>(triples[k]);
      RangeFactor<Pose2, Point2> factor(i, symbol('L', j), range, rangeNoise);
      // Throw out obvious outliers based on current landmark estimates
      Vector error = factor.unwhitenedError(landmarkEstimates);
      if (k <= 200 || std::abs(error[0]) < 5)
        newFactors.push_back(factor);
      k = k + 1;
      countK = countK + 1;
    }

    // Check whether to update iSAM 2
    if (countK > incK) {
      gttic_(update);
      isam.update(newFactors, initial);
      gttoc_(update);
      gttic_(calculateEstimate);
      Values result = isam.calculateEstimate();
      gttoc_(calculateEstimate);
      lastPose = result.at<Pose2>(i);
      // update landmark estimates
      landmarkEstimates = Values();
      landmarkEstimates.insert(symbol('L', 1), result.at(symbol('L', 1)));
      landmarkEstimates.insert(symbol('L', 6), result.at(symbol('L', 6)));
      landmarkEstimates.insert(symbol('L', 0), result.at(symbol('L', 0)));
      landmarkEstimates.insert(symbol('L', 5), result.at(symbol('L', 5)));
      newFactors = NonlinearFactorGraph();
      initial = Values();
      countK = 0;
    }
    i += 1;
    //--------------------------------- odometry loop -----------------------------------------
  } // end for
  gttoc_(iSAM);

  // Print timings
  tictoc_print_();

  // Write result to file
  Values result = isam.calculateEstimate();
  ofstream os2("rangeResultLM.txt");
  for(const auto it: result.filter<Point2>())
    os2 << it.key << "\t" << it.value.x() << "\t" << it.value.y() << "\t1"
        << endl;
  ofstream os("rangeResult.txt");
  for(const auto it: result.filter<Pose2>())
    os << it.key << "\t" << it.value.x() << "\t" << it.value.y() << "\t"
        << it.value.theta() << endl;
  exit(0);
}

