/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  TimeOfArrivalExample.cpp
 *  @brief Track a moving object "Time of Arrival" measurements at 4
 * microphones.
 *  @author Frank Dellaert
 *  @author Jay Chakravarty
 *  @date March 2020
 */

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam_unstable/geometry/Event.h>
#include <gtsam_unstable/slam/TOAFactor.h>

#include <boost/format.hpp>

#include <vector>

using namespace std;
using namespace gtsam;

// units
static const double ms = 1e-3;
static const double cm = 1e-2;

// Instantiate functor with speed of sound value
static const TimeOfArrival kTimeOfArrival(330);

/* ************************************************************************* */
// Create microphones
vector<Point3> defineMicrophones() {
  const double height = 0.5;
  vector<Point3> microphones;
  microphones.push_back(Point3(0, 0, height));
  microphones.push_back(Point3(403 * cm, 0, height));
  microphones.push_back(Point3(403 * cm, 403 * cm, height));
  microphones.push_back(Point3(0, 403 * cm, 2 * height));
  return microphones;
}

/* ************************************************************************* */
// Create ground truth trajectory
vector<Event> createTrajectory(size_t n) {
  vector<Event> trajectory;
  double timeOfEvent = 10;
  // simulate emitting a sound every second while moving on straight line
  for (size_t key = 0; key < n; key++) {
    trajectory.push_back(
        Event(timeOfEvent, 245 * cm + key * 1.0, 201.5 * cm, (212 - 45) * cm));
    timeOfEvent += 1;
  }
  return trajectory;
}

/* ************************************************************************* */
// Simulate time-of-arrival measurements for a single event
vector<double> simulateTOA(const vector<Point3>& microphones,
                           const Event& event) {
  size_t K = microphones.size();
  vector<double> simulatedTOA(K);
  for (size_t i = 0; i < K; i++) {
    simulatedTOA[i] = kTimeOfArrival(event, microphones[i]);
  }
  return simulatedTOA;
}

/* ************************************************************************* */
// Simulate time-of-arrival measurements for an entire trajectory
vector<vector<double>> simulateTOA(const vector<Point3>& microphones,
                                   const vector<Event>& trajectory) {
  vector<vector<double>> simulatedTOA;
  for (auto event : trajectory) {
    simulatedTOA.push_back(simulateTOA(microphones, event));
  }
  return simulatedTOA;
}

/* ************************************************************************* */
// create factor graph
NonlinearFactorGraph createGraph(const vector<Point3>& microphones,
                                 const vector<vector<double>>& simulatedTOA) {
  NonlinearFactorGraph graph;

  // Create a noise model for the TOA error
  auto model = noiseModel::Isotropic::Sigma(1, 0.5 * ms);

  size_t K = microphones.size();
  size_t key = 0;
  for (auto toa : simulatedTOA) {
    for (size_t i = 0; i < K; i++) {
      graph.emplace_shared<TOAFactor>(key, microphones[i], toa[i], model);
    }
    key += 1;
  }
  return graph;
}

/* ************************************************************************* */
// create initial estimate for n events
Values createInitialEstimate(size_t n) {
  Values initial;

  Event zero;
  for (size_t key = 0; key < n; key++) {
    initial.insert(key, zero);
  }
  return initial;
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  // Create microphones
  auto microphones = defineMicrophones();
  size_t K = microphones.size();
  for (size_t i = 0; i < K; i++) {
    cout << "mic" << i << " = " << microphones[i] << endl;
  }

  // Create a ground truth trajectory
  const size_t n = 5;
  auto groundTruth = createTrajectory(n);

  // Simulate time-of-arrival measurements
  auto simulatedTOA = simulateTOA(microphones, groundTruth);
  for (size_t key = 0; key < n; key++) {
    for (size_t i = 0; i < K; i++) {
      cout << "z_" << key << i << " = " << simulatedTOA[key][i] / ms << " ms"
           << endl;
    }
  }

  // Create factor graph
  auto graph = createGraph(microphones, simulatedTOA);

  // Create initial estimate
  auto initialEstimate = createInitialEstimate(n);
  initialEstimate.print("Initial Estimate:\n");

  // Optimize using Levenberg-Marquardt optimization.
  LevenbergMarquardtParams params;
  params.setAbsoluteErrorTol(1e-10);
  params.setVerbosityLM("SUMMARY");
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");
}
/* ************************************************************************* */
