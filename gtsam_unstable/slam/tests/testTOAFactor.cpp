/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testTOAFactor.cpp
 *  @brief Unit tests for "Time of Arrival" factor
 *  @author Frank Dellaert
 *  @author Jay Chakravarty
 *  @date December 2014
 */

#include <gtsam_unstable/slam/TOAFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/format.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

// Create a noise model for the TOA error
static const double ms = 1e-3;
static const double cm = 1e-2;
typedef Eigen::Matrix<double, 1, 1> Vector1;
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(1, 0.5 * ms));

static const double timeOfEvent = 25;
static const Event exampleEvent(timeOfEvent, 1, 0, 0);
static const Point3 microphoneAt0;

//*****************************************************************************
TEST( TOAFactor, Construct ) {
  Key key = 12;
  Expression<Event> eventExpression(key);
  Expression<Point3> knownMicrophone_(microphoneAt0); // constant expression
  double measurement = 7;
  TOAFactor factor(eventExpression, knownMicrophone_, measurement, model);
}

//*****************************************************************************
TEST( TOAFactor, WholeEnchilada ) {

  static const bool verbose = false;

  // Create microphones
  const double height = 0.5;
  vector<Point3> microphones;
  microphones.push_back(Point3(0, 0, height));
  microphones.push_back(Point3(403 * cm, 0, height));
  microphones.push_back(Point3(403 * cm, 403 * cm, height));
  microphones.push_back(Point3(0, 403 * cm, height));
  EXPECT_LONGS_EQUAL(4, microphones.size());
  microphones.push_back(Point3(200 * cm, 200 * cm, height));

  // Create a ground truth point
  const double timeOfEvent = 0;
  Event groundTruthEvent(timeOfEvent, 245 * cm, 201.5 * cm, (212 - 45) * cm);

  // Simulate measurements
  size_t K = microphones.size();
  vector<double> measurements(K);
  for (size_t i = 0; i < K; i++) {
    measurements[i] = groundTruthEvent.toa(microphones[i]);
    if (verbose) {
      cout << "mic" << i << " = " << microphones[i] << endl;
      cout << "z" << i << " = " << measurements[i] / ms << endl;
    }
  }

  // Now, estimate using non-linear optimization
  NonlinearFactorGraph graph;
  Key key = 12;
  Expression<Event> eventExpression(key);
  for (size_t i = 0; i < K; i++) {
    Expression<Point3> knownMicrophone_(microphones[i]); // constant expression
    graph.add(
        TOAFactor(eventExpression, knownMicrophone_, measurements[i], model));
  }

  /// Print the graph
  if (verbose)
    GTSAM_PRINT(graph);

  // Create initial estimate
  Values initialEstimate;
  //Event estimatedEvent(timeOfEvent -10, 200 * cm, 150 * cm, 350 * cm);
  Vector4 delta;
  delta << 0.1, 0.1, -0.1, 0.1;
  Event estimatedEvent = groundTruthEvent.retract(delta);
  initialEstimate.insert(key, estimatedEvent);

  // Print
  if (verbose)
    initialEstimate.print("Initial Estimate:\n");

  // Optimize using Levenberg-Marquardt optimization.
  LevenbergMarquardtParams params;
  params.setAbsoluteErrorTol(1e-10);
  if (verbose)
    params.setVerbosity("ERROR");
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
  Values result = optimizer.optimize();
  if (verbose)
    result.print("Final Result:\n");

  EXPECT(assert_equal(groundTruthEvent, result.at<Event>(key), 1e-6));
}
//*****************************************************************************
/// Test real data
TEST( TOAFactor, RealExperiment1 ) {

  static const bool verbose = false;

  // Create microphones
  const double height = 0.5;
  vector<Point3> microphones;
  microphones.push_back(Point3(0, 0, height));
  microphones.push_back(Point3(403 * cm, 0, height));
  microphones.push_back(Point3(403 * cm, 403 * cm, height));
  microphones.push_back(Point3(0, 403 * cm, height));
  EXPECT_LONGS_EQUAL(4, microphones.size());

  vector<Vector4> data(15);
  size_t i = 0;
  data[i++] << 1.2648, 1.2648, 1.2677, 1.2643;
  data[i++] << 1.7329, 1.7347, 1.7354, 1.7338;
  data[i++] << 2.2475, 2.2551, 2.2538, 2.2474;
  data[i++] << 2.6945, 2.696, 2.6958, 2.694;
  data[i++] << 3.1486, 3.152, 3.1513, 3.1501;
  data[i++] << 3.6145, 3.611, 3.6076, 3.6067;
  data[i++] << 4.1003, 4.1004, 4.099, 4.0972;
  data[i++] << 4.5732, 4.568, 4.5667, 4.5722;
  data[i++] << 5.0482, 5.0458, 5.0443, 5.0453;
  data[i++] << 5.5311, 5.5256, 5.5254, 5.5305;
  data[i++] << 5.9908, 5.9856, 5.9853, 5.9905;
  data[i++] << 6.4575, 6.4524, 6.4527, 6.4579;
  data[i++] << 6.8983, 6.8971, 6.8984, 6.9016;
  data[i++] << 7.3581, 7.3524, 7.3538, 7.3588;
  data[i++] << 7.8286, 7.8286, 7.8302, 7.8353;

  // Create unknowns and initial estimate
  Event nullEvent(3, 403 / 2 * cm, 403 / 2 * cm, (212 - 45) * cm);
  Values initialEstimate;
  vector<Expression<Event> > eventExpressions;
  for (size_t j = 0; j < 15; j++) {
    initialEstimate.insert(j, nullEvent);
    eventExpressions.push_back(Expression<Event>(j));
  }

  // Print
  if (verbose)
    initialEstimate.print("Initial Estimate:\n");

  // Create factor graph and initial estimate
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < 4; i++) {
    Expression<Point3> mic_(microphones[i]); // constant expression
    for (size_t j = 0; j < 15; j++)
      graph.add(TOAFactor(eventExpressions[j], mic_, data[j][i], model));
  }

  /// Print the graph
  if (verbose)
    GTSAM_PRINT(graph);

  // Optimize using Levenberg-Marquardt optimization.
  LevenbergMarquardtParams params;
  params.setAbsoluteErrorTol(1e-10);
  if (verbose)
    params.setVerbosity("ERROR");
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
  Values result = optimizer.optimize();
  if (verbose)
    for (size_t j = 0; j < 15; j++) {
      Event event = result.at<Event>(j);
      double t = event.time();
      Point3 p = event.location();
      cout
          << boost::format("t(%1%) = %2%;\tlocation(%1%,:) = [%3%, %4%, %5%];")
              % (j + 1) % t % p.x() % p.y() % p.z() << endl;
    }
}

//*****************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*****************************************************************************

