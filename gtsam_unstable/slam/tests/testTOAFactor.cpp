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

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam_unstable/geometry/Event.h>
#include <gtsam_unstable/slam/TOAFactor.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace gtsam;

// typedefs
typedef Expression<Point3> Point3_;
typedef Expression<Event> Event_;

// units
static const double ms = 1e-3;
static const double cm = 1e-2;

// Create a noise model for the TOA error
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(1, 0.5 * ms));

static const double timeOfEvent = 25;
static const Event exampleEvent(timeOfEvent, 1, 0, 0);
static const Point3 sensorAt0(0, 0, 0);

//*****************************************************************************
TEST(TOAFactor, NewWay) {
  Key key = 12;
  double measurement = 7;
  TOAFactor factor(key, sensorAt0, measurement, model);
}

//*****************************************************************************
TEST(TOAFactor, WholeEnchilada) {
  // Create sensors
  const double height = 0.5;
  vector<Point3> sensors;
  sensors.push_back(Point3(0, 0, height));
  sensors.push_back(Point3(403 * cm, 0, height));
  sensors.push_back(Point3(403 * cm, 403 * cm, height));
  sensors.push_back(Point3(0, 403 * cm, 2 * height));
  EXPECT_LONGS_EQUAL(4, sensors.size());
  //  sensors.push_back(Point3(200 * cm, 200 * cm, height));

  // Create a ground truth point
  const double timeOfEvent = 0;
  Event groundTruthEvent(timeOfEvent, 245 * cm, 201.5 * cm, (212 - 45) * cm);

  // Simulate simulatedTOA
  size_t K = sensors.size();
  vector<double> simulatedTOA(K);
  TimeOfArrival toa;
  for (size_t i = 0; i < K; i++) {
    simulatedTOA[i] = toa(groundTruthEvent, sensors[i]);
  }

  // Now, estimate using non-linear optimization
  NonlinearFactorGraph graph;
  Key key = 12;
  for (size_t i = 0; i < K; i++) {
    graph.emplace_shared<TOAFactor>(key, sensors[i], simulatedTOA[i], model);
  }

  // Create initial estimate
  Values initialEstimate;
  // Event estimatedEvent(timeOfEvent -10, 200 * cm, 150 * cm, 350 * cm);
  Vector4 delta;
  delta << 0.1, 0.1, -0.1, 0.1;
  Event estimatedEvent = groundTruthEvent.retract(delta);
  initialEstimate.insert(key, estimatedEvent);

  // Optimize using Levenberg-Marquardt optimization.
  LevenbergMarquardtParams params;
  params.setAbsoluteErrorTol(1e-10);
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
  Values result = optimizer.optimize();

  EXPECT(assert_equal(groundTruthEvent, result.at<Event>(key), 1e-6));
}
//*****************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*****************************************************************************
