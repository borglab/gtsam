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

#include <gtsam_unstable/nonlinear/ExpressionFactor.h>
#include <gtsam_unstable/geometry/Event.h>

namespace gtsam {

/// A "Time of Arrival" factor - so little code seems hardly worth it :-)
class TOAFactor: public ExpressionFactor<double> {

  typedef Expression<double> double_;

public:

  /**
   * Constructor
   * @param some expression yielding an event
   * @param microphone_ expression yielding a microphone location
   * @param toaMeasurement time of arrival at microphone
   * @param model noise model
   */
  TOAFactor(const Expression<Event>& event_,
      const Expression<Point3>& microphone_, double toaMeasurement,
      const SharedNoiseModel& model) :
      ExpressionFactor<double>(model, toaMeasurement,
          double_(&Event::toa, event_, microphone_)) {
  }

};

} //\ namespace gtsam

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

// Create a noise model for the TOA error
static const double ms = 1e-3;
static const double cm = 1e-2;
typedef Eigen::Matrix<double, 1, 1> Vector1;
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(1,0.5*ms));

static const double timeOfEvent = 25;
static const Event exampleEvent(timeOfEvent, 1, 0, 0);
static const Point3 microphoneAt0;

//*****************************************************************************
TEST( TOAFactor, Construct ) {
  Key key = 12;
  Expression<Event> event_(key);
  Expression<Point3> knownMicrophone_(microphoneAt0); // constant expression
  double measurement = 7;
  TOAFactor factor(event_, knownMicrophone_, measurement, model);
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
  microphones.push_back(Point3(200*cm, 200 * cm, height));

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
      cout << "z" << i << " = " << measurements[i]/ms << endl;
    }
  }

  // Now, estimate using non-linear optimization
  NonlinearFactorGraph graph;
  Key key = 12;
  Expression<Event> event_(key);
  for (size_t i = 0; i < K; i++) {
    Expression<Point3> knownMicrophone_(microphones[i]); // constant expression
    graph.add(TOAFactor(event_, knownMicrophone_, measurements[i], model));
  }

  /// Print the graph
  if (verbose)
    GTSAM_PRINT(graph);

  // Create initial estimate
  Values initialEstimate;
  //Event estimatedEvent(timeOfEvent -10, 200 * cm, 150 * cm, 350 * cm);
  Vector4 delta; delta << 0.1, 0.1, -0.1, 0.1;
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
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*****************************************************************************

