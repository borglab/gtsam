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

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam_unstable/nonlinear/ExpressionFactor.h>
#include <gtsam/geometry/Point3.h>
#include <cmath>

namespace gtsam {

/// A space-time event
class Event {

  double time_; ///< Time event was generated
  Point3 location_; ///< Location at time event was generated

public:

  /// Speed of sound
  static const double Speed;

  /// Default Constructor
  Event() :
      time_(0) {
  }

  /// Constructor from time and location
  Event(double t, const Point3& p) :
      time_(t), location_(p) {
  }

  /// Constructor with doubles
  Event(double t, double x, double y, double z) :
      time_(t), location_(x, y, z) {
  }

  /** print with optional string */
  void print(const std::string& s = "") const {
    std::cout << s << "time = " << time_;
    location_.print("; location");
  }

  /** equals with an tolerance */
  bool equals(const Event& other, double tol = 1e-9) const {
    return std::abs(time_ - other.time_) < tol
        && location_.equals(other.location_, tol);
  }

  /// Manifold stuff:

  size_t dim() const {
    return 4;
  }
  static size_t Dim() {
    return 4;
  }

  /// Updates a with tangent space delta
  inline Event retract(const Vector4& v) const {
    return Event(time_ + v[0], location_.retract(v.tail(3)));
  }

  /// Returns inverse retraction
  inline Vector4 localCoordinates(const Event& q) const {
    return Vector4::Zero(); // TODO
  }

  /// Time of arrival to given microphone
  double toa(const Point3& microphone, //
      OptionalJacobian<1, 4> H1 = boost::none, //
      OptionalJacobian<1, 3> H2 = boost::none) const {
    Matrix13 D1, D2;
    double distance = location_.distance(microphone, D1, D2);
    if (H1)
      // derivative of toa with respect to event
      *H1 << 1.0, D1 / Speed;
    if (H2)
      // derivative of toa with respect to microphone location
      *H2 << D2 / Speed;
    return time_ + distance / Speed;
  }
};

const double Event::Speed = 330;

// Define GTSAM traits
namespace traits {

template<>
struct GTSAM_EXPORT dimension<Event> : public boost::integral_constant<int, 4> {
};

template<>
struct GTSAM_EXPORT is_manifold<Event> : public boost::true_type {
};

}

/// A "Time of Arrival" factor
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

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

// Create a noise model for the TOA error
//static const double ms = 1e-3;
static const double cm = 1e-2;
typedef Eigen::Matrix<double, 1, 1> Vector1;
static SharedNoiseModel model(noiseModel::Unit::Create(1));

static const double timeOfEvent = 25;
static const Event exampleEvent(timeOfEvent, 1, 0, 0);
static const Point3 microphoneAt0;

//*****************************************************************************
TEST( Event, Constructor ) {
  const double t = 0;
  Event actual(t, 201.5 * cm, 201.5 * cm, (212 - 45) * cm);
}

//*****************************************************************************
TEST( Event, Toa1 ) {
  Event event(0, 1, 0, 0);
  double expected = 1 / Event::Speed;
  EXPECT_DOUBLES_EQUAL(expected, event.toa(microphoneAt0), 1e-9);
}

//*****************************************************************************
TEST( Event, Toa2 ) {
  double expectedTOA = timeOfEvent + 1 / Event::Speed;
  EXPECT_DOUBLES_EQUAL(expectedTOA, exampleEvent.toa(microphoneAt0), 1e-9);
}

//*************************************************************************
TEST (Event, Derivatives) {
  Matrix14 actualH1;
  Matrix13 actualH2;
  exampleEvent.toa(microphoneAt0, actualH1, actualH2);
  Matrix expectedH1 = numericalDerivative11<double, Event>(
      boost::bind(&Event::toa, _1, microphoneAt0, boost::none, boost::none),
      exampleEvent);
  EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
  Matrix expectedH2 = numericalDerivative11<double, Point3>(
      boost::bind(&Event::toa, exampleEvent, _1, boost::none, boost::none),
      microphoneAt0);
  EXPECT(assert_equal(expectedH2, actualH2, 1e-8));
}

//*****************************************************************************
TEST( Event, Expression ) {
  Key key = 12;
  Expression<Event> event_(key);
  Expression<Point3> knownMicrophone_(microphoneAt0); // constant expression
  Expression<double> expression(&Event::toa, event_, knownMicrophone_);

  Values values;
  values.insert(key, exampleEvent);
  double expectedTOA = timeOfEvent + 1 / Event::Speed;
  EXPECT_DOUBLES_EQUAL(expectedTOA, expression.value(values), 1e-9);
}

//*****************************************************************************
TEST(Event, Retract) {
  Event event, expected(1, 2, 3, 4);
  Vector4 v;
  v << 1, 2, 3, 4;
  EXPECT(assert_equal(expected, event.retract(v)));
}

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

  // Create microphones
  vector<Point3> microphones;
  microphones.push_back(Point3(0, 0, 0));
  microphones.push_back(Point3(403 * cm, 0, 0));
  microphones.push_back(Point3(403 * cm, 403 * cm, 0));
  microphones.push_back(Point3(0, 403 * cm, 0));
  EXPECT_LONGS_EQUAL(4, microphones.size());

  // Create a ground truth point
  const double timeOfEvent = 0;
  Event groundTruthEvent(timeOfEvent, 201.5 * cm, 201.5 * cm, (212 - 45) * cm);

  // Simulate measurements
  vector<double> measurements(4);
  for (size_t i = 0; i < 4; i++)
    measurements[i] = groundTruthEvent.toa(microphones[i]);

  // Now, estimate using non-linear optimization
  NonlinearFactorGraph graph;
  Key key = 12;
  Expression<Event> event_(key);
  for (size_t i = 0; i < 4; i++) {
    Expression<Point3> knownMicrophone_(microphones[i]); // constant expression
    graph.add(TOAFactor(event_, knownMicrophone_, measurements[i], model));
  }

  /// Print the graph
//  GTSAM_PRINT(graph);

  // Create initial estimate
  Values initialEstimate;
  Event estimatedEvent(timeOfEvent + 10, 200 * cm, 150 * cm, 50 * cm);
  initialEstimate.insert(key, estimatedEvent);

  // Print
  initialEstimate.print("Initial Estimate:\n");

  // Optimize using Levenberg-Marquardt optimization.
  LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-10);
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();

  result.print("Final Result:\n");
  EXPECT(assert_equal(groundTruthEvent, result.at<Event>(key), 1e-6));
}

//*****************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*****************************************************************************

