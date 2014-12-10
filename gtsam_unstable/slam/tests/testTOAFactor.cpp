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
#include <gtsam/geometry/Point3.h>

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

  /// Constructor
  Event(double t, double x, double y, double z) :
      time_(t), location_(x, y, z) {
  }

  /// Time of arrival to given microphone
  double toa(const Point3& microphone, OptionalJacobian<1, 4> H1 = boost::none,
      OptionalJacobian<1, 3> H2 = boost::none) const {
    if (H1) {

    }
    return time_ + location_.distance(microphone) / Speed;
  }
};

const double Event::Speed = 330;

// Define GTSAM traits
namespace traits {

template<>
struct GTSAM_EXPORT dimension<Event> : public boost::integral_constant<int, 4> {
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

}//\ namespace gtsam

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

// Create a noise model for the TOA error
static const double ms = 1e-3, cm = 1e-2;
typedef Eigen::Matrix<double, 1, 1> Vector1;
static SharedNoiseModel model(noiseModel::Diagonal::Sigmas(Vector1(5. * ms)));

//*****************************************************************************
TEST( Event, Constructor ) {
  const double t = 0;
  Event actual(t, 201.5 * cm, 201.5 * cm, (212 - 45) * cm);
}

//*****************************************************************************
TEST( TOA, Toa1 ) {
  Point3 microphone;
  Event event(0, 1, 0, 0);
  double expected = 1 / Event::Speed;
  EXPECT_DOUBLES_EQUAL(expected, event.toa(microphone), 1e-9);
}

//*****************************************************************************
TEST( TOA, Toa2 ) {
  Point3 microphone;
  double timeOfEvent = 25;
  Event event(timeOfEvent, 1, 0, 0);
  double expectedTOA = timeOfEvent + 1 / Event::Speed;
  EXPECT_DOUBLES_EQUAL(expectedTOA, event.toa(microphone), 1e-9);
}

//*****************************************************************************
TEST( TOA, Expression ) {
  Key key = 12;
  Expression<Event> event_(key);
  Point3 microphone;
  Expression<Point3> knownMicrophone_(microphone); // constant expression
  Expression<double> expression(&Event::toa, event_, knownMicrophone_);

//  double timeOfEvent = 25;
//  Event event12(timeOfEvent, 1, 0, 0);
//  Values values;
//  values.insert(key,event12);
//  double expectedTOA = timeOfEvent + 1 / Event::Speed;
//  EXPECT_DOUBLES_EQUAL(expectedTOA, expression.value(values), 1e-9);
}

//*****************************************************************************
TEST( TOAFactor, Constract ) {
  Key key = 12;
  Expression<Event> event_(key);
  Point3 microphone;
  Expression<Point3> knownMicrophone_(microphone); // constant expression
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
  Event event(timeOfEvent, 201.5 * cm, 201.5 * cm, (212 - 45) * cm);

  // Simulate measurements
  vector<double> measurements(4);
  for (size_t i = 0; i < 4; i++)
    measurements[i] = event.toa(microphones[i]);

  // Now, estimate using non-linear optimization
  NonlinearFactorGraph graph;
  Key key = 12;
  Expression<Event> event_(key);
  for (size_t i = 0; i < 4; i++) {
    Expression<Point3> knownMicrophone_(microphones[i]); // constant expression
    graph.add(TOAFactor(event_, knownMicrophone_, measurements[i], model));
  }
}

//*****************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*****************************************************************************

