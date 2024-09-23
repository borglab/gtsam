/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testEvent.cpp
 *  @brief Unit tests for space time "Event"
 *  @author Frank Dellaert
 *  @author Jay Chakravarty
 *  @date December 2014
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam_unstable/geometry/Event.h>

#include <CppUnitLite/TestHarness.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// Create a noise model for the TOA error
static const double ms = 1e-3;
static const double cm = 1e-2;
typedef Eigen::Matrix<double, 1, 1> Vector1;
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(1, 0.5 * ms));

static const double timeOfEvent = 25;
static const Event exampleEvent(timeOfEvent, 1, 0, 0);
static const Point3 microphoneAt0(0, 0, 0);

static const double kSpeedOfSound = 340;
static const TimeOfArrival kToa(kSpeedOfSound);

//*****************************************************************************
TEST(Event, Constructor) {
  const double t = 0;
  Event actual(t, 201.5 * cm, 201.5 * cm, (212 - 45) * cm);
}

//*****************************************************************************
TEST(Event, Toa1) {
  Event event(0, 1, 0, 0);
  double expected = 1. / kSpeedOfSound;
  EXPECT_DOUBLES_EQUAL(expected, kToa(event, microphoneAt0), 1e-9);
}

//*****************************************************************************
TEST(Event, Toa2) {
  double expectedTOA = timeOfEvent + 1. / kSpeedOfSound;
  EXPECT_DOUBLES_EQUAL(expectedTOA, kToa(exampleEvent, microphoneAt0), 1e-9);
}

//*************************************************************************
TEST(Event, Derivatives) {
  Matrix14 actualH1;
  Matrix13 actualH2;
  kToa(exampleEvent, microphoneAt0, actualH1, actualH2);
  Matrix expectedH1 = numericalDerivative11<double, Event>(
      std::bind(kToa, std::placeholders::_1, microphoneAt0, nullptr, nullptr),
      exampleEvent);
  EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
  Matrix expectedH2 = numericalDerivative11<double, Point3>(
      std::bind(kToa, exampleEvent, std::placeholders::_1, nullptr, nullptr),
      microphoneAt0);
  EXPECT(assert_equal(expectedH2, actualH2, 1e-8));
}

//*****************************************************************************
TEST(Event, Expression) {
  Key key = 12;
  Expression<Event> event_(key);
  Expression<Point3> knownMicrophone_(microphoneAt0);  // constant expression
  Expression<double> expression(kToa, event_, knownMicrophone_);

  Values values;
  values.insert(key, exampleEvent);
  double expectedTOA = timeOfEvent + 1. / kSpeedOfSound;
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
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*****************************************************************************
