/**
 * @file    testEasyExample.cpp
 * @brief   Unit tests for easyExample file
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/numericalDerivative.h>

#include <iostream>

#include "easyExample.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(easyExample, createExample) {
  Cal3_S2 K(625, 625, 0, 0, 0);
  double markerSize = 160.0;
  EasySLAMGraph fg = createExampleGraph();

  // compare factor 1
  ARRobotMarker markerMeasurement(0, 1, -100, 100, -100, -100, 100, -100, 100,
                                  100, markerSize, K);
  CameraMarkerFactor0 factor(markerMeasurement, 1.0);
  CameraMarkerFactor::shared_ptr cmf =
      boost::static_pointer_cast<CameraMarkerFactor>(fg[0]);
  CHECK(factor.equals(*cmf, 0.1));
}

/* ************************************************************************* */
Pose3 prior_pose(const Pose3& x) { return x; }
TEST(easyExample, pose_prior) {
  // create a nonlinear factor that is a pose prior
  Pose3 origin;
  boost::shared_ptr<NonlinearFactor1> prior =
      createPosePrior(origin, 0.1, "x0");

  // verify that error is zero
  FGConfig config = createExampleConfig();
  // DOUBLES_EQUAL(prior->error(config), 0, 0.0001);

  // check the error vector itself
  // CHECK(zero(prior->error_vector(config)));

  // check the size of the measurement
  // CHECK(prior->get_measurement().size() == 12);

  // derivative should be identity matrix
  Matrix Dp = prior->H_(config["x0"]);
  Matrix numerical = numericalDerivative11(prior_pose, origin, 1e-5);
  CHECK(assert_equal(Dp, numerical));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  TestRegistry::runAllTests(tr);
  return 0;
}
/* ************************************************************************* */
