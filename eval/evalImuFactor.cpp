/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   evalImuFactor.cpp
 * @brief  Evaluations for ImuFactor
 * @author Porter Zach
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/debug.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/navigation/tests/imuFactorTesting.h>

using namespace std;
using namespace gtsam;

// IMU frequency for EuRoC data.
static const double kImuFreq = 200.0;

/**
 * This test suite uses the EuRoC MAV dataset, published in:
 * M. Burri, J. Nikolic, P. Gohl, T. Schneider, J. Rehder, S. Omari, M. Achtelik 
 * and R. Siegwart, The EuRoC micro aerial vehicle datasets, International 
 * Journal of Robotic Research, DOI: 10.1177/0278364915620033, early 2016.
 * 
 * Data download available at:
 * https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
 */

/* ************************************************************************* */
TEST(ImuFactor, Accelerating) {
  const double a = 0.2, v = 50;

  // Set up body pointing towards y axis, and start at 10,20,0 with velocity
  // going in X The body itself has Z axis pointing down
  const Rot3 nRb(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
  const Point3 initial_position(10, 20, 0);
  const Vector3 initial_velocity(v, 0, 0);

  const AcceleratingScenario scenario(nRb, initial_position, initial_velocity,
                                      Vector3(a, 0, 0));

  const double T = 3.0;  // seconds
  ScenarioRunner runner(scenario, testing::Params(), T / 10);

  PreintegratedImuMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.navState(T), runner.predict(pim), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 1000);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.01));
}

/* ************************************************************************* */
TEST(ImuFactor, EurocDataEasy) {
  const string data_path = ".\\data\\euroc\\euroc_MH01.csv";

  const DiscreteScenario scenario = DiscreteScenario::FromCSV(data_path);

  ScenarioRunner runner(scenario, testing::Params(), 1.0 / kImuFreq);

  PreintegratedImuMeasurements pim = runner.integrate(scenario.duration());
  EXPECT(assert_equal(scenario.navState(scenario.duration()), runner.predict(pim), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(scenario.duration(), 1000);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.01));
}

/* ************************************************************************* */
TEST(ImuFactor, EurocDataHard) {
  const string data_path = ".\\data\\euroc\\euroc_V202.csv";

  const DiscreteScenario scenario = DiscreteScenario::FromCSV(data_path);

  ScenarioRunner runner(scenario, testing::Params(), 1.0 / kImuFreq);

  PreintegratedImuMeasurements pim = runner.integrate(scenario.duration());
  EXPECT(assert_equal(scenario.navState(scenario.duration()), runner.predict(pim), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(scenario.duration(), 1000);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.01));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
