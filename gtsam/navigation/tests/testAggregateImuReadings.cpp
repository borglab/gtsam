/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInertialNavFactor.cpp
 * @brief   Unit test for the InertialNavFactor
 * @author  Frank Dellaert
 */

#include <gtsam/navigation/AggregateImuReadings.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

static const double kDt = 0.1;

static const double kGyroSigma = 0.02;
static const double kAccelSigma = 0.1;

// Create default parameters with Z-down and above noise parameters
static boost::shared_ptr<AggregateImuReadings::Params> defaultParams() {
  auto p = PreintegratedImuMeasurements::Params::MakeSharedD(10);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0000001 * I_3x3;
  return p;
}

/* ************************************************************************* */
TEST(AggregateImuReadings, UpdateEstimate) {
  AggregateImuReadings pim(defaultParams());
  Matrix9 aH1;
  Matrix93 aH2, aH3;
  boost::function<Vector9(const Vector9&, const Vector3&, const Vector3&)> f =
      boost::bind(&AggregateImuReadings::UpdateEstimate, _1, _2, _3, kDt,
                  boost::none, boost::none, boost::none);
  Vector9 zeta;
  zeta << 0.01, 0.02, 0.03, 100, 200, 300, 10, 5, 3;
  const Vector3 acc(0.1, 0.2, 10), omega(0.1, 0.2, 0.3);
  pim.UpdateEstimate(zeta, acc, omega, kDt, aH1, aH2, aH3);
  EXPECT(assert_equal(numericalDerivative31(f, zeta, acc, omega), aH1, 1e-3));
  EXPECT(assert_equal(numericalDerivative32(f, zeta, acc, omega), aH2, 1e-5));
  EXPECT(assert_equal(numericalDerivative33(f, zeta, acc, omega), aH3, 1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
