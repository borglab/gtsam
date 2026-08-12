/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testBinaryMeasurement.cpp
 *  @brief Unit tests for BinaryMeasurement class
 *  @author Akshay Krishnan
 *  @date July 2020
 */

#include <gtsam/sfm/BinaryMeasurement.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Pose3.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

static const Key kKey1(2), kKey2(1);

// Create noise models for unit3 and rot3
static SharedNoiseModel unit3_model(noiseModel::Isotropic::Sigma(2, 0.05));
static SharedNoiseModel rot3_model(noiseModel::Isotropic::Sigma(3, 0.05));

const Unit3 unit3Measured(Vector3(1, 1, 1));
const Rot3 rot3Measured;

/* ************************************************************************* */
TEST(BinaryMeasurement, Unit3) {
  BinaryMeasurement<Unit3> unit3Measurement(kKey1, kKey2, unit3Measured,
                                            unit3_model);
  EXPECT_LONGS_EQUAL(unit3Measurement.key1(), kKey1);
  EXPECT_LONGS_EQUAL(unit3Measurement.key2(), kKey2);
  EXPECT(unit3Measurement.measured().equals(unit3Measured));

  BinaryMeasurement<Unit3> unit3MeasurementCopy(kKey1, kKey2, unit3Measured,
                                                unit3_model);
  EXPECT(unit3Measurement.equals(unit3MeasurementCopy));
}

/* ************************************************************************* */
TEST(BinaryMeasurement, Rot3) {
  // testing the accessors
  BinaryMeasurement<Rot3> rot3Measurement(kKey1, kKey2, rot3Measured,
                                          rot3_model);
  EXPECT_LONGS_EQUAL(rot3Measurement.key1(), kKey1);
  EXPECT_LONGS_EQUAL(rot3Measurement.key2(), kKey2);
  EXPECT(rot3Measurement.measured().equals(rot3Measured));

  // testing the equals function
  BinaryMeasurement<Rot3> rot3MeasurementCopy(kKey1, kKey2, rot3Measured,
                                              rot3_model);
  EXPECT(rot3Measurement.equals(rot3MeasurementCopy));
}

/* ************************************************************************* */
TEST(BinaryMeasurement, Rot3MakeRobust) {
  auto huber_model = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(1.345), rot3_model);
  BinaryMeasurement<Rot3> rot3Measurement(kKey1, kKey2, rot3Measured,
                                          huber_model);

  EXPECT_LONGS_EQUAL(rot3Measurement.key1(), kKey1);
  EXPECT_LONGS_EQUAL(rot3Measurement.key2(), kKey2);
  EXPECT(rot3Measurement.measured().equals(rot3Measured));
  const auto &robust = std::dynamic_pointer_cast<noiseModel::Robust>(
      rot3Measurement.noiseModel());
  EXPECT(robust);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
