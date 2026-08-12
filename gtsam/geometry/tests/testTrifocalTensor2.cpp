/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testTrifocalTensor2.cpp
 * @brief   Tests for the trifocal tensor class.
 * @author  Zhaodong Yang
 * @author  Akshay Krishnan
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/TrifocalTensor2.h>

#include <cmath>
#include <stdexcept>
#include <vector>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(TrifocalTensor2)

/* ************************************************************************* */
namespace trifocal_tensor2_fixture {

struct TrifocalTestData {
  std::vector<Pose2> poses;
  std::vector<Point2> landmarks;

  // Outer vector over poses.
  std::vector<std::vector<Rot2>> measurements;
};

TrifocalTestData getTestData() {
  TrifocalTestData data;

  data.poses.emplace_back(0, 0, 0);
  data.poses.emplace_back(-1.9, 4, -2 * std::acos(0.0) / 8);
  data.poses.emplace_back(2.1, -2.1, 2 * std::acos(0.0) / 3);

  data.landmarks.emplace_back(1.2, 1.0);
  data.landmarks.emplace_back(2.4, 3.5);
  data.landmarks.emplace_back(-1.0, 0.5);
  data.landmarks.emplace_back(3.4, -1.5);
  data.landmarks.emplace_back(5.1, 0.6);
  data.landmarks.emplace_back(-0.1, -0.7);
  data.landmarks.emplace_back(3.1, 1.9);

  for (const Pose2& pose : data.poses) {
    std::vector<Rot2> measurements;
    for (const Point2& landmark : data.landmarks) {
      measurements.push_back(pose.bearing(landmark));
    }
    data.measurements.push_back(measurements);
  }
  return data;
}

// Checks that transfer from two views recovers the bearing in the third view.
TEST(TrifocalTensor2, transform) {
  const TrifocalTestData data = getTestData();

  const TrifocalTensor2 tensor = TrifocalTensor2::FromBearingMeasurements(
      data.measurements[0], data.measurements[1], data.measurements[2]);

  for (size_t i = 0; i < data.measurements[0].size(); ++i) {
    const Rot2 actual_measurement =
        tensor.transform(data.measurements[1][i], data.measurements[2][i]);

    // Bearings are homogeneous: their coordinate ratios, not signs, matter.
    EXPECT(assert_equal(actual_measurement.c() * data.measurements[0][i].s(),
                        actual_measurement.s() * data.measurements[0][i].c(),
                        1e-8));
  }
}

// Regression: checks the tensor against an independent NumPy implementation.
TEST(TrifocalTensor2, tensorRegression) {
  const TrifocalTestData data = getTestData();

  const TrifocalTensor2 actual = TrifocalTensor2::FromBearingMeasurements(
      data.measurements[0], data.measurements[1], data.measurements[2]);

  Matrix2 expected0, expected1;
  expected0 << -0.16301732, -0.1968196, -0.6082839, -0.10324949;
  expected1 << 0.45758469, -0.36310941, 0.30334159, -0.34751881;
  const TrifocalTensor2 expected(expected0, expected1);

  EXPECT(assert_equal(expected, actual, 1e-2));
}

// Checks the projective transfer overload and its homogeneous scale.
TEST(TrifocalTensor2, projectiveTransform) {
  const TrifocalTestData data = getTestData();
  const TrifocalTensor2 tensor = TrifocalTensor2::FromBearingMeasurements(
      data.measurements[0], data.measurements[1], data.measurements[2]);

  const Rot2& bearingV = data.measurements[1][0];
  const Rot2& bearingW = data.measurements[2][0];
  const Point2 actual = tensor.transform(Point2(bearingV.c(), bearingV.s()),
                                         Point2(bearingW.c(), bearingW.s()));
  const Rot2& expected = data.measurements[0][0];
  EXPECT_DOUBLES_EQUAL(
      0.0, actual.x() * expected.s() - actual.y() * expected.c(), 1e-8);
}

// Checks input validation for insufficient and mismatched correspondences.
TEST(TrifocalTensor2, invalidMeasurements) {
  const TrifocalTestData data = getTestData();
  std::vector<Rot2> tooFew(data.measurements[0].begin(),
                           data.measurements[0].end() - 1);
  CHECK_EXCEPTION(
      TrifocalTensor2::FromBearingMeasurements(tooFew, tooFew, tooFew),
      std::invalid_argument);

  std::vector<Rot2> mismatched = data.measurements[2];
  mismatched.pop_back();
  CHECK_EXCEPTION(TrifocalTensor2::FromBearingMeasurements(
                      data.measurements[0], data.measurements[1], mismatched),
                  std::invalid_argument);
}

}  // namespace trifocal_tensor2_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
