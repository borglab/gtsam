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
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/TrifocalTensor2.h>

#include <vector>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

namespace trifocal {

struct TrifocalTestData {
  vector<Pose2> gt_poses;
  vector<Point2> gt_landmarks;

  // Outer vector over poses.
  std::vector<vector<Rot2>> measurements;
};

TrifocalTestData getTestData() {
  TrifocalTestData data;

  // Poses
  data.gt_poses.emplace_back(0, 0, 0);
  data.gt_poses.emplace_back(-1.9, 4, -2 * acos(0.0) / 8);
  data.gt_poses.emplace_back(2.1, -2.1, 2 * acos(0.0) / 3);

  // Landmarks
  data.gt_landmarks.emplace_back(1.2, 1.0);
  data.gt_landmarks.emplace_back(2.4, 3.5);
  data.gt_landmarks.emplace_back(-1.0, 0.5);
  data.gt_landmarks.emplace_back(3.4, -1.5);
  data.gt_landmarks.emplace_back(5.1, 0.6);
  data.gt_landmarks.emplace_back(-0.1, -0.7);
  data.gt_landmarks.emplace_back(3.1, 1.9);

  // Measurements
  for (const Pose2& pose : data.gt_poses) {
    std::vector<Rot2> measurements;
    for (const Point2& landmark : data.gt_landmarks) {
      measurements.push_back(pose.bearing(landmark));
    }
    data.measurements.push_back(measurements);
  }
  return data;
}

}  // namespace trifocal

// Check transform() correctly transforms measurements from 2 views to third.
TEST(TrifocalTensor2, transform) {
  trifocal::TrifocalTestData data = trifocal::getTestData();

  // calculate trifocal tensor
  TrifocalTensor2 T = TrifocalTensor2::FromBearingMeasurements(
      data.measurements[0], data.measurements[1], data.measurements[2]);

  // estimate measurement of a robot from the measurements of the other two
  // robots
  for (unsigned int i = 0; i < data.measurements[0].size(); i++) {
    const Rot2 actual_measurement =
        T.transform(data.measurements[1][i], data.measurements[2][i]);

    // there might be two solutions for u1 and u2, comparing the ratio instead
    // of both cos and sin
    EXPECT(assert_equal(actual_measurement.c() * data.measurements[0][i].s(),
                        actual_measurement.s() * data.measurements[0][i].c(),
                        1e-8));
  }
}

// Check the correct tensor is computed from measurements (catch regressions).
TEST(TrifocalTensor2, tensorRegression) {
  trifocal::TrifocalTestData data = trifocal::getTestData();

  // calculate trifocal tensor
  TrifocalTensor2 T = TrifocalTensor2::FromBearingMeasurements(
      data.measurements[0], data.measurements[1], data.measurements[2]);

  Matrix2 expected_tensor_mat0, expected_tensor_mat1;
  // These values were obtained from a numpy-based python implementation.
  expected_tensor_mat0 << -0.16301732 -0.1968196, -0.6082839  -0.10324949;
  expected_tensor_mat1 << 0.45758469 -0.36310941, 0.30334159 -0.34751881;

  EXPECT(assert_equal(T.mat0(), expected_tensor_mat0, 1e-2));
  EXPECT(assert_equal(T.mat1(), expected_tensor_mat1, 1e-2));
}

// Check the calculation of Jacobian (Ground-true Jacobian comes from Auto-Grad
// result of Pytorch)
TEST(TrifocalTensor2, Jacobian) {
  trifocal::TrifocalTestData data = trifocal::getTestData();

  // Construct trifocal tensor using 2 rotations and 3 bearing measurements in 3
  // cameras.
  std::vector<Rot2> trifocal_in_angle;
  trifocal_in_angle.insert(
      trifocal_in_angle.end(),
      {-0.39269908169872414, 1.0471975511965976, 2.014244663214635,
       -0.7853981633974483, -0.5976990577022983});

  // calculate trifocal tensor
  TrifocalTensor2 T(trifocal_in_angle);

  // Calculate Jacobian matrix
  Matrix jacobian_of_trifocal = T.Jacobian(
      data.measurements[0], data.measurements[1], data.measurements[2]);
  // These values were obtained from a Pytorch-based python implementation.
  Matrix expected_jacobian(7, 5) << -2.2003, 0.7050, 0.9689, 0.6296, -3.1280,
      -4.6886, 1.1274, 2.7912, 1.6121, -5.1817, -0.7223, -0.6841, 0.5387,
      0.7208, -0.5677, -0.8645, 0.1767, 0.5967, 0.9383, -2.2041, -3.0437,
      0.5239, 2.0144, 1.6368, -4.0335, -1.9855, -0.2741, 1.4741, 0.6783,
      -0.9262, -4.6600, 0.7275, 2.8182, 1.9639, -5.5489;

  EXPECT(assert_equal(jacobian_of_trifocal, expected_jacobian, 1e-8));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
