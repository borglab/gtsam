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
  data.gt_poses.emplace_back(0, 0, 0);
  data.gt_poses.emplace_back(0, 0, 0);

  // Landmarks
  data.gt_landmarks.emplace_back(2.0, 0.5);
  data.gt_landmarks.emplace_back(-0.8, 2.4);
  data.gt_landmarks.emplace_back(1.9, -0.4);
  data.gt_landmarks.emplace_back(2.3, 1.0);
  data.gt_landmarks.emplace_back(-0.4, -0.4);
  data.gt_landmarks.emplace_back(-3.2, -1.0);
  data.gt_landmarks.emplace_back(1.5, 2.0);

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

TEST(TrifocalTensor2, transform) {
  trifocal::TrifocalTestData data = trifocal::getTestData();

  // calculate trifocal tensor
  TrifocalTensor2 T(data.measurements[0], data.measurements[1],
                    data.measurements[2]);

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

TEST(TrifocalTensor2, tensorRegression) {
  trifocal::TrifocalTestData data = trifocal::getTestData();

  // calculate trifocal tensor
  TrifocalTensor2 T(data.measurements[0], data.measurements[1],
                    data.measurements[2]);

  Matrix2 expected_tensor_mat0, expected_tensor_mat1;
  // These values were obtained from a numpy-based python implementation.
  expected_tensor_mat0 << -0.13178263, 0.29210566, -0.00860471, -0.73975238;
  expected_tensor_mat1 << -0.27261704, 0.09097327, 0.51699647, 0.0108839;

  EXPECT(assert_equal(T.mat0(), expected_tensor_mat0, 1e-2));
  EXPECT(assert_equal(T.mat1(), expected_tensor_mat1, 1e-2));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
