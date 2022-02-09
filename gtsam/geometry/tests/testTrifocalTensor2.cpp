#include <iostream>
#include <vector>

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/TrifocalTensor2.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

TEST(TrifocalTensor2, transform) {
  // 2D robots poses
  Pose2 u = Pose2(0, 0, 0);
  Pose2 v = Pose2(1.0, 2.0, 3.1415926 / 6);
  Pose2 w = Pose2(-0.5, 0.5, -3.1415926 / 18);

  // 2D landmarks
  vector<Point2> landmarks;
  landmarks.push_back(Point2(2.0, 0.5));
  landmarks.push_back(Point2(-0.8, 2.4));
  landmarks.push_back(Point2(1.9, -0.4));
  landmarks.push_back(Point2(2.3, 1.0));
  landmarks.push_back(Point2(-0.4, -0.4));
  landmarks.push_back(Point2(-3.2, -1.0));
  landmarks.push_back(Point2(1.5, 2.0));

  // getting bearing measurement from landmarks
  vector<Rot2> measurement_u, measurement_v, measurement_w;
  for (int i = 0; i < landmarks.size(); ++i) {
    measurement_u.push_back(u.bearing(landmarks[i]));
    measurement_v.push_back(v.bearing(landmarks[i]));
    measurement_w.push_back(w.bearing(landmarks[i]));
  }

  // calculate trifocal tensor
  TrifocalTensor2 T(measurement_u, measurement_v, measurement_w);

  // estimate measurement of a robot from the measurements of the other two
  // robots
  for (unsigned int i = 0; i < measurement_u.size(); i++) {
    const Rot2 actual_measurement_u =
        T.transform(measurement_v[i], measurement_w[i]);

    // there might be two solutions for u1 and u2, comparing the ratio instead
    // of both cos and sin
    EXPECT(assert_equal(actual_measurement_u.c() * measurement_u[i].s(),
                        actual_measurement_u.s() * measurement_u[i].c(), 1e-8));
  }
}

TEST(TrifocalTensor2, mat0) {
  Pose2 u = Pose2(0, 0, 0);
  Pose2 v = Pose2(1.0, 2.0, 3.1415926 / 6);
  Pose2 w = Pose2(-0.5, 0.5, -3.1415926 / 18);

  // 2D landmarks
  vector<Point2> landmarks;
  landmarks.push_back(Point2(2.0, 0.5));
  landmarks.push_back(Point2(-0.8, 2.4));
  landmarks.push_back(Point2(1.9, -0.4));
  landmarks.push_back(Point2(2.3, 1.0));
  landmarks.push_back(Point2(-0.4, -0.4));
  landmarks.push_back(Point2(-3.2, -1.0));
  landmarks.push_back(Point2(1.5, 2.0));

  // getting bearing measurement from landmarks
  vector<Rot2> measurement_u, measurement_v, measurement_w;
  for (unsigned int i = 0; i < landmarks.size(); ++i) {
    measurement_u.push_back(u.bearing(landmarks[i]));
    measurement_v.push_back(v.bearing(landmarks[i]));
    measurement_w.push_back(w.bearing(landmarks[i]));
  }

  // calculate trifocal tensor
  TrifocalTensor2 T(measurement_u, measurement_v, measurement_w);

  Matrix2 actual_trifocal_tensor_mat0 = T.mat0();

  Matrix2 exp_trifocal_tensor_mat0;
  exp_trifocal_tensor_mat0 << -0.13178263, 0.29210566, -0.00860471, -0.73975238;

  EXPECT(assert_equal(actual_trifocal_tensor_mat0, exp_trifocal_tensor_mat0,
                      1e-2));
}

TEST(TrifocalTensor2, mat1) {
  Pose2 u = Pose2(0, 0, 0);
  Pose2 v = Pose2(1.0, 2.0, 3.1415926 / 6);
  Pose2 w = Pose2(-0.5, 0.5, -3.1415926 / 18);

  // 2D landmarks
  vector<Point2> landmarks;
  landmarks.push_back(Point2(2.0, 0.5));
  landmarks.push_back(Point2(-0.8, 2.4));
  landmarks.push_back(Point2(1.9, -0.4));
  landmarks.push_back(Point2(2.3, 1.0));
  landmarks.push_back(Point2(-0.4, -0.4));
  landmarks.push_back(Point2(-3.2, -1.0));
  landmarks.push_back(Point2(1.5, 2.0));

  // getting bearing measurement from landmarks
  vector<Rot2> measurement_u, measurement_v, measurement_w;
  for (unsigned int i = 0; i < landmarks.size(); ++i) {
    measurement_u.push_back(u.bearing(landmarks[i]));
    measurement_v.push_back(v.bearing(landmarks[i]));
    measurement_w.push_back(w.bearing(landmarks[i]));
  }

  // calculate trifocal tensor
  TrifocalTensor2 T(measurement_u, measurement_v, measurement_w);

  Matrix2 actual_trifocal_tensor_mat0 = T.mat1();

  Matrix2 exp_trifocal_tensor_mat0;
  exp_trifocal_tensor_mat0 << -0.27261704, 0.09097327, 0.51699647, 0.0108839;

  EXPECT(assert_equal(actual_trifocal_tensor_mat0, exp_trifocal_tensor_mat0,
                      1e-2));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
