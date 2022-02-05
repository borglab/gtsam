#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/TrifocalTensor2.h>

#include <vector>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

typedef BearingRange<Pose2, Point2> BearingRange2D;

TEST(TrifocalTensor2, transform) {
  // 2D robots poses
  Pose2 u = Pose2(0, 0, 0);
  Pose2 v = Pose2(1.0, 2.0, 3.1415926 / 6);
  Pose2 w = Pose2(-0.5, 0.5, -3.1415926 / 10);

  // 2D landmarks
  vector<Point2> landmarks;
  landmarks.push_back(Point2(2.0, 0.5));
  landmarks.push_back(Point2(-0.8, 2.4));
  landmarks.push_back(Point2(1.9, -0.4));
  landmarks.push_back(Point2(2.3, 1.0));
  landmarks.push_back(Point2(-0.4, -0.4));
  landmarks.push_back(Point2(-3.2, -1.0));
  landmarks.push_back(Point2(1.5, 2.0));

  /*
   * the example
   * D:\Gatech\research\3DReconstruction\GTSAM\gtsam\examples\Pose3SLAMExampleExpressions_BearingRangeWithTransform.cpp
   * uses class graph. I don't have enough time to read code about that now.
   * Maybe I'll do it later.
   */

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
  for (int i = 0; i < measurement_u.size(); i++) {
    const Rot2 actual_measurement_u = T.transform(measurement_v[i], measurement_w[i]);
    EXPECT(assert_equal(actual_measurement_u, measurement_u[i], 1e-8));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
