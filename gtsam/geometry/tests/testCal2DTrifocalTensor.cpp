#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Pose2.h>

#include <vector>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

typedef BearingRange<Pose2, Point2> BearingRange2D;

GTSAM_CONCEPT_TESTABLE_INST(TrifocalTensor2D)
GTSAM_CONCEPT_MANIFOLD_INST(TrifocalTensor2D)

TEST(TrifocalTensor2D, get_estimate_measurement) {
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
  TrifocalTensor2D T(measurement_u, measurement_v, measurement_w);

  // estimate measurement of a robot from the measurements of the other two
  // robots
  vector<Rot2> exp_measurement_u;
  exp_measurement_u = T.get_estimate_measurement(measurement_v, measurement_w);
  EXPECT(assert_equal(exp_measurement_u, measurement_u, 1e-8));
}