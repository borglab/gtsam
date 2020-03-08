/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testTranslationRecovery.cpp
 * @author Frank Dellaert
 * @date March 2020
 * @brief test recovering translations when rotations are given.
 */

#include <gtsam/slam/dataset.h>
// #include <gtsam/sam/TranslationFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

class TranslationFactor {};

// Set up an optimization problem for the unknown translations Ti in the world
// coordinate frame, given the known camera attitudes wRi with respect to the
// world frame, and a set of (noisy) translation directions of type Unit3,
// aZb. The measurement equation is
//    aZb = Unit3(aRw * (Tb - Ta))   (1)
// i.e., aZb is the normalized translation of B's origin in the camera frame A.
// It is clear that we cannot recover the scale, nor the absolute position, so
// the gauge freedom in this case is 3 + 1 = 4. We fix these by taking fixing
// the translations Ta and Tb associated with the first measurement aZb,
// clamping them to their initial values as given to this method. If no initial
// values are given, we use the origin for Tb and set Tb to make (1) come
// through, i.e.,
//    Tb = s * wRa * Point3(aZb)     (2)
// where s is an arbitrary scale that can be supplied, default 1.0. Hence, two
// versions are supplied below corresponding to whether we have initial values
// or not. Because the latter one is called from the first one, this is prime.

void recoverTranslations(const double scale = 1.0) {
  //   graph.emplace_shared<TranslationFactor>(m.second, unit2, m.first,
  //   P(j));
}

using KeyPair = pair<Key, Key>;

/**
 * @brief Simulate translation direction measurements
 *
 * @param poses SE(3) ground truth poses stored as Values
 * @param edges pairs (a,b) for which a measurement aZb will be generated.
 */
vector<Unit3> simulateMeasurements(const Values& poses,
                                   const vector<KeyPair>& edges) {
  vector<Unit3> measurements;
  for (auto edge : edges) {
    Key a, b;
    std::tie(a, b) = edge;
    Pose3 wTa = poses.at<Pose3>(a), wTb = poses.at<Pose3>(b);
    Point3 Ta = wTa.translation(), Tb = wTb.translation();
    auto aRw = wTa.rotation().inverse();
    Unit3 aZb = Unit3(aRw * (Tb - Ta));
    measurements.push_back(aZb);
  }
  return measurements;
}

/* ************************************************************************* */
// We read the BAL file, which has 3 cameras in it, with poses. We then assume
// the rotations are correct, but translations have to be estimated from
// translation directions only. Since we have 3 cameras, A, B, and C, we can at
// most create three relative measurements, let's call them aZb, aZc, and bZc.
// These will be of type Unit3. We then call `recoverTranslations` which sets up
// an optimization problem for the three unknown translations.
TEST(TranslationRecovery, BAL) {
  string filename = findExampleDataFile("dubrovnik-3-7-pre");
  SfM_data db;
  bool success = readBAL(filename, db);
  if (!success) throw runtime_error("Could not access file!");

  SharedNoiseModel unit2 = noiseModel::Unit::Create(2);
  NonlinearFactorGraph graph;

  size_t i = 0;
  Values poses;
  for (auto camera : db.cameras) {
    GTSAM_PRINT(camera);
    poses.insert(i++, camera.pose());
  }
  Pose3 wTa = poses.at<Pose3>(0), wTb = poses.at<Pose3>(1),
        wTc = poses.at<Pose3>(2);
  Point3 Ta = wTa.translation(), Tb = wTb.translation(), Tc = wTc.translation();
  auto measurements = simulateMeasurements(poses, {{0, 1}, {0, 2}, {1, 2}});
  auto aRw = wTa.rotation().inverse();
  Unit3 aZb = measurements[0];
  EXPECT(assert_equal(Unit3(aRw * (Tb - Ta)), aZb));
  Unit3 aZc = measurements[1];
  EXPECT(assert_equal(Unit3(aRw * (Tc - Ta)), aZc));
  //   Values initial = initialCamerasAndPointsEstimate(db);

  //   LevenbergMarquardtOptimizer lm(graph, initial);

  //   Values actual = lm.optimize();
  //   double actualError = graph.error(actual);
  //   EXPECT_DOUBLES_EQUAL(0.0199833, actualError, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
