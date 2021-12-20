/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
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

#include <gtsam/sfm/TranslationRecovery.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/slam/dataset.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// We read the BAL file, which has 3 cameras in it, with poses. We then assume
// the rotations are correct, but translations have to be estimated from
// translation directions only. Since we have 3 cameras, A, B, and C, we can at
// most create three relative measurements, let's call them w_aZb, w_aZc, and
// bZc. These will be of type Unit3. We then call `recoverTranslations` which
// sets up an optimization problem for the three unknown translations.
TEST(TranslationRecovery, BAL) {
  const string filename = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData db;
  bool success = readBAL(filename, db);
  if (!success) throw runtime_error("Could not access file!");

  // Get camera poses, as Values
  size_t j = 0;
  Values poses;
  for (auto camera : db.cameras) {
    poses.insert(j++, camera.pose());
  }

  // Simulate measurements
  const auto relativeTranslations = TranslationRecovery::SimulateMeasurements(
      poses, {{0, 1}, {0, 2}, {1, 2}});

  // Check
  const Pose3 wTa = poses.at<Pose3>(0), wTb = poses.at<Pose3>(1),
              wTc = poses.at<Pose3>(2);
  const Point3 Ta = wTa.translation(), Tb = wTb.translation(),
               Tc = wTc.translation();
  const Rot3 aRw = wTa.rotation().inverse();
  const Unit3 w_aZb = relativeTranslations.at({0, 1});
  EXPECT(assert_equal(Unit3(Tb - Ta), w_aZb));
  const Unit3 w_aZc = relativeTranslations.at({0, 2});
  EXPECT(assert_equal(Unit3(Tc - Ta), w_aZc));

  TranslationRecovery algorithm(relativeTranslations);
  const auto graph = algorithm.buildGraph();
  EXPECT_LONGS_EQUAL(3, graph.size());

  // Translation recovery, version 1
  const double scale = 2.0;
  const auto result = algorithm.run(scale);

  // Check result for first two translations, determined by prior
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0)));
  EXPECT(assert_equal(Point3(2 * w_aZb.point3()), result.at<Point3>(1)));

  // Check that the third translations is correct
  Point3 expected = (Tc - Ta) * (scale / (Tb - Ta).norm());
  EXPECT(assert_equal(expected, result.at<Point3>(2), 1e-4));

  // TODO(frank): how to get stats back?
  // EXPECT_DOUBLES_EQUAL(0.0199833, actualError, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
