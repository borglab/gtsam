/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testTranslationRecovery.cpp
 * @author Frank Dellaert, Akshay Krishnan
 * @date March 2020
 * @brief test recovering translations when rotations are given.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/sfm/TranslationRecovery.h>
#include <gtsam/slam/dataset.h>

using namespace std;
using namespace gtsam;

// Returns the Unit3 direction as measured in the binary measurement, but
// computed from the input poses. Helper function used in the unit tests.
Unit3 GetDirectionFromPoses(const Values& poses,
                            const BinaryMeasurement<Unit3>& unitTranslation) {
  const Pose3 wTa = poses.at<Pose3>(unitTranslation.key1()),
              wTb = poses.at<Pose3>(unitTranslation.key2());
  const Point3 Ta = wTa.translation(), Tb = wTb.translation();
  return Unit3(Tb - Ta);
}

/* ************************************************************************* */
// We read the BAL file, which has 3 cameras in it, with poses. We then assume
// the rotations are correct, but translations have to be estimated from
// translation directions only. Since we have 3 cameras, A, B, and C, we can at
// most create three relative measurements, let's call them w_aZb, w_aZc, and
// bZc. These will be of type Unit3. We then call `recoverTranslations` which
// sets up an optimization problem for the three unknown translations.
TEST(TranslationRecovery, BAL) {
  const string filename = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData db = SfmData::FromBalFile(filename);

  // Get camera poses, as Values
  size_t j = 0;
  Values poses;
  for (auto camera : db.cameras) {
    poses.insert(j++, camera.pose());
  }

  // Simulate measurements
  const auto relativeTranslations = TranslationRecovery::SimulateMeasurements(
      poses, {{0, 1}, {0, 2}, {1, 2}});

  // Check simulated measurements.
  for (auto& unitTranslation : relativeTranslations) {
    EXPECT(assert_equal(GetDirectionFromPoses(poses, unitTranslation),
                        unitTranslation.measured()));
  }

  TranslationRecovery algorithm;
  const auto graph = algorithm.buildGraph(relativeTranslations);
  EXPECT_LONGS_EQUAL(3, graph.size());

  // Run translation recovery
  const double scale = 2.0;
  const auto result = algorithm.run(relativeTranslations, scale);

  // Check result for first two translations, determined by prior
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0)));
  EXPECT(assert_equal(
      Point3(2 * GetDirectionFromPoses(poses, relativeTranslations[0])),
      result.at<Point3>(1)));

  // Check that the third translations is correct
  Point3 Ta = poses.at<Pose3>(0).translation();
  Point3 Tb = poses.at<Pose3>(1).translation();
  Point3 Tc = poses.at<Pose3>(2).translation();
  Point3 expected = (Tc - Ta) * (scale / (Tb - Ta).norm());
  EXPECT(assert_equal(expected, result.at<Point3>(2), 1e-4));

  // TODO(frank): how to get stats back?
  // EXPECT_DOUBLES_EQUAL(0.0199833, actualError, 1e-5);
}

TEST(TranslationRecovery, TwoPoseTest) {
  // Create a dataset with 2 poses.
  // __      __
  // \/      \/
  //  0 _____ 1
  //
  // 0 and 1 face in the same direction but have a translation offset.
  Values poses;
  poses.insert<Pose3>(0, Pose3(Rot3(), Point3(0, 0, 0)));
  poses.insert<Pose3>(1, Pose3(Rot3(), Point3(2, 0, 0)));

  auto relativeTranslations =
      TranslationRecovery::SimulateMeasurements(poses, {{0, 1}});

  // Check simulated measurements.
  for (auto& unitTranslation : relativeTranslations) {
    EXPECT(assert_equal(GetDirectionFromPoses(poses, unitTranslation),
                        unitTranslation.measured()));
  }

  TranslationRecovery algorithm;
  const auto graph = algorithm.buildGraph(relativeTranslations);
  EXPECT_LONGS_EQUAL(1, graph.size());

  // Run translation recovery
  const auto result = algorithm.run(relativeTranslations, /*scale=*/3.0);

  // Check result for first two translations, determined by prior
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0), 1e-8));
  EXPECT(assert_equal(Point3(3, 0, 0), result.at<Point3>(1), 1e-8));
}

TEST(TranslationRecovery, ThreePoseTest) {
  // Create a dataset with 3 poses.
  // __      __
  // \/      \/
  //  0 _____ 1
  //    \ __ /
  //     \\//
  //       3
  //
  // 0 and 1 face in the same direction but have a translation offset. 3 is in
  // the same direction as 0 and 1, in between 0 and 1, with some Y axis offset.

  Values poses;
  poses.insert<Pose3>(0, Pose3(Rot3(), Point3(0, 0, 0)));
  poses.insert<Pose3>(1, Pose3(Rot3(), Point3(2, 0, 0)));
  poses.insert<Pose3>(3, Pose3(Rot3(), Point3(1, -1, 0)));

  auto relativeTranslations = TranslationRecovery::SimulateMeasurements(
      poses, {{0, 1}, {1, 3}, {3, 0}});

  // Check simulated measurements.
  for (auto& unitTranslation : relativeTranslations) {
    EXPECT(assert_equal(GetDirectionFromPoses(poses, unitTranslation),
                        unitTranslation.measured()));
  }

  TranslationRecovery algorithm;
  const auto graph = algorithm.buildGraph(relativeTranslations);
  EXPECT_LONGS_EQUAL(3, graph.size());

  const auto result = algorithm.run(relativeTranslations, /*scale=*/3.0);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0), 1e-8));
  EXPECT(assert_equal(Point3(3, 0, 0), result.at<Point3>(1), 1e-8));
  EXPECT(assert_equal(Point3(1.5, -1.5, 0), result.at<Point3>(3), 1e-8));
}

TEST(TranslationRecovery, ThreePosesIncludingZeroTranslation) {
  // Create a dataset with 3 poses.
  // __      __
  // \/      \/
  //  0 _____ 1
  //          2 <|
  //
  // 0 and 1 face in the same direction but have a translation offset. 2 is at
  // the same point as 1 but is rotated, with little FOV overlap.
  Values poses;
  poses.insert<Pose3>(0, Pose3(Rot3(), Point3(0, 0, 0)));
  poses.insert<Pose3>(1, Pose3(Rot3(), Point3(2, 0, 0)));
  poses.insert<Pose3>(2, Pose3(Rot3::RzRyRx(-M_PI / 2, 0, 0), Point3(2, 0, 0)));

  auto relativeTranslations =
      TranslationRecovery::SimulateMeasurements(poses, {{0, 1}, {1, 2}});

  // Check simulated measurements.
  for (auto& unitTranslation : relativeTranslations) {
    EXPECT(assert_equal(GetDirectionFromPoses(poses, unitTranslation),
                        unitTranslation.measured()));
  }

  TranslationRecovery algorithm;
  // Run translation recovery
  const auto result = algorithm.run(relativeTranslations, /*scale=*/3.0);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0), 1e-8));
  EXPECT(assert_equal(Point3(3, 0, 0), result.at<Point3>(1), 1e-8));
  EXPECT(assert_equal(Point3(3, 0, 0), result.at<Point3>(2), 1e-8));
}

TEST(TranslationRecovery, FourPosesIncludingZeroTranslation) {
  // Create a dataset with 4 poses.
  // __      __
  // \/      \/
  //  0 _____ 1
  //    \ __  2 <|
  //     \\//
  //       3
  //
  // 0 and 1 face in the same direction but have a translation offset. 2 is at
  // the same point as 1 but is rotated, with very little FOV overlap. 3 is in
  // the same direction as 0 and 1, in between 0 and 1, with some Y axis offset.

  Values poses;
  poses.insert<Pose3>(0, Pose3(Rot3(), Point3(0, 0, 0)));
  poses.insert<Pose3>(1, Pose3(Rot3(), Point3(2, 0, 0)));
  poses.insert<Pose3>(2, Pose3(Rot3::RzRyRx(-M_PI / 2, 0, 0), Point3(2, 0, 0)));
  poses.insert<Pose3>(3, Pose3(Rot3(), Point3(1, -1, 0)));

  auto relativeTranslations = TranslationRecovery::SimulateMeasurements(
      poses, {{0, 1}, {1, 2}, {1, 3}, {3, 0}});

  // Check simulated measurements.
  for (auto& unitTranslation : relativeTranslations) {
    EXPECT(assert_equal(GetDirectionFromPoses(poses, unitTranslation),
                        unitTranslation.measured()));
  }

  TranslationRecovery algorithm;

  // Run translation recovery
  const auto result = algorithm.run(relativeTranslations, /*scale=*/4.0);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0), 1e-8));
  EXPECT(assert_equal(Point3(4, 0, 0), result.at<Point3>(1), 1e-8));
  EXPECT(assert_equal(Point3(4, 0, 0), result.at<Point3>(2), 1e-8));
  EXPECT(assert_equal(Point3(2, -2, 0), result.at<Point3>(3), 1e-8));
}

TEST(TranslationRecovery, ThreePosesWithZeroTranslation) {
  Values poses;
  poses.insert<Pose3>(0, Pose3(Rot3::RzRyRx(-M_PI / 6, 0, 0), Point3(0, 0, 0)));
  poses.insert<Pose3>(1, Pose3(Rot3(), Point3(0, 0, 0)));
  poses.insert<Pose3>(2, Pose3(Rot3::RzRyRx(M_PI / 6, 0, 0), Point3(0, 0, 0)));

  auto relativeTranslations = TranslationRecovery::SimulateMeasurements(
      poses, {{0, 1}, {1, 2}, {2, 0}});

  // Check simulated measurements.
  for (auto& unitTranslation : relativeTranslations) {
    EXPECT(assert_equal(GetDirectionFromPoses(poses, unitTranslation),
                        unitTranslation.measured()));
  }

  TranslationRecovery algorithm;

  // Run translation recovery
  const auto result = algorithm.run(relativeTranslations, /*scale=*/4.0);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0), 1e-8));
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(1), 1e-8));
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(2), 1e-8));
}

TEST(TranslationRecovery, ThreePosesWithOneSoftConstraint) {
  // Create a dataset with 3 poses.
  // __      __
  // \/      \/
  //  0 _____ 1
  //    \ __ /
  //     \\//
  //       3
  //
  // 0 and 1 face in the same direction but have a translation offset. 3 is in
  // the same direction as 0 and 1, in between 0 and 1, with some Y axis offset.

  Values poses;
  poses.insert<Pose3>(0, Pose3(Rot3(), Point3(0, 0, 0)));
  poses.insert<Pose3>(1, Pose3(Rot3(), Point3(2, 0, 0)));
  poses.insert<Pose3>(3, Pose3(Rot3(), Point3(1, -1, 0)));

  auto relativeTranslations = TranslationRecovery::SimulateMeasurements(
      poses, {{0, 1}, {0, 3}, {1, 3}});

  std::vector<BinaryMeasurement<Point3>> betweenTranslations;
  betweenTranslations.emplace_back(0, 3, Point3(1, -1, 0),
                                   noiseModel::Isotropic::Sigma(3, 1e-2));

  TranslationRecovery algorithm;
  auto result =
      algorithm.run(relativeTranslations, /*scale=*/0.0, betweenTranslations);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0), 1e-4));
  EXPECT(assert_equal(Point3(2, 0, 0), result.at<Point3>(1), 1e-4));
  EXPECT(assert_equal(Point3(1, -1, 0), result.at<Point3>(3), 1e-4));
}

TEST(TranslationRecovery, ThreePosesWithOneHardConstraint) {
  // Create a dataset with 3 poses.
  // __      __
  // \/      \/
  //  0 _____ 1
  //    \ __ /
  //     \\//
  //       3
  //
  // 0 and 1 face in the same direction but have a translation offset. 3 is in
  // the same direction as 0 and 1, in between 0 and 1, with some Y axis offset.

  Values poses;
  poses.insert<Pose3>(0, Pose3(Rot3(), Point3(0, 0, 0)));
  poses.insert<Pose3>(1, Pose3(Rot3(), Point3(2, 0, 0)));
  poses.insert<Pose3>(3, Pose3(Rot3(), Point3(1, -1, 0)));

  auto relativeTranslations = TranslationRecovery::SimulateMeasurements(
      poses, {{0, 1}, {0, 3}, {1, 3}});

  std::vector<BinaryMeasurement<Point3>> betweenTranslations;
  betweenTranslations.emplace_back(0, 1, Point3(2, 0, 0),
                                   noiseModel::Constrained::All(3, 1e2));

  TranslationRecovery algorithm;
  auto result =
      algorithm.run(relativeTranslations, /*scale=*/0.0, betweenTranslations);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0), 1e-4));
  EXPECT(assert_equal(Point3(2, 0, 0), result.at<Point3>(1), 1e-4));
  EXPECT(assert_equal(Point3(1, -1, 0), result.at<Point3>(3), 1e-4));
}

TEST(TranslationRecovery, NodeWithBetweenFactorAndNoMeasurements) {
  // Checks that valid results are obtained when a between translation edge is
  // provided with a node that does not have any other relative translations.
  Values poses;
  poses.insert<Pose3>(0, Pose3(Rot3(), Point3(0, 0, 0)));
  poses.insert<Pose3>(1, Pose3(Rot3(), Point3(2, 0, 0)));
  poses.insert<Pose3>(3, Pose3(Rot3(), Point3(1, -1, 0)));
  poses.insert<Pose3>(4, Pose3(Rot3(), Point3(1, 2, 1)));

  auto relativeTranslations = TranslationRecovery::SimulateMeasurements(
      poses, {{0, 1}, {0, 3}, {1, 3}});

  std::vector<BinaryMeasurement<Point3>> betweenTranslations;
  betweenTranslations.emplace_back(0, 1, Point3(2, 0, 0),
                                   noiseModel::Constrained::All(3, 1e2));
  // Node 4 only has this between translation prior, no relative translations.
  betweenTranslations.emplace_back(0, 4, Point3(1, 2, 1));

  TranslationRecovery algorithm;
  auto result =
      algorithm.run(relativeTranslations, /*scale=*/0.0, betweenTranslations);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0), 1e-4));
  EXPECT(assert_equal(Point3(2, 0, 0), result.at<Point3>(1), 1e-4));
  EXPECT(assert_equal(Point3(1, -1, 0), result.at<Point3>(3), 1e-4));
  EXPECT(assert_equal(Point3(1, 2, 1), result.at<Point3>(4), 1e-4));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
