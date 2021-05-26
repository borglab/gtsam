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

#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
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

  // Check simulated measurements.
  for (auto& unitTranslation : relativeTranslations) {
    EXPECT(assert_equal(GetDirectionFromPoses(poses, unitTranslation),
                        unitTranslation.measured()));
  }

  TranslationRecovery algorithm(relativeTranslations);
  const auto graph = algorithm.buildGraph();
  EXPECT_LONGS_EQUAL(3, graph.size());

  // Run translation recovery
  const double scale = 2.0;
  const auto result = algorithm.run(scale);

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

  TranslationRecovery algorithm(relativeTranslations);
  const auto graph = algorithm.buildGraph();
  EXPECT_LONGS_EQUAL(1, graph.size());

  // Run translation recovery
  const auto result = algorithm.run(/*scale=*/3.0);

  // Check result for first two translations, determined by prior
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0)));
  EXPECT(assert_equal(Point3(3, 0, 0), result.at<Point3>(1)));
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

  TranslationRecovery algorithm(relativeTranslations);
  const auto graph = algorithm.buildGraph();
  EXPECT_LONGS_EQUAL(3, graph.size());

  const auto result = algorithm.run(/*scale=*/3.0);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0)));
  EXPECT(assert_equal(Point3(3, 0, 0), result.at<Point3>(1)));
  EXPECT(assert_equal(Point3(1.5, -1.5, 0), result.at<Point3>(3)));
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

  TranslationRecovery algorithm(relativeTranslations);
  const auto graph = algorithm.buildGraph();
  // There is only 1 non-zero translation edge.
  EXPECT_LONGS_EQUAL(1, graph.size());

  // Run translation recovery
  const auto result = algorithm.run(/*scale=*/3.0);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0)));
  EXPECT(assert_equal(Point3(3, 0, 0), result.at<Point3>(1)));
  EXPECT(assert_equal(Point3(3, 0, 0), result.at<Point3>(2)));
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

  TranslationRecovery algorithm(relativeTranslations);
  const auto graph = algorithm.buildGraph();
  EXPECT_LONGS_EQUAL(3, graph.size());

  // Run translation recovery
  const auto result = algorithm.run(/*scale=*/4.0);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0)));
  EXPECT(assert_equal(Point3(4, 0, 0), result.at<Point3>(1)));
  EXPECT(assert_equal(Point3(4, 0, 0), result.at<Point3>(2)));
  EXPECT(assert_equal(Point3(2, -2, 0), result.at<Point3>(3)));
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

  TranslationRecovery algorithm(relativeTranslations);
  const auto graph = algorithm.buildGraph();
  // Graph size will be zero as there no 'non-zero distance' edges.
  EXPECT_LONGS_EQUAL(0, graph.size());

  // Run translation recovery
  const auto result = algorithm.run(/*scale=*/4.0);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0)));
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(1)));
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(2)));
}


TEST(TranslationRecovery, FourPosesSquareWithTwoEdges) {
  Values poses;
  float radius = 40;
  float height = 10.0;
  for(int i = 0; i < 4; i++) {
   poses.insert<Pose3>(i, Pose3(Rot3::RzRyRx(0, 0, 0), 
			  Point3(radius*cos(i * M_PI/2), radius*sin(i * M_PI/2), height)));
  }
  auto relativeTranslations = TranslationRecovery::SimulateMeasurements(
		  poses, {{1, 0}, {2, 1}, {3, 2}, {0, 3}});

  for (auto& unitTranslation : relativeTranslations) {
    EXPECT(assert_equal(GetDirectionFromPoses(poses, unitTranslation), 
			    unitTranslation.measured()));
  }

  LevenbergMarquardtParams params;
  params.setVerbosityLM("summary");
  TranslationRecovery algorithm(relativeTranslations, params);
  const auto graph = algorithm.buildGraph();
  const auto result = algorithm.run(2.8284);

  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0)));
  EXPECT(assert_equal(Point3(-2, 2, 0), result.at<Point3>(1)));
  std::cout.precision(2);
  Ordering ordering = Ordering::Colamd(graph);
  try {
    Marginals marginals(graph, result, ordering);
  } catch (const IndeterminantLinearSystemException& e) {
    std::cout << "ILS Error caught at " << e.nearbyVariable() << std::endl;
    for(const Key k : ordering) {
      if(k == e.nearbyVariable()) break;
      std::cout << k << " has a good solution" << std::endl;
    }
  }


/*  std::cout << "x0 covariance:\n" << marginals.marginalInformation(0) << std::endl;  
  std::cout << "x1 covariance:\n" << marginals.marginalInformation(1) << std::endl;
/*  std::cout << "x2 covariance:\n" << marginals.marginalInformation(2) << std::endl;
  std::cout << "x3 covariance:\n" << marginals.marginalInformation(3) << std::endl;  
*/
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
