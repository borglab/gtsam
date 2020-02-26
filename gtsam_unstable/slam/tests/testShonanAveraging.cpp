/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testShonanAveraging.cpp
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Unit tests for Shonan Averaging algorithm
 */

#include <gtsam_unstable/slam/ShonanAveraging.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <map>

using namespace std;
using namespace gtsam;

string g2oFile = findExampleDataFile("toyExample.g2o");
static const ShonanAveraging kShonan(g2oFile);

/* ************************************************************************* */
TEST(ShonanAveraging, buildGraphAt)
{
  auto graph = kShonan.buildGraphAt(5);
  EXPECT_LONGS_EQUAL(5, kShonan.nrPoses());
  EXPECT_LONGS_EQUAL(6, graph.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging, checkOptimality)
{
  auto Q = kShonan.buildQ();
  EXPECT_LONGS_EQUAL(3 * 5, Q.rows());
  EXPECT_LONGS_EQUAL(3 * 5, Q.cols());
  const Values random = kShonan.initializeRandomlyAt(4);
  auto Lambda = kShonan.computeLambda(random);
  EXPECT_LONGS_EQUAL(3 * 5, Lambda.rows());
  EXPECT_LONGS_EQUAL(3 * 5, Lambda.cols());
  EXPECT_LONGS_EQUAL(45, Lambda.nonZeros());
  auto lambdaMin = kShonan.computeMinEigenValue(random);
  // EXPECT_DOUBLES_EQUAL(-5.2964625490657866, lambdaMin,
  //                      1e-4);  // Regression test
  EXPECT_DOUBLES_EQUAL(-4.3860073075695709, lambdaMin,
                       1e-4); // Regression test
  EXPECT(!kShonan.checkOptimality(random));
}

/* ************************************************************************* */
TEST(ShonanAveraging, tryOptimizingAt3)
{
  const Values initial = kShonan.initializeRandomlyAt(3);
  EXPECT(!kShonan.checkOptimality(initial));
  const Values result = kShonan.tryOptimizingAt(3, initial);
  EXPECT(kShonan.checkOptimality(result));
  auto lambdaMin = kShonan.computeMinEigenValue(result);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, lambdaMin,
                       1e-4); // Regression test
  EXPECT_DOUBLES_EQUAL(0, kShonan.costAt(3, result), 1e-4);
  const Values SO3Values = kShonan.roundSolution(result);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(SO3Values), 1e-4);
}

/* ************************************************************************* */
TEST(ShonanAveraging, tryOptimizingAt4)
{
  const Values result = kShonan.tryOptimizingAt(4);
  EXPECT(kShonan.checkOptimality(result));
  EXPECT_DOUBLES_EQUAL(0, kShonan.costAt(4, result), 1e-3);
  auto lambdaMin = kShonan.computeMinEigenValue(result);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, lambdaMin,
                       1e-4); // Regression test
  const Values SO3Values = kShonan.roundSolution(result);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(SO3Values), 1e-4);
}

/* ************************************************************************* */
// TEST(ShonanAveraging, tryOptimizingAt5) {
//   const Values result = kShonan.tryOptimizingAt(5);
//   EXPECT_DOUBLES_EQUAL(0, kShonan.costAt(5, result), 1e-3);
//   auto lambdaMin = kShonan.computeMinEigenValue(result);
//   EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, lambdaMin,
//                        1e-4);  // Regression test
// }

/* ************************************************************************* */
TEST(ShonanAveraging, runWithRandom)
{
  auto result = kShonan.runWithRandom(5);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(result.first), 1e-3);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, result.second,
                       1e-4); // Regression test
}

/* ************************************************************************* */
TEST(ShonanAveraging, MakeATangentVector)
{
  Vector9 v;
  v << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Matrix expected(5, 5);
  expected << 0, 0, 0, 0, -4, //
      0, 0, 0, 0, -5,         //
      0, 0, 0, 0, -6,         //
      0, 0, 0, 0, 0,          //
      4, 5, 6, 0, 0;
  const Vector xi_1 = ShonanAveraging::MakeATangentVector(5, v, 1);
  const auto actual = SOn::Hat(xi_1);
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(ShonanAveraging, dimensionLifting)
{
  const Values Qstar3 = kShonan.tryOptimizingAt(3);
  Vector minEigenVector;
  kShonan.computeMinEigenValue(Qstar3, &minEigenVector);
  Values initialQ4 = kShonan.dimensionLifting(4, Qstar3, minEigenVector);
  EXPECT_LONGS_EQUAL(5, initialQ4.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging, initializeWithDescent)
{
  const Values Qstar3 = kShonan.tryOptimizingAt(3);
  Vector minEigenVector;
  double lambdaMin = kShonan.computeMinEigenValue(Qstar3, &minEigenVector);
  Values initialQ4 =
      kShonan.initializeWithDescent(4, Qstar3, minEigenVector, lambdaMin);
  EXPECT_LONGS_EQUAL(5, initialQ4.size());
}

/* ************************************************************************* */
TEST(ShonanAveraging, runWithDescent)
{
  auto result = kShonan.runWithDescent(5);
  EXPECT_DOUBLES_EQUAL(0, kShonan.cost(result.first), 1e-3);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, result.second,
                       1e-4); // Regression test
}

/* ************************************************************************* */
TEST(ShonanAveraging, runWithRandomKlaus)
{
  string g2oFile = findExampleDataFile("Klaus3.g2o");
  static const ShonanAveraging shonan(g2oFile);

  // Check nr poses
  EXPECT_LONGS_EQUAL(3, shonan.nrPoses());

  // Check poses in g2o file
  const auto &poses = shonan.poses();
  Point3 r1(1, 0, 0), r2(0, 1, 0), r3(0, 0, 1);
  const Rot3 wRc(r1, r2, r3);
  // const Pose3 wTc(wRc, Point3(-3.9, 0, 0));
  // Check rotation from the datafile 
  Rot3 rot0 = poses.at(0).rotation();
  Rot3 rot1 = poses.at(1).rotation();
  Rot3 rot2 = poses.at(2).rotation();
  EXPECT(assert_equal(wRc, rot0, 0.2));
  EXPECT(assert_equal(wRc, rot1, 0.2));
  EXPECT(assert_equal(wRc, rot2, 0.2));

  // Check measurement data
  Rot3 R01(0.9995433591728293, -0.022048798853273946, -0.01796327847857683,
           0.010210006313668573);
  Rot3 R12(0.9927742290779572, -0.054972994022992064, 0.10432547598981769,
           -0.02221474884651081);
  Rot3 R02(0.9922479626852876, -0.03174661848656213, 0.11646825423134777,
           -0.02951742735854383);
  EXPECT(assert_equal(rot1, rot0.compose(R01), 0.1));
  EXPECT(assert_equal(rot2, rot1.compose(R12), 0.1));
  EXPECT(assert_equal(rot2, rot0.compose(R02), 0.1));

  auto result = shonan.runWithRandom(5);
  EXPECT_DOUBLES_EQUAL(0, shonan.cost(result.first), 1e-3);
  EXPECT_DOUBLES_EQUAL(-5.427688831332745e-07, result.second,
                       1e-4);  // Regression test

  // Normalize the rotations
  Rot3 expected_rot0 = rot0.compose(rot0.inverse());
  Rot3 expected_rot1 = rot1.compose(rot0.inverse());
  Rot3 expected_rot2 = rot2.compose(rot0.inverse());
  Rot3 result_rot0 = Rot3(result.first.at<SO3>(0));
  Rot3 result_rot1 = Rot3(result.first.at<SO3>(1));
  Rot3 result_rot2 = Rot3(result.first.at<SO3>(2));
  Rot3 actual_rot0 = result_rot0.compose(result_rot0.inverse());
  Rot3 actual_rot1 = result_rot1.compose(result_rot0.inverse());
  Rot3 actual_rot2 = result_rot2.compose(result_rot0.inverse());

  // Check shonan result
  EXPECT(assert_equal(Rot3(), expected_rot0))
  EXPECT(assert_equal(Rot3(), actual_rot0))
  EXPECT(assert_equal(expected_rot1, actual_rot1));
  EXPECT(assert_equal(expected_rot2, actual_rot2));
  EXPECT(assert_equal(Rot3::Logmap(expected_rot1), Rot3::Logmap(actual_rot1),
                      0.1));
  EXPECT(assert_equal(Rot3::Logmap(expected_rot2), Rot3::Logmap(actual_rot2),
                      0.1));
}

/* ************************************************************************* */
int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
