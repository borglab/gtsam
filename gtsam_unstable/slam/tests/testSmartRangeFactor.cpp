/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartRangeFactor.cpp
 *  @brief Unit tests for SmartRangeFactor Class
 *  @author Frank Dellaert
 *  @date Nov 2013
 */

#include <gtsam_unstable/slam/SmartRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

static const double sigma = 2.0;

// Test situation:
static const Point2 p(0, 10);
static const Pose2 pose1(0, 0, 0), pose2(5, 0, 0), pose3(5, 5, 0);
static const double r1 = pose1.range(p), r2 = pose2.range(p), r3 = pose3.range(
    p);

/* ************************************************************************* */

TEST( SmartRangeFactor, constructor ) {
  SmartRangeFactor f1;
  LONGS_EQUAL(0, f1.size())
  SmartRangeFactor f2(sigma);
  LONGS_EQUAL(0, f2.size())
}
/* ************************************************************************* */

TEST( SmartRangeFactor, addRange ) {
  SmartRangeFactor f(sigma);
  f.addRange(1, 10);
  f.addRange(2, 12);
  LONGS_EQUAL(2, f.size())
}
/* ************************************************************************* */

TEST( SmartRangeFactor, scenario ) {
  DOUBLES_EQUAL(10, r1, 1e-9);
  DOUBLES_EQUAL(sqrt(100.0+25.0), r2, 1e-9);
  DOUBLES_EQUAL(sqrt(50.0), r3, 1e-9);
}
/* ************************************************************************* */

TEST( SmartRangeFactor, unwhitenedError ) {
  Values values; // all correct
  values.insert(1, pose1);
  values.insert(2, pose2);
  values.insert(3, pose3);

  SmartRangeFactor f(sigma);
  f.addRange(1, r1);

  // Check Jacobian for n==1
  vector<Matrix> H1(1);
  f.unwhitenedError(values, H1); // with H now !
  CHECK(assert_equal(Matrix::Zero(3,1), H1.front()));

  // Whenever there are two ranges or less, error should be zero
  Vector actual1 = f.unwhitenedError(values);
  EXPECT(assert_equal((Vector(1) << 0.0).finished(), actual1));
  f.addRange(2, r2);
  Vector actual2 = f.unwhitenedError(values);
  EXPECT(assert_equal((Vector(1) << 0.0).finished(), actual2));

  f.addRange(3, r3);
  vector<Matrix> H(3);
  Vector actual3 = f.unwhitenedError(values);
  EXPECT_LONGS_EQUAL(3, f.keys().size());
  EXPECT(assert_equal((Vector(1) << 0.0).finished(), actual3));

  // Check keys and Jacobian
  Vector actual4 = f.unwhitenedError(values, H); // with H now !
  EXPECT(assert_equal((Vector(1) << 0.0).finished(), actual4));
  CHECK(assert_equal((Matrix(1, 3) << 0.0,-1.0,0.0).finished(), H.front()));
  CHECK(assert_equal((Matrix(1, 3) << sqrt(2.0)/2,-sqrt(2.0)/2,0.0).finished(), H.back()));

  // Test clone
  NonlinearFactor::shared_ptr clone = f.clone();
  EXPECT_LONGS_EQUAL(3, clone->keys().size());
}
/* ************************************************************************* */

TEST( SmartRangeFactor, optimization ) {
  SmartRangeFactor f(sigma);
  f.addRange(1, r1);
  f.addRange(2, r2);
  f.addRange(3, r3);

  // Create initial value for optimization
  Values initial;
  initial.insert(1, pose1);
  initial.insert(2, pose2);
  initial.insert(3, Pose2(5, 6, 0)); // does not satisfy range measurement
  Vector actual5 = f.unwhitenedError(initial);
  EXPECT(assert_equal((Vector(1) << sqrt(25.0+16.0)-sqrt(50.0)).finished(), actual5));

  // Create Factor graph
  NonlinearFactorGraph graph;
  graph.push_back(f);
  const noiseModel::Base::shared_ptr //
  priorNoise = noiseModel::Diagonal::Sigmas(Vector3(1, 1, M_PI));
  graph.addPrior(1, pose1, priorNoise);
  graph.addPrior(2, pose2, priorNoise);

  // Try optimizing
  LevenbergMarquardtParams params;
  //  params.setVerbosity("ERROR");
  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  Values result = optimizer.optimize();
  EXPECT(assert_equal(initial.at<Pose2>(1), result.at<Pose2>(1)));
  EXPECT(assert_equal(initial.at<Pose2>(2), result.at<Pose2>(2)));
  // only the third pose will be changed, converges on following:
  EXPECT(assert_equal(Pose2(5.52159, 5.582727, 0), result.at<Pose2>(3),1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

