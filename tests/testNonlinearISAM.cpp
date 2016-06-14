/**
 * @file testNonlinearISAM
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/geometry/Pose2.h>


using namespace gtsam;

const double tol=1e-5;

/* ************************************************************************* */
TEST(testNonlinearISAM, markov_chain ) {
  int reorder_interval = 2;
  NonlinearISAM isamChol(reorder_interval, EliminatePreferCholesky); // create an ISAM object
  NonlinearISAM isamQR(reorder_interval, EliminateQR); // create an ISAM object

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector3(3.0, 3.0, 0.5));
  Sampler sampler(model, 42u);

  // create initial graph
  Pose2 cur_pose; // start at origin
  NonlinearFactorGraph start_factors;
  start_factors += NonlinearEquality<Pose2>(0, cur_pose);

  Values init;
  Values expected;
  init.insert(0, cur_pose);
  expected.insert(0, cur_pose);
  isamChol.update(start_factors, init);
  isamQR.update(start_factors, init);

  // loop for a period of time to verify memory usage
  size_t nrPoses = 21;
  Pose2 z(1.0, 2.0, 0.1);
  for (size_t i=1; i<=nrPoses; ++i) {
    NonlinearFactorGraph new_factors;
    new_factors += BetweenFactor<Pose2>(i-1, i, z, model);
    Values new_init;

    cur_pose = cur_pose.compose(z);
    new_init.insert(i, cur_pose.retract(sampler.sample()));
    expected.insert(i, cur_pose);
    isamChol.update(new_factors, new_init);
    isamQR.update(new_factors, new_init);
  }

  // verify values - all but the last one should be very close
  Values actualChol = isamChol.estimate();
  for (size_t i=0; i<nrPoses; ++i) {
    EXPECT(assert_equal(expected.at<Pose2>(i), actualChol.at<Pose2>(i), tol));
  }
  Values actualQR = isamQR.estimate();
  for (size_t i=0; i<nrPoses; ++i) {
    EXPECT(assert_equal(expected.at<Pose2>(i), actualQR.at<Pose2>(i), tol));
  }
}

/* ************************************************************************* */
TEST(testNonlinearISAM, markov_chain_with_disconnects ) {
  int reorder_interval = 2;
  NonlinearISAM isamChol(reorder_interval, EliminatePreferCholesky); // create an ISAM object
  NonlinearISAM isamQR(reorder_interval, EliminateQR); // create an ISAM object

  SharedDiagonal model3 = noiseModel::Diagonal::Sigmas(Vector3(3.0, 3.0, 0.5));
  SharedDiagonal model2 = noiseModel::Diagonal::Sigmas(Vector2(2.0, 2.0));
  Sampler sampler(model3, 42u);

  // create initial graph
  Pose2 cur_pose; // start at origin
  NonlinearFactorGraph start_factors;
  start_factors += NonlinearEquality<Pose2>(0, cur_pose);

  Values init;
  Values expected;
  init.insert(0, cur_pose);
  expected.insert(0, cur_pose);
  isamChol.update(start_factors, init);
  isamQR.update(start_factors, init);

  size_t nrPoses = 21;

  // create a constrained constellation of landmarks
  Key lm1 = nrPoses+1, lm2 = nrPoses+2, lm3 = nrPoses+3;
  Point2 landmark1(3., 4.), landmark2(6., 4.), landmark3(6., 9.);
  expected.insert(lm1, landmark1);
  expected.insert(lm2, landmark2);
  expected.insert(lm3, landmark3);

  // loop for a period of time to verify memory usage
  Pose2 z(1.0, 2.0, 0.1);
  for (size_t i=1; i<=nrPoses; ++i) {
    NonlinearFactorGraph new_factors;
    new_factors += BetweenFactor<Pose2>(i-1, i, z, model3);
    Values new_init;

    cur_pose = cur_pose.compose(z);
    new_init.insert(i, cur_pose.retract(sampler.sample()));
    expected.insert(i, cur_pose);

    // Add a floating landmark constellation
    if (i == 7) {
      new_factors += PriorFactor<Point2>(lm1, landmark1, model2);
      new_factors += PriorFactor<Point2>(lm2, landmark2, model2);
      new_factors += PriorFactor<Point2>(lm3, landmark3, model2);

      // Initialize to origin
      new_init.insert(lm1, Point2(0,0));
      new_init.insert(lm2, Point2(0,0));
      new_init.insert(lm3, Point2(0,0));
    }

    isamChol.update(new_factors, new_init);
    isamQR.update(new_factors, new_init);
  }

  // verify values - all but the last one should be very close
  Values actualChol = isamChol.estimate();
  for (size_t i=0; i<nrPoses; ++i)
    EXPECT(assert_equal(expected.at<Pose2>(i), actualChol.at<Pose2>(i), tol));

  Values actualQR = isamQR.estimate();
  for (size_t i=0; i<nrPoses; ++i)
    EXPECT(assert_equal(expected.at<Pose2>(i), actualQR.at<Pose2>(i), tol));

  // Check landmarks
  EXPECT(assert_equal(expected.at<Point2>(lm1), actualChol.at<Point2>(lm1), tol));
  EXPECT(assert_equal(expected.at<Point2>(lm2), actualChol.at<Point2>(lm2), tol));
  EXPECT(assert_equal(expected.at<Point2>(lm3), actualChol.at<Point2>(lm3), tol));

  EXPECT(assert_equal(expected.at<Point2>(lm1), actualQR.at<Point2>(lm1), tol));
  EXPECT(assert_equal(expected.at<Point2>(lm2), actualQR.at<Point2>(lm2), tol));
  EXPECT(assert_equal(expected.at<Point2>(lm3), actualQR.at<Point2>(lm3), tol));
}

/* ************************************************************************* */
TEST(testNonlinearISAM, markov_chain_with_reconnect ) {
  int reorder_interval = 2;
  NonlinearISAM isamChol(reorder_interval, EliminatePreferCholesky); // create an ISAM object
  NonlinearISAM isamQR(reorder_interval, EliminateQR); // create an ISAM object

  SharedDiagonal model3 = noiseModel::Diagonal::Sigmas(Vector3(3.0, 3.0, 0.5));
  SharedDiagonal model2 = noiseModel::Diagonal::Sigmas(Vector2(2.0, 2.0));
  Sampler sampler(model3, 42u);

  // create initial graph
  Pose2 cur_pose; // start at origin
  NonlinearFactorGraph start_factors;
  start_factors += NonlinearEquality<Pose2>(0, cur_pose);

  Values init;
  Values expected;
  init.insert(0, cur_pose);
  expected.insert(0, cur_pose);
  isamChol.update(start_factors, init);
  isamQR.update(start_factors, init);

  size_t nrPoses = 21;

  // create a constrained constellation of landmarks
  Key lm1 = nrPoses+1, lm2 = nrPoses+2, lm3 = nrPoses+3;
  Point2 landmark1(3., 4.), landmark2(6., 4.), landmark3(6., 9.);
  expected.insert(lm1, landmark1);
  expected.insert(lm2, landmark2);
  expected.insert(lm3, landmark3);

  // loop for a period of time to verify memory usage
  Pose2 z(1.0, 2.0, 0.1);
  for (size_t i=1; i<=nrPoses; ++i) {
    NonlinearFactorGraph new_factors;
    new_factors += BetweenFactor<Pose2>(i-1, i, z, model3);
    Values new_init;

    cur_pose = cur_pose.compose(z);
    new_init.insert(i, cur_pose.retract(sampler.sample()));
    expected.insert(i, cur_pose);

    // Add a floating landmark constellation
    if (i == 7) {
      new_factors += PriorFactor<Point2>(lm1, landmark1, model2);
      new_factors += PriorFactor<Point2>(lm2, landmark2, model2);
      new_factors += PriorFactor<Point2>(lm3, landmark3, model2);

      // Initialize to origin
      new_init.insert(lm1, Point2(0,0));
      new_init.insert(lm2, Point2(0,0));
      new_init.insert(lm3, Point2(0,0));
    }

    // Reconnect with observation later
    if (i == 15) {
      new_factors += BearingRangeFactor<Pose2, Point2>(
          i, lm1, cur_pose.bearing(landmark1), cur_pose.range(landmark1), model2);
    }

    isamChol.update(new_factors, new_init);
    isamQR.update(new_factors, new_init);
  }

  // verify values - all but the last one should be very close
  Values actualChol = isamChol.estimate();
  for (size_t i=0; i<nrPoses; ++i)
    EXPECT(assert_equal(expected.at<Pose2>(i), actualChol.at<Pose2>(i), 1e-4));

  Values actualQR = isamQR.estimate();
  for (size_t i=0; i<nrPoses; ++i)
    EXPECT(assert_equal(expected.at<Pose2>(i), actualQR.at<Pose2>(i), 1e-4));

  // Check landmarks
  EXPECT(assert_equal(expected.at<Point2>(lm1), actualChol.at<Point2>(lm1), tol));
  EXPECT(assert_equal(expected.at<Point2>(lm2), actualChol.at<Point2>(lm2), tol));
  EXPECT(assert_equal(expected.at<Point2>(lm3), actualChol.at<Point2>(lm3), tol));

  EXPECT(assert_equal(expected.at<Point2>(lm1), actualQR.at<Point2>(lm1), tol));
  EXPECT(assert_equal(expected.at<Point2>(lm2), actualQR.at<Point2>(lm2), tol));
  EXPECT(assert_equal(expected.at<Point2>(lm3), actualQR.at<Point2>(lm3), tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
