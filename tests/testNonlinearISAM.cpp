/**
 * @file testNonlinearISAM
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/slam/BetweenFactor.h>
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

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector_(3, 3.0, 3.0, 0.5));
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
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
