/**
 * @file testNonlinearISAM
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/geometry/Pose2.h>

#include <iostream>
#include <sstream>

using namespace std;
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
  start_factors.emplace_shared<NonlinearEquality<Pose2>>(0, cur_pose);

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
    new_factors.emplace_shared<BetweenFactor<Pose2>>(i-1, i, z, model);
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
  start_factors.emplace_shared<NonlinearEquality<Pose2>>(0, cur_pose);

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
    new_factors.emplace_shared<BetweenFactor<Pose2>>(i-1, i, z, model3);
    Values new_init;

    cur_pose = cur_pose.compose(z);
    new_init.insert(i, cur_pose.retract(sampler.sample()));
    expected.insert(i, cur_pose);

    // Add a floating landmark constellation
    if (i == 7) {
      new_factors.addPrior(lm1, landmark1, model2);
      new_factors.addPrior(lm2, landmark2, model2);
      new_factors.addPrior(lm3, landmark3, model2);

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
  start_factors.emplace_shared<NonlinearEquality<Pose2>>(0, cur_pose);

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
    new_factors.emplace_shared<BetweenFactor<Pose2>>(i-1, i, z, model3);
    Values new_init;

    cur_pose = cur_pose.compose(z);
    new_init.insert(i, cur_pose.retract(sampler.sample()));
    expected.insert(i, cur_pose);

    // Add a floating landmark constellation
    if (i == 7) {
      new_factors.addPrior(lm1, landmark1, model2);
      new_factors.addPrior(lm2, landmark2, model2);
      new_factors.addPrior(lm3, landmark3, model2);

      // Initialize to origin
      new_init.insert(lm1, Point2(0,0));
      new_init.insert(lm2, Point2(0,0));
      new_init.insert(lm3, Point2(0,0));
    }

    // Reconnect with observation later
    if (i == 15) {
      new_factors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(
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
TEST(testNonlinearISAM, loop_closures ) {
  int relinearizeInterval = 100;
  NonlinearISAM isam(relinearizeInterval);

  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph graph;
  Values initialEstimate;

  vector<string> lines;
  lines.emplace_back("VERTEX2 0 0.000000 0.000000 0.000000");
  lines.emplace_back("EDGE2 1 0 1.030390 0.011350 -0.012958");
  lines.emplace_back("VERTEX2 1 1.030390 0.011350 -0.012958");
  lines.emplace_back("EDGE2 2 1 1.013900 -0.058639 -0.013225");
  lines.emplace_back("VERTEX2 2 2.043445 -0.060422 -0.026183");
  lines.emplace_back("EDGE2 3 2 1.027650 -0.007456 0.004833");
  lines.emplace_back("VERTEX2 3 3.070548 -0.094779 -0.021350");
  lines.emplace_back("EDGE2 4 3 -0.012016 1.004360 1.566790");
  lines.emplace_back("VERTEX2 4 3.079976 0.909609 1.545440");
  lines.emplace_back("EDGE2 5 4 1.016030 0.014565 -0.016304");
  lines.emplace_back("VERTEX2 5 3.091176 1.925681 1.529136");
  lines.emplace_back("EDGE2 6 5 1.023890 0.006808 0.010981");
  lines.emplace_back("VERTEX2 6 3.127018 2.948966 1.540117");
  lines.emplace_back("EDGE2 7 6 0.957734 0.003159 0.010901");
  lines.emplace_back("VERTEX2 7 3.153237 3.906347 1.551018");
  lines.emplace_back("EDGE2 8 7 -1.023820 -0.013668 -3.093240");
  lines.emplace_back("VERTEX2 8 3.146655 2.882457 -1.542222");
  lines.emplace_back("EDGE2 9 8 1.023440 0.013984 -0.007802");
  lines.emplace_back("EDGE2 9 5 0.033943 0.032439 -3.127400");
  lines.emplace_back("VERTEX2 9 3.189873 1.859834 -1.550024");
  lines.emplace_back("EDGE2 10 9 1.003350 0.022250 0.023491");
  lines.emplace_back("EDGE2 10 3 0.044020 0.988477 -1.563530");
  lines.emplace_back("VERTEX2 10 3.232959 0.857162 -1.526533");
  lines.emplace_back("EDGE2 11 10 0.977245 0.019042 -0.028623");
  lines.emplace_back("VERTEX2 11 3.295225 -0.118283 -1.555156");
  lines.emplace_back("EDGE2 12 11 -0.996880 -0.025512 -3.126915");
  lines.emplace_back("VERTEX2 12 3.254125 0.878076 1.601114");
  lines.emplace_back("EDGE2 13 12 0.990646 0.018396 -0.016519");
  lines.emplace_back("VERTEX2 13 3.205708 1.867709 1.584594");
  lines.emplace_back("EDGE2 14 13 0.945873 0.008893 -0.002602");
  lines.emplace_back("EDGE2 14 8 0.015808 0.021059 3.128310");
  lines.emplace_back("VERTEX2 14 3.183765 2.813370 1.581993");
  lines.emplace_back("EDGE2 15 14 1.000010 0.006428 0.028234");
  lines.emplace_back("EDGE2 15 7 -0.014728 -0.001595 -0.019579");
  lines.emplace_back("VERTEX2 15 3.166141 3.813245 1.610227");

  auto model = noiseModel::Diagonal::Sigmas(Vector3(3.0, 3.0, 0.5));

  // Loop over the different poses, adding the observations to iSAM incrementally
  for (const string& str : lines) {
    // scan the tag
    string tag;
    istringstream is(str);
    if (!(is >> tag))
      break;

    // Check if vertex
    const auto indexedPose = parseVertexPose(is, tag);
    if (indexedPose) {
      Key id = indexedPose->first;
      initialEstimate.insert(Symbol('x', id), indexedPose->second);
      if (id == 0) {
        noiseModel::Diagonal::shared_ptr priorNoise =
            noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
        graph.addPrior(Symbol('x', id), Pose2(0, 0, 0), priorNoise);
      } else {
        isam.update(graph, initialEstimate);

        // Clear the factor graph and values for the next iteration
        graph.resize(0);
        initialEstimate.clear();
      }
    }

    // check if edge
    const auto betweenPose = parseEdge(is, tag);
    if (betweenPose) {
      size_t id1, id2;
      tie(id1, id2) = betweenPose->first;
      graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x', id2),
          Symbol('x', id1), betweenPose->second, model);
    }
  }
  EXPECT_LONGS_EQUAL(16, isam.estimate().size())
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
