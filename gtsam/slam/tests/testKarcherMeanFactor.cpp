/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testKarcherMeanFactor.cpp
 * @author Frank Dellaert
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/KarcherMeanFactor-inl.h>
#include <gtsam/slam/KarcherMeanFactor.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Rot3 version
/* ************************************************************************* */
static const Rot3 R = Rot3::Expmap(Vector3(0.1, 0, 0));

/* ************************************************************************* */
// Check that optimizing for Karcher mean (which minimizes Between distance)
// gets correct result.
TEST(KarcherMean, FindRot3) {
  std::vector<Rot3> rotations = {R, R.inverse()};
  Rot3 expected;
  auto actual = FindKarcherMean(rotations);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
// Check that the InnerConstraint factor leaves the mean unchanged.
TEST(KarcherMean, FactorRot3) {
  // Make a graph with two variables, one between, and one InnerConstraint
  // The optimal result should satisfy the between, while moving the other
  // variable to make the mean the same as before.
  // Mean of R and R' is identity. Let's make a BetweenFactor making R21 =
  // R*R*R, i.e. geodesic length is 3 rather than 2.
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Rot3>>(1, 2, R * R * R);
  std::vector<Key> keys{1, 2};
  graph.emplace_shared<KarcherMeanFactor<Rot3>>(keys);

  Values initial;
  initial.insert<Rot3>(1, R.inverse());
  initial.insert<Rot3>(2, R);
  const auto expected = FindKarcherMean<Rot3>({R, R.inverse()});

  auto result = GaussNewtonOptimizer(graph, initial).optimize();
  const auto actual =
      FindKarcherMean<Rot3>({result.at<Rot3>(1), result.at<Rot3>(2)});
  EXPECT(assert_equal(expected, actual));
  EXPECT(
      assert_equal(R * R * R, result.at<Rot3>(1).between(result.at<Rot3>(2))));
}

/* ************************************************************************* */
// SO(4) version
/* ************************************************************************* */
static const SO4 Q = SO4::Expmap((Vector6() << 1, 2, 3, 4, 5, 6).finished());

/* ************************************************************************* */
TEST(KarcherMean, FindSO4) {
  std::vector<SO4, Eigen::aligned_allocator<SO4>> rotations = {Q, Q.inverse()};
  auto expected = SO4();  //::ChordalMean(rotations);
  auto actual = FindKarcherMean(rotations);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(KarcherMean, FactorSO4) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<SO4>>(1, 2, Q * Q * Q);
  std::vector<Key> keys{1, 2};
  graph.emplace_shared<KarcherMeanFactor<SO4>>(keys);

  Values initial;
  initial.insert<SO4>(1, Q.inverse());
  initial.insert<SO4>(2, Q);

  std::vector<SO4, Eigen::aligned_allocator<SO4> > rotations = {Q, Q.inverse()};
  const auto expected = FindKarcherMean<SO4>(rotations);

  auto result = GaussNewtonOptimizer(graph, initial).optimize();
  const auto actual =
      FindKarcherMean<SO4>({result.at<SO4>(1), result.at<SO4>(2)});
  EXPECT(assert_equal(expected, actual));
  EXPECT(assert_equal((Matrix)(Q * Q * Q).matrix(),
                      result.at<SO4>(1).between(result.at<SO4>(2)).matrix()));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
