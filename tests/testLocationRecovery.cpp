/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testLocationRecovery.cpp
 * @author Kathir Gounder
 * @date April 2026
 * @brief Test LocationRecovery base class: recover Point3 positions from
 *        Unit3 direction measurements with both chordal and bilinear costs.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/sfm/LocationRecovery.h>

using namespace std;
using namespace gtsam;

// Simple 3-node problem: three points with known directions between them.
static const Point3 kP0(0, 0, 0);
static const Point3 kP1(2, 0, 0);
static const Point3 kP2(1, -1, 0);

// Build direction measurements between all pairs.
LocationRecovery::DirectionEdges SimulateEdges(
    const vector<pair<Key, Key>>& pairs,
    const map<Key, Point3>& positions) {
  auto noiseModel = noiseModel::Isotropic::Sigma(2, 0.01);
  LocationRecovery::DirectionEdges edges;
  for (const auto& [a, b] : pairs) {
    Unit3 direction(positions.at(b) - positions.at(a));
    edges.emplace_back(a, b, direction, noiseModel);
  }
  return edges;
}

/* ************************************************************************* */
// Test buildGraph with bilinear (BATA) factors.
TEST(LocationRecovery, BuildGraphBilinear) {
  map<Key, Point3> positions = {{0, kP0}, {1, kP1}, {2, kP2}};
  auto edges = SimulateEdges({{0, 1}, {0, 2}, {1, 2}}, positions);

  LocationRecovery lr;
  auto graph = lr.buildGraph(edges, /*bilinear=*/true);
  // 3 BATA factors (each connects 3 variables: two Point3 + one scale).
  EXPECT_LONGS_EQUAL(3, graph.size());
}

/* ************************************************************************* */
// Test buildGraph with chordal factors.
TEST(LocationRecovery, BuildGraphChordal) {
  map<Key, Point3> positions = {{0, kP0}, {1, kP1}, {2, kP2}};
  auto edges = SimulateEdges({{0, 1}, {0, 2}, {1, 2}}, positions);

  LocationRecovery lr;
  auto graph = lr.buildGraph(edges, /*bilinear=*/false);
  // 3 chordal factors (each connects 2 variables).
  EXPECT_LONGS_EQUAL(3, graph.size());
}

/* ************************************************************************* */
// Test that the base class can solve a complete problem end-to-end
// using the bilinear (BATA) cost function.
TEST(LocationRecovery, SolveBilinear) {
  map<Key, Point3> positions = {{0, kP0}, {1, kP1}, {2, kP2}};
  auto edges = SimulateEdges({{0, 1}, {0, 2}, {1, 2}}, positions);

  LocationRecovery lr;
  auto graph = lr.buildGraph(edges, /*bilinear=*/true);
  lr.addAnchorPrior(0, &graph);

  set<Key> keys = {0, 1, 2};
  auto initial = lr.initializeRandomly(keys, edges.size(), /*bilinear=*/true);

  LevenbergMarquardtOptimizer lm(graph, initial);
  Values result = lm.optimize();

  // Anchor at origin.
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0), 1e-3));

  // Check relative direction from key 0 to key 1 is along +X.
  Unit3 expected_dir(kP1 - kP0);
  Unit3 actual_dir(result.at<Point3>(1) - result.at<Point3>(0));
  EXPECT(assert_equal(expected_dir, actual_dir, 1e-2));
}

/* ************************************************************************* */
// Test that the base class can solve a complete problem end-to-end
// using the chordal cost function.
TEST(LocationRecovery, SolveChordal) {
  map<Key, Point3> positions = {{0, kP0}, {1, kP1}, {2, kP2}};
  auto edges = SimulateEdges({{0, 1}, {0, 2}, {1, 2}}, positions);

  LocationRecovery lr;
  auto graph = lr.buildGraph(edges, /*bilinear=*/false);
  lr.addAnchorPrior(0, &graph);

  set<Key> keys = {0, 1, 2};
  auto initial = lr.initializeRandomly(keys, edges.size(), /*bilinear=*/false);

  LevenbergMarquardtOptimizer lm(graph, initial);
  Values result = lm.optimize();

  // Anchor at origin.
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0), 1e-3));

  // Check relative direction from key 0 to key 1 is along +X.
  Unit3 expected_dir(kP1 - kP0);
  Unit3 actual_dir(result.at<Point3>(1) - result.at<Point3>(0));
  EXPECT(assert_equal(expected_dir, actual_dir, 1e-2));
}

/* ************************************************************************* */
// Test initializeRandomly creates correct number of variables.
TEST(LocationRecovery, InitializeRandomly) {
  map<Key, Point3> positions = {{0, kP0}, {1, kP1}};
  auto edges = SimulateEdges({{0, 1}}, positions);

  LocationRecovery lr;

  // Bilinear: 2 Point3 + 1 scale = 3 entries.
  set<Key> keys = {0, 1};
  auto init_bilinear = lr.initializeRandomly(keys, edges.size(), true);
  EXPECT_LONGS_EQUAL(3, init_bilinear.size());

  // Chordal: 2 Point3 + 0 scales = 2 entries.
  auto init_chordal = lr.initializeRandomly(keys, edges.size(), false);
  EXPECT_LONGS_EQUAL(2, init_chordal.size());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
