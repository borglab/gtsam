/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGraph.cpp
 * @date Jan 12, 2010
 * @author nikai
 * @brief unit test for graph-inl.h
 */

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/graph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>

#include <CppUnitLite/TestHarness.h>

#include <memory>

#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// x1 -> x2
//    -> x3 -> x4
//    -> x5
TEST ( Ordering, predecessorMap2Keys ) {
  PredecessorMap<Key> p_map;
  p_map.insert(1,1);
  p_map.insert(2,1);
  p_map.insert(3,1);
  p_map.insert(4,3);
  p_map.insert(5,1);

  list<Key> expected{4, 5, 3, 2, 1};

  list<Key> actual = predecessorMap2Keys<Key>(p_map);
  LONGS_EQUAL((long)expected.size(), (long)actual.size());

  list<Key>::const_iterator it1 = expected.begin();
  list<Key>::const_iterator it2 = actual.begin();
  for(; it1!=expected.end(); it1++, it2++)
    CHECK(*it1 == *it2)
}

/* ************************************************************************* */
TEST( Graph, predecessorMap2Graph )
{
  typedef SGraph<string>::Vertex SVertex;
  SGraph<Key> graph;
  SVertex root;
  map<Key, SVertex> key2vertex;

  PredecessorMap<Key> p_map;
  p_map.insert(1, 2);
  p_map.insert(2, 2);
  p_map.insert(3, 2);
  std::tie(graph, root, key2vertex) = predecessorMap2Graph<SGraph<Key>, SVertex, Key>(p_map);

  LONGS_EQUAL(3, (long)boost::num_vertices(graph));
  CHECK(root == key2vertex[2]);
}

/* ************************************************************************* */
TEST( Graph, composePoses )
{
  NonlinearFactorGraph graph;
  SharedNoiseModel cov = noiseModel::Unit::Create(3);
  Pose2 p1(1.0, 2.0, 0.3), p2(4.0, 5.0, 0.6), p3(7.0, 8.0, 0.9), p4(2.0, 2.0, 2.9);
  Pose2 p12=p1.between(p2), p23=p2.between(p3), p43=p4.between(p3);
  graph += BetweenFactor<Pose2>(1,2, p12, cov);
  graph += BetweenFactor<Pose2>(2,3, p23, cov);
  graph += BetweenFactor<Pose2>(4,3, p43, cov);

  PredecessorMap<Key> tree;
  tree.insert(1,2);
  tree.insert(2,2);
  tree.insert(3,2);
  tree.insert(4,3);

  Pose2 rootPose = p2;

  std::shared_ptr<Values> actual = composePoses<NonlinearFactorGraph, BetweenFactor<Pose2>, Pose2, Key> (graph, tree, rootPose);

  Values expected;
  expected.insert(1, p1);
  expected.insert(2, p2);
  expected.insert(3, p3);
  expected.insert(4, p4);

  LONGS_EQUAL(4, (long)actual->size());
  CHECK(assert_equal(expected, *actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, findMinimumSpanningTree )
{
  GaussianFactorGraph g;
  Matrix I = I_2x2;
  Vector2 b(0, 0);
  const SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.5));
  using namespace symbol_shorthand;
  g += JacobianFactor(X(1), I, X(2), I, b, model);
  g += JacobianFactor(X(1), I, X(3), I, b, model);
  g += JacobianFactor(X(1), I, X(4), I, b, model);
  g += JacobianFactor(X(2), I, X(3), I, b, model);
  g += JacobianFactor(X(2), I, X(4), I, b, model);
  g += JacobianFactor(X(3), I, X(4), I, b, model);

  PredecessorMap<Key> tree = findMinimumSpanningTree<GaussianFactorGraph, Key, JacobianFactor>(g);
  EXPECT_LONGS_EQUAL(X(1),tree[X(1)]);
  EXPECT_LONGS_EQUAL(X(1),tree[X(2)]);
  EXPECT_LONGS_EQUAL(X(1),tree[X(3)]);
  EXPECT_LONGS_EQUAL(X(1),tree[X(4)]);

  // we add a disconnected component - does not work yet
  //  g += JacobianFactor(X(5), I, X(6), I, b, model);
  //
  //  PredecessorMap<Key> forest = findMinimumSpanningTree<GaussianFactorGraph, Key, JacobianFactor>(g);
  //  EXPECT_LONGS_EQUAL(X(1),forest[X(1)]);
  //  EXPECT_LONGS_EQUAL(X(1),forest[X(2)]);
  //  EXPECT_LONGS_EQUAL(X(1),forest[X(3)]);
  //  EXPECT_LONGS_EQUAL(X(1),forest[X(4)]);
  //  EXPECT_LONGS_EQUAL(X(5),forest[X(5)]);
  //  EXPECT_LONGS_EQUAL(X(5),forest[X(6)]);
}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactorGraph, split )
//{
//  GaussianFactorGraph g;
//  Matrix I = eye(2);
//  Vector b = Vector_(0, 0, 0);
//  g += X(1), I, X(2), I, b, model;
//  g += X(1), I, X(3), I, b, model;
//  g += X(1), I, X(4), I, b, model;
//  g += X(2), I, X(3), I, b, model;
//  g += X(2), I, X(4), I, b, model;
//
//  PredecessorMap<string> tree;
//  tree[X(1)] = X(1);
//  tree[X(2)] = X(1);
//  tree[X(3)] = X(1);
//  tree[X(4)] = X(1);
//
//  GaussianFactorGraph Ab1, Ab2;
//  g.split<string, GaussianFactor>(tree, Ab1, Ab2);
//  LONGS_EQUAL(3, Ab1.size());
//  LONGS_EQUAL(2, Ab2.size());
//}

///* ************************************************************************* */
// SL-FIX TEST( FactorGraph, splitMinimumSpanningTree )
//{
//  SymbolicFactorGraph G;
//  G.push_factor("x1", "x2");
//  G.push_factor("x1", "x3");
//  G.push_factor("x1", "x4");
//  G.push_factor("x2", "x3");
//  G.push_factor("x2", "x4");
//  G.push_factor("x3", "x4");
//
//  SymbolicFactorGraph T, C;
//  std::tie(T, C) = G.splitMinimumSpanningTree();
//
//  SymbolicFactorGraph expectedT, expectedC;
//  expectedT.push_factor("x1", "x2");
//  expectedT.push_factor("x1", "x3");
//  expectedT.push_factor("x1", "x4");
//  expectedC.push_factor("x2", "x3");
//  expectedC.push_factor("x2", "x4");
//  expectedC.push_factor("x3", "x4");
//  CHECK(assert_equal(expectedT,T));
//  CHECK(assert_equal(expectedC,C));
//}

///* ************************************************************************* */
///**
// *  x1 - x2 - x3 - x4 - x5
// *       |    |  / |
// *       l1   l2   l3
// */
// SL-FIX TEST( FactorGraph, removeSingletons )
//{
//  SymbolicFactorGraph G;
//  G.push_factor("x1", "x2");
//  G.push_factor("x2", "x3");
//  G.push_factor("x3", "x4");
//  G.push_factor("x4", "x5");
//  G.push_factor("x2", "l1");
//  G.push_factor("x3", "l2");
//  G.push_factor("x4", "l2");
//  G.push_factor("x4", "l3");
//
//  SymbolicFactorGraph singletonGraph;
//  set<Symbol> singletons;
//  std::tie(singletonGraph, singletons) = G.removeSingletons();
//
//  set<Symbol> singletons_excepted; singletons_excepted += "x1", "x2", "x5", "l1", "l3";
//  CHECK(singletons_excepted == singletons);
//
//  SymbolicFactorGraph singletonGraph_excepted;
//  singletonGraph_excepted.push_factor("x2", "l1");
//  singletonGraph_excepted.push_factor("x4", "l3");
//  singletonGraph_excepted.push_factor("x1", "x2");
//  singletonGraph_excepted.push_factor("x4", "x5");
//  singletonGraph_excepted.push_factor("x2", "x3");
//  CHECK(singletonGraph_excepted.equals(singletonGraph));
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
