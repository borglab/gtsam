/*
 * testGenericGraph.cpp
 *
 *   Created on: Nov 23, 2010
 *       Author: nikai
 *  Description: unit tests for generic graph
 */

#include <gtsam_unstable/partition/GenericGraph.h>

#include <CppUnitLite/TestHarness.h>


#include <map>

using namespace std;
using namespace gtsam;
using namespace gtsam::partition;

/* ************************************************************************* */
/**
 *      l7               l9
 *     / | \            / |
 *   x1 -x2-x3 - l8 - x4- x5-x6
 */
TEST ( GenerciGraph, findIslands )
{
  GenericGraph2D graph;
  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 7, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 7, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(3, NODE_POSE_2D, 7, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(3, NODE_POSE_2D, 8, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(4, NODE_POSE_2D, 8, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(4, NODE_POSE_2D, 9, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(5, NODE_POSE_2D, 9, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(6, NODE_POSE_2D, 9, NODE_LANDMARK_2D));

  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 2, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 3, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(4, NODE_POSE_2D, 5, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(5, NODE_POSE_2D, 6, NODE_POSE_2D));
  std::vector<size_t> keys{1, 2, 3, 4, 5, 6, 7, 8, 9};

  WorkSpace workspace(10); // from 0 to 9
  list<vector<size_t> > islands = findIslands(graph, keys, workspace, 7, 2);
  LONGS_EQUAL(2, islands.size());
  vector<size_t> island1{1, 2, 3, 7, 8};
  vector<size_t> island2{4, 5, 6, 9};
  CHECK(island1 == islands.front());
  CHECK(island2 == islands.back());
}

/* ************************************************************************* */
/**
 *         l7    l8
 *     / /  |  X  |  \
 *   x1 -x2-x3   x4- x5-x6
 */
TEST( GenerciGraph, findIslands2 )
{
  GenericGraph2D graph;
  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 7, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 7, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(3, NODE_POSE_2D, 7, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(3, NODE_POSE_2D, 8, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(4, NODE_POSE_2D, 7, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(4, NODE_POSE_2D, 8, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(5, NODE_POSE_2D, 8, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(6, NODE_POSE_2D, 8, NODE_LANDMARK_2D));

  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 2, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 3, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(4, NODE_POSE_2D, 5, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(5, NODE_POSE_2D, 6, NODE_POSE_2D));
  std::vector<size_t> keys{1, 2, 3, 4, 5, 6, 7, 8};

  WorkSpace workspace(15); // from 0 to 8, but testing over-allocation here
  list<vector<size_t> > islands = findIslands(graph, keys, workspace, 7, 2);
  LONGS_EQUAL(1, islands.size());
  vector<size_t> island1{1, 2, 3, 4, 5, 6, 7, 8};
  CHECK(island1 == islands.front());
}

/* ************************************************************************* */
/**
 *     x1 - l5
 *     x2 - x3 - x4 - l6
 */
TEST ( GenerciGraph, findIslands3 )
{
  GenericGraph2D graph;
  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 5, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(4, NODE_POSE_2D, 6, NODE_LANDMARK_2D));

  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 3, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(3, NODE_POSE_2D, 4, NODE_POSE_2D));
  std::vector<size_t> keys{1, 2, 3, 4, 5, 6};

  WorkSpace workspace(7); // from 0 to 9
  list<vector<size_t> > islands = findIslands(graph, keys, workspace, 7, 2);
  LONGS_EQUAL(2, islands.size());
  vector<size_t> island1{1, 5};
  vector<size_t> island2{2, 3, 4, 6};
  CHECK(island1 == islands.front());
  CHECK(island2 == islands.back());
}

/* ************************************************************************* */
/**
 *     x3 - l4 - x7
 */
TEST ( GenerciGraph, findIslands4 )
{
  GenericGraph2D graph;
  graph.push_back(std::make_shared<GenericFactor2D>(3, NODE_POSE_2D, 4, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(7, NODE_POSE_2D, 7, NODE_LANDMARK_2D));
  std::vector<size_t> keys{3, 4, 7};

  WorkSpace workspace(8); // from 0 to 7
  list<vector<size_t> > islands = findIslands(graph, keys, workspace, 7, 2);
  LONGS_EQUAL(2, islands.size());
  vector<size_t> island1{3, 4};
  vector<size_t> island2{7};
  CHECK(island1 == islands.front());
  CHECK(island2 == islands.back());
}

/* ************************************************************************* */
/**
 *     x1 - l5 - x2
 *      | /    \ |
 *     x3        x4
 */
TEST ( GenerciGraph, findIslands5 )
{
  GenericGraph2D graph;
  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 5, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 5, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(3, NODE_POSE_2D, 5, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(4, NODE_POSE_2D, 5, NODE_LANDMARK_2D));

  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 3, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 4, NODE_POSE_2D));

  std::vector<size_t> keys{1, 2, 3, 4, 5};

  WorkSpace workspace(6); // from 0 to 5
  list<vector<size_t> > islands = findIslands(graph, keys, workspace, 7, 2);
  LONGS_EQUAL(2, islands.size());
  vector<size_t> island1{1, 3, 5};
  vector<size_t> island2{2, 4};
  CHECK(island1 == islands.front());
  CHECK(island2 == islands.back());
}

/* ************************************************************************* */
/**
 *     l3  l4  l5  l6
 *       \ | /  \  /
 *         x1    x2
 */
TEST ( GenerciGraph, reduceGenericGraph )
{
  GenericGraph3D graph;
  graph.push_back(std::make_shared<GenericFactor3D>(1, 3));
  graph.push_back(std::make_shared<GenericFactor3D>(1, 4));
  graph.push_back(std::make_shared<GenericFactor3D>(1, 5));
  graph.push_back(std::make_shared<GenericFactor3D>(2, 5));
  graph.push_back(std::make_shared<GenericFactor3D>(2, 6));

  std::vector<size_t> cameraKeys, landmarkKeys;
  cameraKeys.push_back(1);
  cameraKeys.push_back(2);
  landmarkKeys.push_back(3);
  landmarkKeys.push_back(4);
  landmarkKeys.push_back(5);
  landmarkKeys.push_back(6);

  std::vector<int> dictionary;
  dictionary.resize(7, -1); // from 0 to 6
  dictionary[1] = 0;
  dictionary[2] = 1;

  GenericGraph3D reduced;
  std::map<size_t, vector<size_t> > cameraToLandmarks;
  reduceGenericGraph(graph, cameraKeys, landmarkKeys, dictionary, reduced);
  LONGS_EQUAL(1, reduced.size());
  LONGS_EQUAL(1, reduced[0]->key1.index); LONGS_EQUAL(2, reduced[0]->key2.index);
}

/* ************************************************************************* */
/**
 *     l3  l4  l5  l6
 *       \ | /  \  /
 *         x1    x2 - x7
 */
TEST ( GenericGraph, reduceGenericGraph2 )
{
  GenericGraph3D graph;
  graph.push_back(std::make_shared<GenericFactor3D>(1, 3, 0, NODE_POSE_3D, NODE_LANDMARK_3D));
  graph.push_back(std::make_shared<GenericFactor3D>(1, 4, 1, NODE_POSE_3D, NODE_LANDMARK_3D));
  graph.push_back(std::make_shared<GenericFactor3D>(1, 5, 2, NODE_POSE_3D, NODE_LANDMARK_3D));
  graph.push_back(std::make_shared<GenericFactor3D>(2, 5, 3, NODE_POSE_3D, NODE_LANDMARK_3D));
  graph.push_back(std::make_shared<GenericFactor3D>(2, 6, 4, NODE_POSE_3D, NODE_LANDMARK_3D));
  graph.push_back(std::make_shared<GenericFactor3D>(2, 7, 5, NODE_POSE_3D, NODE_POSE_3D));

  std::vector<size_t> cameraKeys, landmarkKeys;
  cameraKeys.push_back(1);
  cameraKeys.push_back(2);
  cameraKeys.push_back(7);
  landmarkKeys.push_back(3);
  landmarkKeys.push_back(4);
  landmarkKeys.push_back(5);
  landmarkKeys.push_back(6);

  std::vector<int> dictionary;
  dictionary.resize(8, -1); // from 0 to 7
  dictionary[1] = 0;
  dictionary[2] = 1;
  dictionary[7] = 6;

  GenericGraph3D reduced;
  std::map<size_t, vector<size_t> > cameraToLandmarks;
  reduceGenericGraph(graph, cameraKeys, landmarkKeys, dictionary, reduced);
  LONGS_EQUAL(2, reduced.size());
  LONGS_EQUAL(1, reduced[0]->key1.index); LONGS_EQUAL(2, reduced[0]->key2.index);
  LONGS_EQUAL(2, reduced[1]->key1.index); LONGS_EQUAL(7, reduced[1]->key2.index);
}

/* ************************************************************************* */
TEST ( GenerciGraph, hasCommonCamera )
{
  std::set<size_t> cameras1{1, 2, 3, 4, 5};
  std::set<size_t> cameras2{8, 7, 6, 5};
  bool actual = hasCommonCamera(cameras1, cameras2);
  CHECK(actual);
}

/* ************************************************************************* */
TEST ( GenerciGraph, hasCommonCamera2 )
{
  std::set<size_t> cameras1{1, 3, 5, 7};
  std::set<size_t> cameras2{2, 4, 6, 8, 10};
  bool actual = hasCommonCamera(cameras1, cameras2);
  CHECK(!actual);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
