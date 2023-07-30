/*
 * testFindSeparator.cpp
 *
 *   Created on: Nov 23, 2010
 *       Author: nikai
 *  Description: unit tests for FindSeparator
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/partition/FindSeparator-inl.h>
#include <gtsam_unstable/partition/GenericGraph.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::partition;

/* ************************************************************************* */
// x0 - x1 - x2
// l3        l4
TEST ( Partition, separatorPartitionByMetis )
{
  GenericGraph2D graph;
  graph.push_back(std::make_shared<GenericFactor2D>(0, NODE_POSE_2D, 3, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 4, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(0, NODE_POSE_2D, 1, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 2, NODE_POSE_2D));
  std::vector<size_t> keys{0, 1, 2, 3, 4};

  WorkSpace workspace(5);
  std::optional<MetisResult> actual = separatorPartitionByMetis<GenericGraph2D>(graph, keys,
   workspace, true);

  CHECK(actual.has_value());
  vector<size_t> A_expected{0, 3}; // frontal
  vector<size_t> B_expected{2, 4}; // frontal
  vector<size_t> C_expected{1};    // separator
  CHECK(A_expected == actual->A);
  CHECK(B_expected == actual->B);
  CHECK(C_expected == actual->C);
}

/* ************************************************************************* */
// x1 - x2 - x3,     variable  not used x0, x4, l7
// l5        l6
TEST ( Partition, separatorPartitionByMetis2 )
{
  GenericGraph2D graph;
  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 5, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(3, NODE_POSE_2D, 6, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 2, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 3, NODE_POSE_2D));
  std::vector<size_t> keys{1, 2, 3, 5, 6};

  WorkSpace workspace(8);
  std::optional<MetisResult> actual = separatorPartitionByMetis<GenericGraph2D>(graph, keys,
   workspace, true);

  CHECK(actual.has_value());
  vector<size_t> A_expected{1, 5}; // frontal
  vector<size_t> B_expected{3, 6}; // frontal
  vector<size_t> C_expected{2};    // separator
  CHECK(A_expected == actual->A);
  CHECK(B_expected == actual->B);
  CHECK(C_expected == actual->C);
}

/* *************************************************************************/
// x0 - x1 - x2 - x3
TEST ( Partition, edgePartitionByMetis )
{
  GenericGraph3D graph;
  graph.push_back(std::make_shared<GenericFactor3D>(0, 1, 0, NODE_POSE_3D, NODE_POSE_3D));
  graph.push_back(std::make_shared<GenericFactor3D>(1, 2, 1, NODE_POSE_3D, NODE_POSE_3D));
  graph.push_back(std::make_shared<GenericFactor3D>(2, 3, 2, NODE_POSE_3D, NODE_POSE_3D));
  std::vector<size_t> keys{0, 1, 2, 3};

  WorkSpace workspace(6);
  std::optional<MetisResult> actual = edgePartitionByMetis<GenericGraph3D>(graph, keys,
   workspace, true);

  CHECK(actual.has_value());
  vector<size_t> A_expected{0, 1}; // frontal
  vector<size_t> B_expected{2, 3}; // frontal
  vector<size_t> C_expected;    // separator
//  for(const size_t a: actual->A)
//    cout << a << " ";
//  cout << endl;
//  for(const size_t b: actual->B)
//    cout << b << " ";
//  cout << endl;

  CHECK(A_expected == actual->A || A_expected == actual->B);
  CHECK(B_expected == actual->B || B_expected == actual->A);
  CHECK(C_expected == actual->C);
}

/* *************************************************************************/
// x0 - x1 - x2 - x3 - x4
TEST ( Partition, edgePartitionByMetis2 )
{
  GenericGraph3D graph;
  graph.push_back(std::make_shared<GenericFactor3D>(0, 1, 0, NODE_POSE_3D, NODE_POSE_3D, 1));
  graph.push_back(std::make_shared<GenericFactor3D>(1, 2, 1, NODE_POSE_3D, NODE_POSE_3D, 1));
  graph.push_back(std::make_shared<GenericFactor3D>(2, 3, 2, NODE_POSE_3D, NODE_POSE_3D, 20));
  graph.push_back(std::make_shared<GenericFactor3D>(3, 4, 3, NODE_POSE_3D, NODE_POSE_3D, 1));
  std::vector<size_t> keys{0, 1, 2, 3, 4};

  WorkSpace workspace(6);
  std::optional<MetisResult> actual = edgePartitionByMetis<GenericGraph3D>(graph, keys,
   workspace, true);
  CHECK(actual.has_value());
  vector<size_t> A_expected{0, 1}; // frontal
  vector<size_t> B_expected{2, 3, 4}; // frontal
  vector<size_t> C_expected;    // separator
  CHECK(A_expected == actual->A);
  CHECK(B_expected == actual->B);
  CHECK(C_expected == actual->C);
}

/* ************************************************************************* */
// x0 - x1 - x2
// l3        l4
TEST ( Partition, findSeparator )
{
  GenericGraph2D graph;
  graph.push_back(std::make_shared<GenericFactor2D>(0, NODE_POSE_2D, 3, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 4, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(0, NODE_POSE_2D, 1, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 2, NODE_POSE_2D));
  std::vector<size_t> keys{0, 1, 2, 3, 4};

  WorkSpace workspace(5);
  int minNodesPerMap = -1;
  bool reduceGraph = false;
  int numSubmaps = findSeparator<GenericGraph2D>(graph, keys, minNodesPerMap, workspace,
    false, {}, reduceGraph, 0, 0);
  LONGS_EQUAL(2, numSubmaps);
  LONGS_EQUAL(5, workspace.partitionTable.size());
  LONGS_EQUAL(1, workspace.partitionTable[0]);
  LONGS_EQUAL(0, workspace.partitionTable[1]);
  LONGS_EQUAL(2, workspace.partitionTable[2]);
  LONGS_EQUAL(1, workspace.partitionTable[3]);
  LONGS_EQUAL(2, workspace.partitionTable[4]);
}

/* ************************************************************************* */
// x1 - x2 - x3,     variable  not used x0, x4, l7
// l5        l6
TEST ( Partition, findSeparator2 )
{
  GenericGraph2D graph;
  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 5, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(3, NODE_POSE_2D, 6, NODE_LANDMARK_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(1, NODE_POSE_2D, 2, NODE_POSE_2D));
  graph.push_back(std::make_shared<GenericFactor2D>(2, NODE_POSE_2D, 3, NODE_POSE_2D));
  std::vector<size_t> keys{1, 2, 3, 5, 6};

  WorkSpace workspace(8);
  int minNodesPerMap = -1;
  bool reduceGraph = false;
  int numSubmaps = findSeparator<GenericGraph2D>(graph, keys, minNodesPerMap, workspace,
    false, {}, reduceGraph, 0, 0);
  LONGS_EQUAL(2, numSubmaps);
  LONGS_EQUAL(8, workspace.partitionTable.size());
  LONGS_EQUAL(-1,workspace.partitionTable[0]);
  LONGS_EQUAL(1, workspace.partitionTable[1]);
  LONGS_EQUAL(0, workspace.partitionTable[2]);
  LONGS_EQUAL(2, workspace.partitionTable[3]);
  LONGS_EQUAL(-1,workspace.partitionTable[4]);
  LONGS_EQUAL(1, workspace.partitionTable[5]);
  LONGS_EQUAL(2, workspace.partitionTable[6]);
  LONGS_EQUAL(-1,workspace.partitionTable[7]);
}

/* *************************************************************************/
/**
 *  l1-l8   l9-l16    l17-l24
 *   / |     /    \    |  \
 * x25 x26             x27 x28
 */
TEST ( Partition, findSeparator3_with_reduced_camera )
{
  GenericGraph3D graph;
  for (int j=1; j<=8; j++)
    graph.push_back(std::make_shared<GenericFactor3D>(25, j));
  for (int j=1; j<=16; j++)
    graph.push_back(std::make_shared<GenericFactor3D>(26, j));
  for (int j=9; j<=24; j++)
    graph.push_back(std::make_shared<GenericFactor3D>(27, j));
  for (int j=17; j<=24; j++)
    graph.push_back(std::make_shared<GenericFactor3D>(28, j));

  std::vector<size_t> keys;
  for(int i=1; i<=28; i++)
    keys.push_back(i);

  vector<Symbol> int2symbol;
  int2symbol.push_back(Symbol('x',0)); // dummy
  for(int i=1; i<=24; i++)
    int2symbol.push_back(Symbol('l',i));
  int2symbol.push_back(Symbol('x',25));
  int2symbol.push_back(Symbol('x',26));
  int2symbol.push_back(Symbol('x',27));
  int2symbol.push_back(Symbol('x',28));

  WorkSpace workspace(29);
  bool reduceGraph = true;
  int numIsland = findSeparator(graph, keys, 3, workspace, false, int2symbol, reduceGraph, 0, 0);
  LONGS_EQUAL(2, numIsland);

  partition::PartitionTable& partitionTable = workspace.partitionTable;
  for (int j=1; j<=8; j++)
    LONGS_EQUAL(1, partitionTable[j]);
  for (int j=9; j<=16; j++)
    LONGS_EQUAL(0, partitionTable[j]);
  for (int j=17; j<=24; j++)
    LONGS_EQUAL(2, partitionTable[j]);
  LONGS_EQUAL(1, partitionTable[25]);
  LONGS_EQUAL(1, partitionTable[26]);
  LONGS_EQUAL(2, partitionTable[27]);
  LONGS_EQUAL(2, partitionTable[28]);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
