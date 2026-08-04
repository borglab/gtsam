/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicClusterTree.cpp
 * @brief   Unit tests for Cluster Tree
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>

#include "symbolicExampleGraphs.h"

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST(ClusterTree, SeparatorKeys) {
  const Ordering order{0, 1, 2, 3};

  SymbolicJunctionTree tree(SymbolicEliminationTree(simpleChain, order));

  auto root = tree.roots().front();
  auto child = root->children.front();

  const KeySet expectedRoot;
  const KeySet expectedChild{2};

  EXPECT(assert_container_equality(expectedRoot, root->separatorKeys()));
  EXPECT(assert_container_equality(expectedChild, child->separatorKeys()));

  SymbolicJunctionTree::Cluster::KeySetMap cache;
  EXPECT(assert_container_equality(expectedRoot, root->separatorKeys(&cache)));
  EXPECT(
      assert_container_equality(expectedChild, child->separatorKeys(&cache)));
}

/* ************************************************************************* */
TEST(ClusterTree, MergeChildren) {
  using Cluster = SymbolicJunctionTree::Cluster;
  auto parent = std::make_shared<Cluster>();
  auto child1 = std::make_shared<Cluster>();
  auto child2 = std::make_shared<Cluster>();
  auto child3 = std::make_shared<Cluster>();

  child1->orderedFrontalKeys.push_back(1);
  child2->orderedFrontalKeys.push_back(2);
  child3->orderedFrontalKeys.push_back(3);

  parent->addChild(child1);
  parent->addChild(child2);
  parent->addChild(child3);

  parent->mergeChildren({true, false, true});

  EXPECT_LONGS_EQUAL(1, parent->children.size());
  EXPECT(parent->children.front() == child2);
  const KeySet expected{1, 3};
  const KeySet actual(parent->orderedFrontalKeys.begin(),
                      parent->orderedFrontalKeys.end());
  EXPECT(assert_container_equality(expected, actual));
}

/* ************************************************************************* */
TEST(ClusterTree, MergeChildrenSiblings) {
  using Cluster = SymbolicJunctionTree::Cluster;
  auto parent = std::make_shared<Cluster>();
  auto child1 = std::make_shared<Cluster>();
  auto child2 = std::make_shared<Cluster>();
  auto child3 = std::make_shared<Cluster>();

  child1->orderedFrontalKeys.push_back(1);
  child2->orderedFrontalKeys.push_back(2);
  child3->orderedFrontalKeys.push_back(3);

  parent->addChild(child1);
  parent->addChild(child2);
  parent->addChild(child3);

  parent->mergeChildrenSiblings({true, false, true});

  EXPECT_LONGS_EQUAL(2, parent->children.size());
  EXPECT(parent->children[1] == child2);
  const KeySet expected{1, 3};
  const KeySet actual(parent->children[0]->orderedFrontalKeys.begin(),
                      parent->children[0]->orderedFrontalKeys.end());
  EXPECT(assert_container_equality(expected, actual));
}

/* ************************************************************************* */
TEST(ClusterTree, MergeChildrenWithGrandchildren) {
  using Cluster = SymbolicJunctionTree::Cluster;
  auto parent = std::make_shared<Cluster>();
  auto child1 = std::make_shared<Cluster>();
  auto child2 = std::make_shared<Cluster>();
  auto child3 = std::make_shared<Cluster>();
  auto grandchild1 = std::make_shared<Cluster>();
  auto grandchild2 = std::make_shared<Cluster>();

  child1->orderedFrontalKeys.push_back(1);
  child2->orderedFrontalKeys.push_back(2);
  child3->orderedFrontalKeys.push_back(3);
  grandchild1->orderedFrontalKeys.push_back(10);
  grandchild2->orderedFrontalKeys.push_back(20);

  child1->addChild(grandchild1);
  child2->addChild(grandchild2);
  parent->addChild(child1);
  parent->addChild(child2);
  parent->addChild(child3);

  parent->mergeChildren({child1, child2});

  EXPECT_LONGS_EQUAL(3, parent->children.size());
  EXPECT(parent->children[2] == child3);
  EXPECT(parent->children[0] == grandchild1);
  EXPECT(parent->children[1] == grandchild2);
  const KeySet expected{1, 2};
  const KeySet actual(parent->orderedFrontalKeys.begin(),
                      parent->orderedFrontalKeys.end());
  EXPECT(assert_container_equality(expected, actual));
}

/* ************************************************************************* */
TEST(ClusterTree, MergeChildrenSiblingsWithGrandchildren) {
  using Cluster = SymbolicJunctionTree::Cluster;
  auto parent = std::make_shared<Cluster>();
  auto child1 = std::make_shared<Cluster>();
  auto child2 = std::make_shared<Cluster>();
  auto child3 = std::make_shared<Cluster>();
  auto grandchild1 = std::make_shared<Cluster>();
  auto grandchild2 = std::make_shared<Cluster>();

  child1->orderedFrontalKeys.push_back(1);
  child2->orderedFrontalKeys.push_back(2);
  child3->orderedFrontalKeys.push_back(3);
  grandchild1->orderedFrontalKeys.push_back(10);
  grandchild2->orderedFrontalKeys.push_back(20);

  child1->addChild(grandchild1);
  child2->addChild(grandchild2);
  parent->addChild(child1);
  parent->addChild(child2);
  parent->addChild(child3);

  parent->mergeChildrenSiblings({child1, child2});

  EXPECT_LONGS_EQUAL(2, parent->children.size());
  EXPECT(parent->children[1] == child3);
  EXPECT_LONGS_EQUAL(2, parent->children[0]->children.size());
  const KeySet expected{1, 2};
  const KeySet actual(parent->children[0]->orderedFrontalKeys.begin(),
                      parent->children[0]->orderedFrontalKeys.end());
  EXPECT(assert_container_equality(expected, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
