/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testTreeTraversal
 * @brief Unit tests for tree traversal, cloning, and printing
 * @author Richard Roberts
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/treeTraversal-inst.h>

#include <vector>
#include <list>
#include <memory>
#include <boost/make_shared.hpp>

using namespace gtsam;

struct TestNode {
  typedef std::shared_ptr<TestNode> shared_ptr;
  int data;
  std::vector<shared_ptr> children;
  TestNode() : data(-1) {}
  TestNode(int data) : data(data) {}
};

struct TestForest {
  typedef TestNode Node;
  typedef Node::shared_ptr sharedNode;
  FastVector<sharedNode> roots_;
  const FastVector<sharedNode>& roots() const { return roots_; }
};

TestForest makeTestForest() {
  //     0     1
  //   / |
  //  2  3
  //     |
  //     4
  TestForest forest;
  forest.roots_.push_back(std::make_shared<TestNode>(0));
  forest.roots_.push_back(std::make_shared<TestNode>(1));
  forest.roots_[0]->children.push_back(std::make_shared<TestNode>(2));
  forest.roots_[0]->children.push_back(std::make_shared<TestNode>(3));
  forest.roots_[0]->children[1]->children.push_back(std::make_shared<TestNode>(4));
  return forest;
}

/* ************************************************************************* */
struct PreOrderVisitor {
  // This visitor stores the nodes visited so the visit order can be checked later.  It also returns
  // the current node index as the data and checks that the parent index properly matches the index
  // referenced in the node.
  std::list<int> visited;
  bool parentsMatched;
  PreOrderVisitor() : parentsMatched(true) {}
  int operator()(const TestNode::shared_ptr& node, int parentData) {
    visited.push_back(node->data);
    // Check parent index
    const int expectedParentIndex =
      node->data == 0 ? -1 :
      node->data == 1 ? -1 :
      node->data == 2 ? 0 :
      node->data == 3 ? 0 :
      node->data == 4 ? 3 :
      node->data == 10 ? 0 :
      (parentsMatched = false, -1);
    if(expectedParentIndex != parentData)
      parentsMatched = false;
    return node->data;
  }
};

/* ************************************************************************* */
struct PostOrderVisitor {
  // This visitor stores the nodes visited so the visit order can be checked later.
  std::list<int> visited;
  void operator()(const TestNode::shared_ptr& node, int myData) {
    visited.push_back(node->data);
  }
};

/* ************************************************************************* */
std::list<int> getPreorder(const TestForest& forest) {
  std::list<int> result;
  PreOrderVisitor preVisitor;
  int rootData = -1;
  treeTraversal::DepthFirstForest(forest, rootData, preVisitor);
  result = preVisitor.visited;
  return result;
}

/* ************************************************************************* */
TEST(treeTraversal, DepthFirst)
{
  // Get test forest
  TestForest testForest = makeTestForest();

  // Expected visit order
  const std::list<int> preOrderExpected{0, 2, 3, 4, 1};
  const std::list<int> postOrderExpected{2, 4, 3, 0, 1};

  // Actual visit order
  PreOrderVisitor preVisitor;
  PostOrderVisitor postVisitor;
  int rootData = -1;
  treeTraversal::DepthFirstForest(testForest, rootData, preVisitor, postVisitor);

  EXPECT(preVisitor.parentsMatched);
  EXPECT(assert_container_equality(preOrderExpected, preVisitor.visited));
  EXPECT(assert_container_equality(postOrderExpected, postVisitor.visited));
}

/* ************************************************************************* */
TEST(treeTraversal, CloneForest)
{
  // Get test forest
  TestForest testForest1 = makeTestForest();
  TestForest testForest2;
  testForest2.roots_ = treeTraversal::CloneForest(testForest1);

  // Check that the original and clone both are expected
  const std::list<int> preOrder1Expected{0, 2, 3, 4, 1};
  std::list<int> preOrder1Actual = getPreorder(testForest1);
  std::list<int> preOrder2Actual = getPreorder(testForest2);
  EXPECT(assert_container_equality(preOrder1Expected, preOrder1Actual));
  EXPECT(assert_container_equality(preOrder1Expected, preOrder2Actual));

  // Modify clone - should not modify original
  testForest2.roots_[0]->children[1]->data = 10;
  const std::list<int> preOrderModifiedExpected{0, 2, 10, 4, 1};

  // Check that original is the same and only the clone is modified
  std::list<int> preOrder1ModActual = getPreorder(testForest1);
  std::list<int> preOrder2ModActual = getPreorder(testForest2);
  EXPECT(assert_container_equality(preOrder1Expected, preOrder1ModActual));
  EXPECT(assert_container_equality(preOrderModifiedExpected, preOrder2ModActual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

