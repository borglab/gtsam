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

#include <vector>
#include <list>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/assign/std/list.hpp>

using boost::assign::operator+=;
using namespace std;

struct TestNode {
  typedef boost::shared_ptr<TestNode> shared_ptr;
  int data;
  vector<shared_ptr> children;
  TestNode(int data) : data(data) {}
};

struct TestForest {
  typedef TestNode::shared_ptr sharedNode;
  vector<sharedNode> roots_;
  const vector<sharedNode>& roots() const { return roots_; }
};

TestForest makeTestForest() {
  //    0     1
  //   / \
  //  2   3
  //      |
  //      4
  TestForest forest;
  forest.roots_.push_back(boost::make_shared<TestNode>(0));
  forest.roots_.push_back(boost::make_shared<TestNode>(1));
  forest.roots_[0]->children.push_back(boost::make_shared<TestNode>(2));
  forest.roots_[0]->children.push_back(boost::make_shared<TestNode>(3));
  forest.roots_[0]->children[1]->children.push_back(boost::make_shared<TestNode>(4));
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
      node->data == 4 ? 0 :
      (throw std::runtime_error("Unexpected node index"), -1);
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
TEST(treeTraversal, DepthFirst)
{
  // Get test forest
  TestForest testForest = makeTestForest();

  // Expected pre-order
  std::list<int> preOrderExpected;
  preOrderExpected += 0, 2, 3, 4, 1;
  std::list<int> postOrderExpected;
  postOrderExpected += 2, 4, 3, 0, 1;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

