/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testForestTraversal.cpp
 * @brief Unit tests for ForestTraversal helpers.
 * @author Frank Dellaert
 * @date May, 2025
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/ForestTraversal.h>

#include <memory>
#include <vector>

using namespace gtsam;

/* ************************************************************************* */
/// Common TreeNode definition (using shared_ptr).
struct TreeNode {
  int value;
  explicit TreeNode(int initialValue) : value(initialValue) {}
};

namespace {
/// Build a small tree for tests.
template <typename NodeT>
std::shared_ptr<NodeT> createSimpleTree() {
  auto n1 = std::make_shared<NodeT>(1);
  auto n2 = std::make_shared<NodeT>(2);
  auto n3 = std::make_shared<NodeT>(3);
  auto n4 = std::make_shared<NodeT>(4);
  auto n5 = std::make_shared<NodeT>(5);
  auto n6 = std::make_shared<NodeT>(6);
  auto n7 = std::make_shared<NodeT>(7);
  n1->children = {n2, n3, n4};
  n2->children = {n5, n6};
  n6->children = {n7};
  return n1;
}

}  // namespace

/* ************************************************************************* */
// Test 1: Bottom-up traversal
/* ************************************************************************* */
/// TreeNode specialization for bottom-up test.
struct TestNode1 : public TreeNode {
  using TreeNode::TreeNode;
  std::vector<std::shared_ptr<TestNode1>> children;
  size_t subtreeCount{0};

  int problemSize() const { return 1; }

  /// Bottom-up payload for tests.
  void bottomUpValue() {
    // Each node stores 1 plus the sum of its children's results.
    size_t sum = 1;
    for (const auto& child : children) {
      sum += child->subtreeCount;
    }
    subtreeCount = sum;
  }
};

/// Forest wrapper for bottom-up traversal.
struct TestForest1 : public ForestTraversal<TestForest1, TestNode1> {
  using Node = TestNode1;
  std::vector<std::shared_ptr<TestNode1>> roots_;

  const std::vector<std::shared_ptr<TestNode1>>& roots() const {
    return roots_;
  }

  /// Count nodes using a bottom-up traversal.
  size_t countNodes() {
    runBottomUp(&TestNode1::bottomUpValue);
    size_t total = 0;
    for (const auto& root : roots_) {
      total += root->subtreeCount;
    }
    return total;
  }
};

TEST(ForestTraversal, BottomUpCounting) {
  std::shared_ptr<TestNode1> root = createSimpleTree<TestNode1>();
  TestForest1 forest;
  forest.roots_.push_back(root);
  size_t rootValue = forest.countNodes();

  const size_t expectedTotalNodes = 7;
  EXPECT_LONGS_EQUAL(expectedTotalNodes, rootValue);
}

/* ************************************************************************* */
// Test 2: Top-down traversal
/* ************************************************************************* */
/// TreeNode specialization for top-down test.
struct TestNode2 : public TreeNode {
  using TreeNode::TreeNode;
  std::vector<std::shared_ptr<TestNode2>> children;
  bool visited{false};
  size_t subtreeCount{0};

  int problemSize() const { return 1; }

  /// Top-down payload for tests.
  void topDownTouch() { visited = true; }

  /// Bottom-up payload for tests.
  void bottomUpVisitedCount() {
    // Each node stores 1 if visited plus the sum of its children.
    size_t sum = visited ? 1 : 0;
    for (const auto& child : children) {
      sum += child->subtreeCount;
    }
    subtreeCount = sum;
  }
};

/// Forest wrapper for top-down traversal.
struct TestForest2 : public ForestTraversal<TestForest2, TestNode2> {
  using Node = TestNode2;
  std::vector<std::shared_ptr<TestNode2>> roots_;

  const std::vector<std::shared_ptr<TestNode2>>& roots() const {
    return roots_;
  }

  /// Touch all nodes using a top-down traversal.
  void touchAll() { runTopDown(&TestNode2::topDownTouch); }

  /// Count visited nodes using a bottom-up traversal.
  size_t countVisited() {
    runBottomUp(&TestNode2::bottomUpVisitedCount);
    size_t total = 0;
    for (const auto& root : roots_) {
      total += root->subtreeCount;
    }
    return total;
  }
};

TEST(ForestTraversal, TopDownVisits) {
  std::shared_ptr<TestNode2> root = createSimpleTree<TestNode2>();
  TestForest2 forest;
  forest.roots_.push_back(root);
  forest.touchAll();

  const size_t expectedTotalNodes = 7;
  EXPECT_LONGS_EQUAL(expectedTotalNodes, forest.countVisited());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
