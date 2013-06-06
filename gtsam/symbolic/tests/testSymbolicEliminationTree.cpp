/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicEliminationTree.cpp
 * @brief   
 * @author  Richard Roberts
 * @date Oct 14, 2010
 */

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/vector.hpp>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/symbolic/SymbolicEliminationTreeUnordered.h>

using namespace gtsam;
using namespace std;

class EliminationTreeUnorderedTester {
public:
  // build hardcoded tree
  static SymbolicEliminationTreeUnordered buildHardcodedTree(const SymbolicFactorGraphUnordered& fg) {

    SymbolicEliminationTreeUnordered::sharedNode leaf0(new SymbolicEliminationTreeUnordered::Node);
    leaf0->key = 0;
    leaf0->factors.push_back(fg[0]);
    leaf0->factors.push_back(fg[1]);

    SymbolicEliminationTreeUnordered::sharedNode node1(new SymbolicEliminationTreeUnordered::Node);
    node1->key = 1;
    node1->factors.push_back(fg[2]);
    node1->subTrees.push_back(leaf0);

    SymbolicEliminationTreeUnordered::sharedNode node2(new SymbolicEliminationTreeUnordered::Node);
    node2->key = 2;
    node2->factors.push_back(fg[3]);
    node2->subTrees.push_back(node1);

    SymbolicEliminationTreeUnordered::sharedNode leaf3(new SymbolicEliminationTreeUnordered::Node);
    leaf3->key = 3;
    leaf3->factors.push_back(fg[4]);

    SymbolicEliminationTreeUnordered::sharedNode root(new SymbolicEliminationTreeUnordered::Node);
    root->key = 4;
    root->subTrees.push_back(leaf3);
    root->subTrees.push_back(node2);

    SymbolicEliminationTreeUnordered tree;
    tree.roots_.push_back(root);
    return tree;
  }
};

TEST(EliminationTree, Create)
{
  // create example factor graph
  SymbolicFactorGraphUnordered fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  SymbolicEliminationTreeUnordered expected = EliminationTreeUnorderedTester::buildHardcodedTree(fg);

  // Build from factor graph
  vector<size_t> order;
  order += 0,1,2,3,4;
  SymbolicEliminationTreeUnordered actual(fg, order);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
// Test to drive elimination tree development
// graph: f(0,1) f(0,2) f(1,4) f(2,4) f(3,4)
/* ************************************************************************* */

TEST_UNSAFE(EliminationTree, eliminate )
{
  // create expected Chordal bayes Net
  SymbolicBayesNetUnordered expected;
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(4));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(3,4));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(2,4));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(1,2,4));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(0,1,2));

  // Create factor graph
  SymbolicFactorGraphUnordered fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  // eliminate
  vector<size_t> order;
  order += 0,1,2,3,4;
  SymbolicBayesNetUnordered actual = *SymbolicEliminationTreeUnordered(fg,order).eliminate(EliminateSymbolicUnordered).first;

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
//TEST(EliminationTree, disconnected_graph) {
//  SymbolicFactorGraph fg;
//  fg.push_factor(0, 1);
//  fg.push_factor(0, 2);
//  fg.push_factor(1, 2);
//  fg.push_factor(3, 4);
//
//  CHECK_EXCEPTION(SymbolicEliminationTree::Create(fg), DisconnectedGraphException);
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
