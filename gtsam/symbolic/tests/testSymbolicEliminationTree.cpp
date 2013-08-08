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

#include <vector>
#include <boost/make_shared.hpp>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/nonlinear/Symbol.h>

#include "symbolicExampleGraphs.h"

using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using namespace std;

class EliminationTreeTester {
public:
  // build hardcoded tree
  static SymbolicEliminationTree buildHardcodedTree(const SymbolicFactorGraph& fg) {

    SymbolicEliminationTree::sharedNode leaf0(new SymbolicEliminationTree::Node);
    leaf0->key = 0;
    leaf0->factors.push_back(fg[0]);
    leaf0->factors.push_back(fg[1]);

    SymbolicEliminationTree::sharedNode node1(new SymbolicEliminationTree::Node);
    node1->key = 1;
    node1->factors.push_back(fg[2]);
    node1->children.push_back(leaf0);

    SymbolicEliminationTree::sharedNode node2(new SymbolicEliminationTree::Node);
    node2->key = 2;
    node2->factors.push_back(fg[3]);
    node2->children.push_back(node1);

    SymbolicEliminationTree::sharedNode leaf3(new SymbolicEliminationTree::Node);
    leaf3->key = 3;
    leaf3->factors.push_back(fg[4]);

    SymbolicEliminationTree::sharedNode root(new SymbolicEliminationTree::Node);
    root->key = 4;
    root->children.push_back(leaf3);
    root->children.push_back(node2);

    SymbolicEliminationTree tree;
    tree.roots_.push_back(root);
    return tree;
  }
};


/* ************************************************************************* */
TEST(EliminationTree, Create)
{
  SymbolicEliminationTree expected =
    EliminationTreeTester::buildHardcodedTree(simpleTestGraph1);

  // Build from factor graph
  Ordering order;
  order += 0,1,2,3,4;
  SymbolicEliminationTree actual(simpleTestGraph1, order);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
