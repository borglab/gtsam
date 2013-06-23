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
#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>
using namespace boost::assign;
#include <boost/make_shared.hpp>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/symbolic/SymbolicEliminationTreeUnordered.h>
#include <gtsam/nonlinear/Symbol.h>

using namespace gtsam;
using namespace gtsam::symbol_shorthand;
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
    node1->children.push_back(leaf0);

    SymbolicEliminationTreeUnordered::sharedNode node2(new SymbolicEliminationTreeUnordered::Node);
    node2->key = 2;
    node2->factors.push_back(fg[3]);
    node2->children.push_back(node1);

    SymbolicEliminationTreeUnordered::sharedNode leaf3(new SymbolicEliminationTreeUnordered::Node);
    leaf3->key = 3;
    leaf3->factors.push_back(fg[4]);

    SymbolicEliminationTreeUnordered::sharedNode root(new SymbolicEliminationTreeUnordered::Node);
    root->key = 4;
    root->children.push_back(leaf3);
    root->children.push_back(node2);

    SymbolicEliminationTreeUnordered tree;
    tree.roots_.push_back(root);
    return tree;
  }
};


/* ************************************************************************* */
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
  OrderingUnordered order;
  order += 0,1,2,3,4;
  SymbolicEliminationTreeUnordered actual(fg, order);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
