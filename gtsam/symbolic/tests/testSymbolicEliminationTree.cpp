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
namespace {

  /* ************************************************************************* */
  // Keys for ASIA example from the tutorial with A and D evidence
  const Key _X_=X(0), _T_=T(0), _S_=S(0), _E_=E(0), _L_=L(0), _B_=B(0);

  // Factor graph for Asia example
  const SymbolicFactorGraphUnordered asiaGraph = list_of
    (boost::make_shared<SymbolicFactorUnordered>(_T_))
    (boost::make_shared<SymbolicFactorUnordered>(_S_))
    (boost::make_shared<SymbolicFactorUnordered>(_T_, _E_, _L_))
    (boost::make_shared<SymbolicFactorUnordered>(_L_, _S_))
    (boost::make_shared<SymbolicFactorUnordered>(_S_, _B_))
    (boost::make_shared<SymbolicFactorUnordered>(_E_, _B_))
    (boost::make_shared<SymbolicFactorUnordered>(_E_, _X_));
  
  /* ************************************************************************* */
  const OrderingUnordered asiaOrdering = list_of(_X_)(_T_)(_S_)(_E_)(_L_)(_B_);

}

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
// Test to drive elimination tree development
// graph: f(0,1) f(0,2) f(1,4) f(2,4) f(3,4)
/* ************************************************************************* */

TEST_UNSAFE(EliminationTree, eliminate )
{
  // create expected Chordal bayes Net
  SymbolicBayesNetUnordered expected;
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(0,1,2));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(1,2,4));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(2,4));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(3,4));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(4));

  // Create factor graph
  SymbolicFactorGraphUnordered fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  // eliminate
  OrderingUnordered order;
  order += 0,1,2,3,4;
  SymbolicBayesNetUnordered actual = *SymbolicEliminationTreeUnordered(fg,order).eliminate(EliminateSymbolicUnordered).first;

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(EliminationTree, eliminateAsiaExample)
{
  SymbolicBayesNetUnordered expected = list_of
    (boost::make_shared<SymbolicConditionalUnordered>(_T_, _E_, _L_))
    (boost::make_shared<SymbolicConditionalUnordered>(_X_, _E_))
    (boost::make_shared<SymbolicConditionalUnordered>(_E_, _B_, _L_))
    (boost::make_shared<SymbolicConditionalUnordered>(_S_, _B_, _L_))
    (boost::make_shared<SymbolicConditionalUnordered>(_L_, _B_))
    (boost::make_shared<SymbolicConditionalUnordered>(_B_));

  SymbolicBayesNetUnordered actual = *createAsiaGraph().eliminateSequential(
    EliminateSymbolicUnordered, asiaOrdering);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(EliminationTree, disconnected_graph) {
  SymbolicFactorGraphUnordered fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 2);
  fg.push_factor(3, 4);

  // create expected Chordal bayes Net
  SymbolicBayesNetUnordered expected;
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(0,1,2));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(1,2));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(2));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(3,4));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(4));

  OrderingUnordered order;
  order += 0,1,2,3,4;
  SymbolicBayesNetUnordered actual = *SymbolicEliminationTreeUnordered(fg, order).eliminate(EliminateSymbolicUnordered).first;
  
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
