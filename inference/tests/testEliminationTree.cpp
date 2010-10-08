/**
 * @file    testEliminationTree.cpp
 * @brief   Unit tests for Elimination Tree
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

// for operator +=
#include <boost/assign/std/list.hpp>
#include <boost/assign/std/map.hpp>
#include <boost/make_shared.hpp>
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/ClusterTree-inl.h>
#include <gtsam/inference/EliminationTree-inl.h>

using namespace std;
using namespace gtsam;
using namespace boost;

// explicit instantiation and typedef
template class EliminationTree<SymbolicFactorGraph>;
typedef EliminationTree<SymbolicFactorGraph> SymbolicEliminationTree;

/* ************************************************************************* *
 * graph: f(1,2) f(1,3) f(2,5) f(3,5) f(4,5)
 * tree: x1 -> x2 -> x3 -> x5 <- x4 (arrow is parent pointer)
 ****************************************************************************/
//TEST( EliminationTree, constructor )
//{
//	Ordering ordering; ordering += "x1","x2","x3","x4","x5";
//
//	/** build expected tree using constructor variant 1 */
//	SymbolicEliminationTree::OrderedGraphs graphs;
//	SymbolicFactorGraph c1,c2,c3,c4,c5;
//	c1.push_factor("x1","x2"); c1.push_factor("x1","x3"); graphs += make_pair("x1",c1);
//	c2.push_factor("x2","x5"); graphs += make_pair("x2",c2);
//	c3.push_factor("x3","x5"); graphs += make_pair("x3",c3);
//	c4.push_factor("x4","x5"); graphs += make_pair("x4",c4);
//	graphs += make_pair("x5",c5);
//	SymbolicEliminationTree expected(graphs);
//
//	/** build actual tree from factor graph (variant 2) */
//	SymbolicFactorGraph fg;
//	fg.push_factor("x1","x2");
//	fg.push_factor("x1","x3");
//	fg.push_factor("x2","x5");
//	fg.push_factor("x3","x5");
//	fg.push_factor("x4","x5");
//	SymbolicEliminationTree actual(fg, ordering);
////	GTSAM_PRINT(actual);
//
//	CHECK(assert_equal<SymbolicEliminationTree>(expected, actual));
//}

/* ************************************************************************* *
 * graph: f(1,2) f(1,3) f(2,5) f(3,5) f(4,5)
 * tree: x1 -> x2 -> x3 -> x5 <- x4 (arrow is parent pointer)
 ****************************************************************************/
TEST( EliminationTree, constructor )
{
  varid_t x1=1, x2=2, x3=3, x4=4, x5=5;
  SymbolicFactorGraph fc1,fc2,fc3,fc4,fc5;

  fc1.push_factor(x1,x2); fc1.push_factor(x1,x3);
  list<varid_t> c1sep; c1sep += x2,x3;
  SymbolicEliminationTree::sharedNode c1(new SymbolicEliminationTree::Node(fc1, x1, c1sep.begin(), c1sep.end()));

  fc2.push_factor(x2,x5);
  list<varid_t> c2sep; c2sep += x3,x5;
  SymbolicEliminationTree::sharedNode c2(new SymbolicEliminationTree::Node(fc2, x2, c2sep.begin(), c2sep.end()));

  fc3.push_factor(x3,x5);
  list<varid_t> c3sep; c3sep += x5;
  SymbolicEliminationTree::sharedNode c3(new SymbolicEliminationTree::Node(fc3, x3, c3sep.begin(), c3sep.end()));

  fc4.push_factor(x4,x5);
  list<varid_t> c4sep; c4sep += x5;
  SymbolicEliminationTree::sharedNode c4(new SymbolicEliminationTree::Node(fc4, x4, c4sep.begin(), c4sep.end()));

  list<varid_t> c5sep;
  SymbolicEliminationTree::sharedNode c5(new SymbolicEliminationTree::Node(fc5, x5, c5sep.begin(), c5sep.end()));

  /** build expected tree using test accessor */
  SymbolicEliminationTree expected;
  _EliminationTreeTester<SymbolicFactorGraph> expected_(expected);
  expected_.nodes().resize(6);
  expected_.root() = c5; expected_.nodes()[x5]=c5;
  c5->addChild(c4); c4->parent()=c5; expected_.nodes()[x4]=c4;
  c5->addChild(c3); c3->parent()=c5; expected_.nodes()[x3]=c3;
  c3->addChild(c2); c2->parent()=c3; expected_.nodes()[x2]=c2;
  c2->addChild(c1); c1->parent()=c2; expected_.nodes()[x1]=c1;

  /** build actual tree from factor graph (variant 2) */
  SymbolicFactorGraph fg;
  fg.push_factor(x1,x2);
  fg.push_factor(x1,x3);
  fg.push_factor(x2,x5);
  fg.push_factor(x3,x5);
  fg.push_factor(x4,x5);
  SymbolicEliminationTree actual(fg);
//  GTSAM_PRINT(actual);

  CHECK(assert_equal<SymbolicEliminationTree>(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
