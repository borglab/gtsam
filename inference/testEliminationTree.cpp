/**
 * @file    testEliminationTree.cpp
 * @brief   Unit tests for Elimination Tree
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

// for operator +=
#include <boost/assign/std/list.hpp>
#include <boost/assign/std/map.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include "SymbolicFactorGraph.h"
#include "ClusterTree-inl.h"
#include "EliminationTree-inl.h"

using namespace std;
using namespace gtsam;

// explicit instantiation and typedef
template class EliminationTree<SymbolicFactorGraph>;
typedef EliminationTree<SymbolicFactorGraph> SymbolicEliminationTree;

/* ************************************************************************* *
 * graph: f(1,2) f(1,3) f(2,5) f(3,5) f(4,5)
 * tree: x1 -> x2 -> x3 -> x5 <- x4 (arrow is parent pointer)
 ****************************************************************************/
TEST( EliminationTree, constructor )
{
	Ordering ordering; ordering += "x1","x2","x3","x4","x5";

	/** build expected tree using constructor variant 1 */
	SymbolicEliminationTree::OrderedGraphs graphs;
	SymbolicFactorGraph c1,c2,c3,c4,c5;
	c1.push_factor("x1","x2"); c1.push_factor("x1","x3"); graphs += make_pair("x1",c1);
	c2.push_factor("x2","x5"); graphs += make_pair("x2",c2);
	c3.push_factor("x3","x5"); graphs += make_pair("x3",c3);
	c4.push_factor("x4","x5"); graphs += make_pair("x4",c4);
	graphs += make_pair("x5",c5);
	SymbolicEliminationTree expected(graphs);

	/** build actual tree from factor graph (variant 2) */
	SymbolicFactorGraph fg;
	fg.push_factor("x1","x2");
	fg.push_factor("x1","x3");
	fg.push_factor("x2","x5");
	fg.push_factor("x3","x5");
	fg.push_factor("x4","x5");
	SymbolicEliminationTree actual(fg, ordering);
//	GTSAM_PRINT(actual);

	CHECK(assert_equal<SymbolicEliminationTree>(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
