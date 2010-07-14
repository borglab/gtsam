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
 * graph: x1 - x2 - x3 - x4
 * tree: x1 -> x2 -> x3 <- x4 (arrow is parent pointer)
 ****************************************************************************/
TEST( EliminationTree, constructor )
{
	Ordering ordering; ordering += "x1","x2","x4","x3";

	/** build expected tree using constructor variant 1 */
	SymbolicEliminationTree::OrderedGraphs orderedGraphs;
	SymbolicFactorGraph c1; c1.push_factor("x1","x2"); orderedGraphs += make_pair("x1",c1);
	SymbolicFactorGraph c2; c2.push_factor("x2","x3"); orderedGraphs += make_pair("x2",c2);
	SymbolicFactorGraph c4; c4.push_factor("x4","x3"); orderedGraphs += make_pair("x4",c4);
	SymbolicFactorGraph c3;                            orderedGraphs += make_pair("x3",c3);
	SymbolicEliminationTree expected(orderedGraphs);

	/** build actual tree from factor graph (variant 2) */
	SymbolicFactorGraph fg;
	fg.push_factor("x1","x2");
	fg.push_factor("x2","x3");
	fg.push_factor("x3","x4");
	SymbolicEliminationTree actual(fg, ordering);

	CHECK(assert_equal<SymbolicEliminationTree>(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
