/**
 * @file    testEliminationTree.cpp
 * @brief   Unit tests for Elimination Tree
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include "SymbolicFactorGraph.h"
#include "ClusterTree-inl.h"
#include "EliminationTree-inl.h"

using namespace gtsam;

// explicit instantiation and typedef
template class EliminationTree<SymbolicFactorGraph>;
typedef EliminationTree<SymbolicFactorGraph> SymbolicEliminationTree;

/* ************************************************************************* *
 * graph: x1 - x2 - x3 - x4
 * tree: x1 -> x2 -> x3 -> x4 (arrow is parent pointer)
 ****************************************************************************/
TEST( EliminationTree, constructor )
{
	SymbolicFactorGraph fg;
	fg.push_factor("x1","x2");
	fg.push_factor("x2","x3");
	fg.push_factor("x3","x4");

	SymbolicEliminationTree expected();

	Ordering ordering; ordering += "x2","x1","x3","x4";
	SymbolicEliminationTree actual(fg, ordering);

//	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
