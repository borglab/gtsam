/**
 * @file    testJunctionTree.cpp
 * @brief   Unit tests for Junction Tree
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include "Ordering.h"
#include "SymbolicFactorGraph.h"
#include "JunctionTree.h"
#include "ClusterTree-inl.h"
#include "JunctionTree-inl.h"

using namespace gtsam;

// explicit instantiation and typedef
template class JunctionTree<SymbolicFactorGraph>;
typedef JunctionTree<SymbolicFactorGraph> SymbolicJunctionTree;

/* ************************************************************************* *
 * x1 - x2 - x3 - x4
 * x3 x4
 *    x2 x1 : x3
 ****************************************************************************/
TEST( JunctionTree, constructor )
{
	SymbolicFactorGraph fg;
	fg.push_factor("x1","x2");
	fg.push_factor("x2","x3");
	fg.push_factor("x3","x4");

	Ordering ordering; ordering += "x2","x1","x3","x4";
	SymbolicJunctionTree junctionTree(fg, ordering);

	Ordering frontal1; frontal1 += "x3", "x4";
	Ordering frontal2; frontal2 += "x2", "x1";
	Unordered sep1;
	Unordered sep2; sep2 += "x3";
	CHECK(assert_equal(frontal1, junctionTree.root()->frontal()));
	CHECK(assert_equal(sep1,     junctionTree.root()->separator()));
	LONGS_EQUAL(1,               junctionTree.root()->size());
	CHECK(assert_equal(frontal2, junctionTree.root()->children()[0]->frontal()));
	CHECK(assert_equal(sep2,     junctionTree.root()->children()[0]->separator()));
	LONGS_EQUAL(2,               junctionTree.root()->children()[0]->size());
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
