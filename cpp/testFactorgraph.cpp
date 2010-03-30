/**
 *  @file   testFactorgraph.cpp
 *  @brief  Unit tests for Factor Graphs
 *  @author Christian Potthast
 **/

/*STL/C++*/
#include <list>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY
#include "SymbolicFactorGraph.h"

using namespace std;
using namespace gtsam;

typedef boost::shared_ptr<SymbolicFactorGraph> shared;

/* ************************************************************************* */
TEST( FactorGraph, splitMinimumSpanningTree )
{
	SymbolicFactorGraph G;
	G.push_factor("x1", "x2");
	G.push_factor("x1", "x3");
	G.push_factor("x1", "x4");
	G.push_factor("x2", "x3");
	G.push_factor("x2", "x4");
	G.push_factor("x3", "x4");

	SymbolicFactorGraph T, C;
	boost::tie(T, C) = G.splitMinimumSpanningTree();

	SymbolicFactorGraph expectedT, expectedC;
	expectedT.push_factor("x1", "x2");
	expectedT.push_factor("x1", "x3");
	expectedT.push_factor("x1", "x4");
	expectedC.push_factor("x2", "x3");
	expectedC.push_factor("x2", "x4");
	expectedC.push_factor("x3", "x4");
	CHECK(assert_equal(expectedT,T));
	CHECK(assert_equal(expectedC,C));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
