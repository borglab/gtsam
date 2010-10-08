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
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY
#include <gtsam/inference/SymbolicFactorGraph.h>

using namespace std;
using namespace gtsam;

typedef boost::shared_ptr<SymbolicFactorGraph> shared;

///* ************************************************************************* */
// SL-FIX TEST( FactorGraph, splitMinimumSpanningTree )
//{
//	SymbolicFactorGraph G;
//	G.push_factor("x1", "x2");
//	G.push_factor("x1", "x3");
//	G.push_factor("x1", "x4");
//	G.push_factor("x2", "x3");
//	G.push_factor("x2", "x4");
//	G.push_factor("x3", "x4");
//
//	SymbolicFactorGraph T, C;
//	boost::tie(T, C) = G.splitMinimumSpanningTree();
//
//	SymbolicFactorGraph expectedT, expectedC;
//	expectedT.push_factor("x1", "x2");
//	expectedT.push_factor("x1", "x3");
//	expectedT.push_factor("x1", "x4");
//	expectedC.push_factor("x2", "x3");
//	expectedC.push_factor("x2", "x4");
//	expectedC.push_factor("x3", "x4");
//	CHECK(assert_equal(expectedT,T));
//	CHECK(assert_equal(expectedC,C));
//}

///* ************************************************************************* */
///**
// *  x1 - x2 - x3 - x4 - x5
// *       |    |  / |
// *       l1   l2   l3
// */
// SL-FIX TEST( FactorGraph, removeSingletons )
//{
//	SymbolicFactorGraph G;
//	G.push_factor("x1", "x2");
//	G.push_factor("x2", "x3");
//	G.push_factor("x3", "x4");
//	G.push_factor("x4", "x5");
//	G.push_factor("x2", "l1");
//	G.push_factor("x3", "l2");
//	G.push_factor("x4", "l2");
//	G.push_factor("x4", "l3");
//
//	SymbolicFactorGraph singletonGraph;
//	set<Symbol> singletons;
//	boost::tie(singletonGraph, singletons) = G.removeSingletons();
//
//	set<Symbol> singletons_excepted; singletons_excepted += "x1", "x2", "x5", "l1", "l3";
//	CHECK(singletons_excepted == singletons);
//
//	SymbolicFactorGraph singletonGraph_excepted;
//	singletonGraph_excepted.push_factor("x2", "l1");
//	singletonGraph_excepted.push_factor("x4", "l3");
//	singletonGraph_excepted.push_factor("x1", "x2");
//	singletonGraph_excepted.push_factor("x4", "x5");
//	singletonGraph_excepted.push_factor("x2", "x3");
//	CHECK(singletonGraph_excepted.equals(singletonGraph));
//}


/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
