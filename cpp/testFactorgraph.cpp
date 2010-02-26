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
// This function is not done yet
// It needs a test to check whether a factor is in the same component
// This can be done with a disjoint-set data structure (Union&Find) to keep
// track of which vertices are in which components.
std::pair<shared, shared> splitMinimumSpanningTree(SymbolicFactorGraph& G) {
	//	create an empty factor graph T (tree) and factor graph C (constraints)
	shared T(new SymbolicFactorGraph);
	shared C(new SymbolicFactorGraph);

	//	while G is nonempty and T is not yet spanning
	size_t m = G.nrFactors();
	for (size_t i=0;i<m;i++) {
		//	remove an factor with minimum weight from G
		const SymbolicFactorGraph::sharedFactor& f = G[i];
		G.remove(i);
		//	if that factor connects two different trees, then add it to T
		if (true) // TODO: test whether f is in the same component
			T->push_back(f);
		else //	otherwise add that factor to C
			C->push_back(f);
}
	return std::pair<shared, shared>(T,C);
}

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

	shared T, C;
	boost::tie(T, C) = splitMinimumSpanningTree(G);

	SymbolicFactorGraph expectedT, expectedC;
	expectedT.push_factor("x1", "x2");
	expectedT.push_factor("x1", "x3");
	expectedT.push_factor("x1", "x4");
	expectedT.push_factor("x2", "x3");
	expectedT.push_factor("x2", "x4");
	expectedT.push_factor("x3", "x4");
	CHECK(assert_equal(expectedT,*T));
	CHECK(assert_equal(expectedC,*C));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
