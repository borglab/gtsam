/*
 * testJunctionTree.cpp
 *
 *   Created on: Jul 8, 2010
 *       Author: nikai
 *  Description:
 */

#include <iostream>
#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#define GTSAM_MAGIC_KEY

#include "smallExample.h"
#include "JunctionTree-inl.h"

using namespace std;
using namespace gtsam;
using namespace example;

/* ************************************************************************* *
 Bayes tree for smoother with "nested dissection" ordering:
	 C1		 x5 x6 x4
	 C2		  x3 x2 : x4
	 C3		    x1 : x2
	 C4		  x7 : x6
/* ************************************************************************* */
TEST( GaussianFactorGraph, constructor )
{
	// create a graph
	GaussianFactorGraph fg = createSmoother(7);

	// create an ordering
	Ordering ordering; ordering += "x1","x3","x5","x7","x2","x6","x4";

	GaussianJunctionTree<GaussianFactorGraph> junctionTree(fg, ordering);
	Ordering frontal1; frontal1 += "x5", "x6", "x4";
	Ordering frontal2; frontal2 += "x3", "x2";
	Ordering frontal3; frontal3 += "x1";
	Ordering frontal4; frontal4 += "x7";
	Unordered sep1;
	Unordered sep2; sep2 += "x4";
	Unordered sep3; sep3 += "x2";
	Unordered sep4; sep4 += "x6";
	CHECK(assert_equal(frontal1, junctionTree.root()->frontal()));
	CHECK(assert_equal(sep1,     junctionTree.root()->separator()));
	LONGS_EQUAL(5,               junctionTree.root()->size());
	CHECK(assert_equal(frontal2, junctionTree.root()->children()[0]->frontal()));
	CHECK(assert_equal(sep2,     junctionTree.root()->children()[0]->separator()));
	LONGS_EQUAL(4,               junctionTree.root()->children()[0]->size());
	CHECK(assert_equal(frontal3, junctionTree.root()->children()[0]->children()[0]->frontal()));
	CHECK(assert_equal(sep3,     junctionTree.root()->children()[0]->children()[0]->separator()));
	LONGS_EQUAL(2,               junctionTree.root()->children()[0]->children()[0]->size());
	CHECK(assert_equal(frontal4, junctionTree.root()->children()[1]->frontal()));
	CHECK(assert_equal(sep4,     junctionTree.root()->children()[1]->separator()));
	LONGS_EQUAL(2,               junctionTree.root()->children()[1]->size());
}


/* ************************************************************************* *
TEST( GaussianFactorGraph, optimizeMultiFrontal )
{
	// create a graph
	GaussianFactorGraph fg = createSmoother(7);

	// create an ordering
	Ordering ordering; ordering += "x1","x3","x5","x7","x2","x6","x4";

	// optimize the graph
	LinearJunctionTree<GaussianConditional, GaussianFactorGraph> junctionTree(fg, ordering);
	VectorConfig actual = junctionTree.optimize();

	// verify
	VectorConfig expected = createCorrectDelta();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
