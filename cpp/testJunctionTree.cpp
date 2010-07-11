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

/* ************************************************************************* */
/**
 * x1 - x2 - x3 - x4
 * x3 x4
 *    x2 x1 : x3
 */
TEST( GaussianFactorGraph, constructor )
{
	typedef GaussianFactorGraph::sharedFactor Factor;
	SharedDiagonal model(Vector_(1, 0.2));
	Factor factor1(new GaussianFactor("x1", Matrix_(1,1,1.), "x2", Matrix_(1,1,1.), Vector_(1,1.),  model));
	Factor factor2(new GaussianFactor("x2", Matrix_(1,1,1.), "x3", Matrix_(1,1,1.), Vector_(1,1.),  model));
	Factor factor3(new GaussianFactor("x3", Matrix_(1,1,1.), "x4", Matrix_(1,1,1.), Vector_(1,1.),  model));

	GaussianFactorGraph fg;
	fg.push_back(factor1);
	fg.push_back(factor2);
	fg.push_back(factor3);

	Ordering ordering; ordering += "x2","x1","x3","x4";
	GaussianJunctionTree<GaussianFactorGraph> junctionTree(fg, ordering);

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

/* ************************************************************************* *
 Bayes tree for smoother with "nested dissection" ordering:
	 C1		 x5 x6 x4
	 C2		  x3 x2 : x4
	 C3		    x1 : x2
	 C4		  x7 : x6
/* ************************************************************************* */
TEST( GaussianFactorGraph, constructor2 )
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

/* ************************************************************************* */
/**
 * x1 - x2 - x3 - x4
 * x3 x4
 *    x2 x1 : x3
 *
 * x2 x1 x3 x4  b
 *  1  1        1
 *  1     1     1
 *        1  1  1
 *           1  1
 *
 *  1  0  0  1
 */
TEST( GaussianFactorGraph, eliminate )
{
	typedef GaussianFactorGraph::sharedFactor Factor;
	SharedDiagonal model(Vector_(1, 0.5));
	Factor factor1(new GaussianFactor("x1", Matrix_(1,1,1.), "x2", Matrix_(1,1,1.), Vector_(1,1.),  model));
	Factor factor2(new GaussianFactor("x2", Matrix_(1,1,1.), "x3", Matrix_(1,1,1.), Vector_(1,1.),  model));
	Factor factor3(new GaussianFactor("x3", Matrix_(1,1,1.), "x4", Matrix_(1,1,1.), Vector_(1,1.),  model));
	Factor factor4(new GaussianFactor("x4", Matrix_(1,1,1.), Vector_(1,1.),  model));

	GaussianFactorGraph fg;
	fg.push_back(factor1);
	fg.push_back(factor2);
	fg.push_back(factor3);
	fg.push_back(factor4);

	Ordering ordering; ordering += "x2","x1","x3","x4";
	GaussianJunctionTree<GaussianFactorGraph> junctionTree(fg, ordering);
		BayesTree<GaussianConditional> bayesTree = junctionTree.eliminate<GaussianConditional>();

	typedef BayesTree<GaussianConditional>::sharedConditional sharedConditional;
	Matrix two = Matrix_(1,1,2.);
	Matrix one = Matrix_(1,1,1.);
	BayesTree<GaussianConditional> bayesTree_expected;
	bayesTree_expected.insert(sharedConditional(new GaussianConditional("x4", Vector_(1,2.), two, Vector_(1,1.))), ordering);
	bayesTree_expected.insert(sharedConditional(new GaussianConditional("x3", Vector_(1,2.), two, "x4", two, Vector_(1,1.))), ordering);
	bayesTree_expected.insert(sharedConditional(new GaussianConditional("x1", Vector_(1,0.), one*(-1), "x3", one, Vector_(1,1.))), ordering);
	bayesTree_expected.insert(sharedConditional(new GaussianConditional("x2", Vector_(1,2.), two, "x1", one, "x3", one, Vector_(1,1.))), ordering);
	CHECK(assert_equal(bayesTree_expected, bayesTree));
}

/* ************************************************************************* *
TEST( GaussianFactorGraph, optimizeMultiFrontal )
{
	// create a graph
	GaussianFactorGraph fg = createSmoother(7);

	// create an ordering
	Ordering ordering; ordering += "x1","x3","x5","x7","x2","x6","x4";

	// optimize the graph
	GaussianJunctionTree<GaussianFactorGraph> junctionTree(fg, ordering);
	VectorConfig actual = junctionTree.optimize();

	// verify
//	VectorConfig expected = createCorrectDelta();
//
//  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
