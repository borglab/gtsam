/*
 * testGaussianJunctionTree.cpp
 *
 * Created on: Jul 8, 2010
 * @author Kai Ni
 */

#include <iostream>
#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#define GTSAM_MAGIC_KEY

#include "Ordering.h"
#include "GaussianJunctionTree.h"

using namespace std;
using namespace gtsam;

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
TEST( GaussianJunctionTree, eliminate )
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
	GaussianJunctionTree junctionTree(fg, ordering);
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

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
