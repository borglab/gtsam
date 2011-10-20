/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/** 
 * @file    testNonlinearFactorGraph.cpp
 * @brief   Unit tests for Non-Linear Factor Graph
 * @brief   testNonlinearFactorGraph
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

/*STL/C++*/
#include <iostream>
using namespace std;

#include <boost/assign/std/list.hpp>
#include <boost/assign/std/set.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

// Magically casts strings like "x3" to a Symbol('x',3) key, see Key.h
#define GTSAM_MAGIC_KEY

#include <gtsam/base/Testable.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>

using namespace gtsam;
using namespace example;

/* ************************************************************************* */
TEST( Graph, equals )
{
	Graph fg = createNonlinearFactorGraph();
	Graph fg2 = createNonlinearFactorGraph();
	CHECK( fg.equals(fg2) );
}

/* ************************************************************************* */
TEST( Graph, error )
{
	Graph fg = createNonlinearFactorGraph();
	Values c1 = createValues();
	double actual1 = fg.error(c1);
	DOUBLES_EQUAL( 0.0, actual1, 1e-9 );

	Values c2 = createNoisyValues();
	double actual2 = fg.error(c2);
	DOUBLES_EQUAL( 5.625, actual2, 1e-9 );
}

/* ************************************************************************* */
TEST( Graph, keys )
{
	Graph fg = createNonlinearFactorGraph();
	set<Symbol> actual = fg.keys();
	LONGS_EQUAL(3, actual.size());
	set<Symbol>::const_iterator it = actual.begin();
	CHECK(assert_equal(Symbol('l', 1), *(it++)));
	CHECK(assert_equal(Symbol('x', 1), *(it++)));
	CHECK(assert_equal(Symbol('x', 2), *(it++)));
}

/* ************************************************************************* */
TEST( Graph, GET_ORDERING)
{
  Ordering expected; expected += "x1","l1","x2";
  Graph nlfg = createNonlinearFactorGraph();
  Ordering actual = *nlfg.orderingCOLAMD(createNoisyValues());
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Graph, probPrime )
{
	Graph fg = createNonlinearFactorGraph();
	Values cfg = createValues();

	// evaluate the probability of the factor graph
	double actual = fg.probPrime(cfg);
	double expected = 1.0;
	DOUBLES_EQUAL(expected,actual,0);
}

/* ************************************************************************* */
TEST( Graph, linearize )
{
	Graph fg = createNonlinearFactorGraph();
	Values initial = createNoisyValues();
	boost::shared_ptr<FactorGraph<GaussianFactor> > linearized = fg.linearize(initial, *initial.orderingArbitrary());
	FactorGraph<GaussianFactor> expected = createGaussianFactorGraph(*initial.orderingArbitrary());
	CHECK(assert_equal(expected,*linearized)); // Needs correct linearizations
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
