/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicSequentialSolver.cpp
 * @brief   Unit tests for a symbolic IndexFactor Graph
 * @author  Frank Dellaert
 * @date    Sept 16, 2012
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/SymbolicSequentialSolver.h>

using namespace std;
using namespace gtsam;

static const Index vx2 = 0;
static const Index vx1 = 1;
static const Index vl1 = 2;

/* ************************************************************************* */
TEST( SymbolicSequentialSolver, SymbolicSequentialSolver )
{
	// create factor graph
	SymbolicFactorGraph g;
	g.push_factor(vx2, vx1, vl1);
	g.push_factor(vx1, vl1);
	g.push_factor(vx1);
	// test solver is Testable
	SymbolicSequentialSolver solver(g);
//	GTSAM_PRINT(solver);
	EXPECT(assert_equal(solver,solver));
}

/* ************************************************************************* */
TEST( SymbolicSequentialSolver, eliminate )
{
	// create expected Chordal bayes Net
  SymbolicBayesNet expected;
  expected.push_front(boost::make_shared<IndexConditional>(4));
  expected.push_front(boost::make_shared<IndexConditional>(3,4));
  expected.push_front(boost::make_shared<IndexConditional>(2,4));
  expected.push_front(boost::make_shared<IndexConditional>(1,2,4));
  expected.push_front(boost::make_shared<IndexConditional>(0,1,2));

	// Create factor graph
	SymbolicFactorGraph fg;
	fg.push_factor(0, 1);
	fg.push_factor(0, 2);
	fg.push_factor(1, 4);
	fg.push_factor(2, 4);
	fg.push_factor(3, 4);

	// eliminate
  SymbolicSequentialSolver solver(fg);
	SymbolicBayesNet::shared_ptr actual = solver.eliminate();

	CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
