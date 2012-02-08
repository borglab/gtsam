/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicBayesNet.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

// Magically casts strings like "x3" to a Symbol('x',3) key, see Symbol.h
#define GTSAM_MAGIC_KEY

#include <gtsam/base/Testable.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/SymbolicSequentialSolver.h>
#include <gtsam/nonlinear/Ordering.h>

using namespace std;
using namespace gtsam;
using namespace example;

//Symbol _B_('B', 0), _L_('L', 0);
//IndexConditional::shared_ptr
//	B(new IndexConditional(_B_)),
//	L(new IndexConditional(_L_, _B_));

/* ************************************************************************* */
TEST( SymbolicBayesNet, constructor )
{
  Ordering o; o += "x2","l1","x1";
	// Create manually
	IndexConditional::shared_ptr
		x2(new IndexConditional(o["x2"],o["l1"], o["x1"])),
		l1(new IndexConditional(o["l1"],o["x1"])),
		x1(new IndexConditional(o["x1"]));
	BayesNet<IndexConditional> expected;
	expected.push_back(x2);
	expected.push_back(l1);
	expected.push_back(x1);

	// Create from a factor graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph(o);
	SymbolicFactorGraph fg(factorGraph);

	// eliminate it
  SymbolicBayesNet actual = *SymbolicSequentialSolver(fg).eliminate(
			&EliminateSymbolic);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
