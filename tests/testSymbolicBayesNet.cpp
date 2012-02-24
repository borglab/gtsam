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

#include <gtsam/base/Testable.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/SymbolicSequentialSolver.h>
#include <gtsam/nonlinear/Ordering.h>

using namespace std;
using namespace gtsam;
using namespace example;

Key kx(size_t i) { return Symbol('x',i); }
Key kl(size_t i) { return Symbol('l',i); }

//Symbol _B_('B', 0), _L_('L', 0);
//IndexConditional::shared_ptr
//	B(new IndexConditional(_B_)),
//	L(new IndexConditional(_L_, _B_));

/* ************************************************************************* */
TEST( SymbolicBayesNet, constructor )
{
  Ordering o; o += kx(2),kl(1),kx(1);
	// Create manually
	IndexConditional::shared_ptr
		x2(new IndexConditional(o[kx(2)],o[kl(1)], o[kx(1)])),
		l1(new IndexConditional(o[kl(1)],o[kx(1)])),
		x1(new IndexConditional(o[kx(1)]));
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
