/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicBayesNetB.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <tests/smallExample.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/SymbolicSequentialSolver.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Symbol.h>

using namespace std;
using namespace gtsam;
using namespace example;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( SymbolicBayesNet, constructor )
{
  Ordering o; o += X(2),L(1),X(1);
	// Create manually
	IndexConditional::shared_ptr
		x2(new IndexConditional(o[X(2)],o[L(1)], o[X(1)])),
		l1(new IndexConditional(o[L(1)],o[X(1)])),
		x1(new IndexConditional(o[X(1)]));
	BayesNet<IndexConditional> expected;
	expected.push_back(x2);
	expected.push_back(l1);
	expected.push_back(x1);

	// Create from a factor graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph(o);
	SymbolicFactorGraph fg(factorGraph);

	// eliminate it
  SymbolicBayesNet actual = *SymbolicSequentialSolver(fg).eliminate();

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( SymbolicBayesNet, FromGaussian) {
  SymbolicBayesNet expected;
  expected.push_back(IndexConditional::shared_ptr(new IndexConditional(0, 1)));
  expected.push_back(IndexConditional::shared_ptr(new IndexConditional(1)));

  GaussianBayesNet gbn = createSmallGaussianBayesNet();
  SymbolicBayesNet actual(gbn);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
