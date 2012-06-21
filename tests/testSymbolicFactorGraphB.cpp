/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicFactorGraphB.cpp
 * @brief   Unit tests for a symbolic Factor Graph
 * @author  Frank Dellaert
 */

#include <tests/smallExample.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/SymbolicSequentialSolver.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( SymbolicFactorGraph, symbolicFactorGraph )
{
  Ordering o; o += X(1),L(1),X(2);
	// construct expected symbolic graph
	SymbolicFactorGraph expected;
	expected.push_factor(o[X(1)]);
	expected.push_factor(o[X(1)],o[X(2)]);
	expected.push_factor(o[X(1)],o[L(1)]);
	expected.push_factor(o[X(2)],o[L(1)]);

	// construct it from the factor graph
	GaussianFactorGraph factorGraph = example::createGaussianFactorGraph(o);
	SymbolicFactorGraph actual(factorGraph);

	CHECK(assert_equal(expected, actual));
}

///* ************************************************************************* */
//TEST( SymbolicFactorGraph, findAndRemoveFactors )
//{
//	// construct it from the factor graph graph
//	GaussianFactorGraph factorGraph = example::createGaussianFactorGraph();
//	SymbolicFactorGraph actual(factorGraph);
//  SymbolicFactor::shared_ptr f1 = actual[0];
//  SymbolicFactor::shared_ptr f3 = actual[2];
//	actual.findAndRemoveFactors(X(2));
//
//	// construct expected graph after find_factors_and_remove
//	SymbolicFactorGraph expected;
//	SymbolicFactor::shared_ptr null;
//	expected.push_back(f1);
//	expected.push_back(null);
//	expected.push_back(f3);
//	expected.push_back(null);
//
//	CHECK(assert_equal(expected, actual));
//}
///* ************************************************************************* */
//TEST( SymbolicFactorGraph, factors)
//{
//	// create a test graph
//	GaussianFactorGraph factorGraph = example::createGaussianFactorGraph();
//	SymbolicFactorGraph fg(factorGraph);
//
//	// ask for all factor indices connected to x1
//	list<size_t> x1_factors = fg.factors(X(1));
//	int x1_indices[] = { 0, 1, 2 };
//	list<size_t> x1_expected(x1_indices, x1_indices + 3);
//	CHECK(x1_factors==x1_expected);
//
//	// ask for all factor indices connected to x2
//	list<size_t> x2_factors = fg.factors(X(2));
//	int x2_indices[] = { 1, 3 };
//	list<size_t> x2_expected(x2_indices, x2_indices + 2);
//	CHECK(x2_factors==x2_expected);
//}

///* ************************************************************************* */
//TEST( SymbolicFactorGraph, removeAndCombineFactors )
//{
//	// create a test graph
//	GaussianFactorGraph factorGraph = example::createGaussianFactorGraph();
//	SymbolicFactorGraph fg(factorGraph);
//
//  // combine all factors connected to x1
//  SymbolicFactor::shared_ptr actual = removeAndCombineFactors(fg,X(1));
//
//  // check result
//  SymbolicFactor expected(L(1),X(1),X(2));
//  CHECK(assert_equal(expected,*actual));
//}

///* ************************************************************************* */
//TEST( SymbolicFactorGraph, eliminateOne )
//{
//  Ordering o; o += X(1),L(1),X(2);
//	// create a test graph
//	GaussianFactorGraph factorGraph = example::createGaussianFactorGraph(o);
//	SymbolicFactorGraph fg(factorGraph);
//
//	// eliminate
//	IndexConditional::shared_ptr actual = GaussianSequentialSolver::EliminateUntil(fg, o[X(1)]+1);
//
//  // create expected symbolic IndexConditional
//  IndexConditional expected(o[X(1)],o[L(1)],o[X(2)]);
//
//  CHECK(assert_equal(expected,*actual));
//}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, eliminate )
{
  Ordering o; o += X(2),L(1),X(1);

  // create expected Chordal bayes Net
  IndexConditional::shared_ptr x2(new IndexConditional(o[X(2)], o[L(1)], o[X(1)]));
  IndexConditional::shared_ptr l1(new IndexConditional(o[L(1)], o[X(1)]));
  IndexConditional::shared_ptr x1(new IndexConditional(o[X(1)]));

  SymbolicBayesNet expected;
  expected.push_back(x2);
  expected.push_back(l1);
  expected.push_back(x1);

  // create a test graph
	GaussianFactorGraph factorGraph = example::createGaussianFactorGraph(o);
	SymbolicFactorGraph fg(factorGraph);

	// eliminate it
  SymbolicBayesNet actual = *SymbolicSequentialSolver(fg).eliminate();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
