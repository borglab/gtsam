/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicFactorGraph.cpp
 * @brief   Unit tests for a symbolic Factor Graph
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/slam/smallExample.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/inference/inference-inl.h>
#include <gtsam/nonlinear/Ordering.h>

using namespace std;
using namespace gtsam;
using namespace example;

/* ************************************************************************* */
TEST( SymbolicFactorGraph, symbolicFactorGraph )
{
  Ordering o; o += "x1","l1","x2";
	// construct expected symbolic graph
	SymbolicFactorGraph expected;
	expected.push_factor(o["x1"]);
	expected.push_factor(o["x1"],o["x2"]);
	expected.push_factor(o["x1"],o["l1"]);
	expected.push_factor(o["l1"],o["x2"]);

	// construct it from the factor graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph(o);
	SymbolicFactorGraph actual(factorGraph);

	CHECK(assert_equal(expected, actual));
}

///* ************************************************************************* */
//TEST( SymbolicFactorGraph, findAndRemoveFactors )
//{
//	// construct it from the factor graph graph
//	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
//	SymbolicFactorGraph actual(factorGraph);
//  SymbolicFactor::shared_ptr f1 = actual[0];
//  SymbolicFactor::shared_ptr f3 = actual[2];
//	actual.findAndRemoveFactors("x2");
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
//	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
//	SymbolicFactorGraph fg(factorGraph);
//
//	// ask for all factor indices connected to x1
//	list<size_t> x1_factors = fg.factors("x1");
//	int x1_indices[] = { 0, 1, 2 };
//	list<size_t> x1_expected(x1_indices, x1_indices + 3);
//	CHECK(x1_factors==x1_expected);
//
//	// ask for all factor indices connected to x2
//	list<size_t> x2_factors = fg.factors("x2");
//	int x2_indices[] = { 1, 3 };
//	list<size_t> x2_expected(x2_indices, x2_indices + 2);
//	CHECK(x2_factors==x2_expected);
//}

///* ************************************************************************* */
//TEST( SymbolicFactorGraph, removeAndCombineFactors )
//{
//	// create a test graph
//	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
//	SymbolicFactorGraph fg(factorGraph);
//
//  // combine all factors connected to x1
//  SymbolicFactor::shared_ptr actual = removeAndCombineFactors(fg,"x1");
//
//  // check result
//  SymbolicFactor expected("l1","x1","x2");
//  CHECK(assert_equal(expected,*actual));
//}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, eliminateOne )
{
  Ordering o; o += "x1","l1","x2";
	// create a test graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph(o);
	SymbolicFactorGraph fg(factorGraph);

	// eliminate
	VariableIndex<> varindex(fg);
	IndexConditional::shared_ptr actual = Inference::EliminateOne(fg, varindex, o["x1"]);

  // create expected symbolic IndexConditional
  IndexConditional expected(o["x1"],o["l1"],o["x2"]);

  CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, eliminate )
{
  Ordering o; o += "x2","l1","x1";

  // create expected Chordal bayes Net
  IndexConditional::shared_ptr x2(new IndexConditional(o["x2"], o["l1"], o["x1"]));
  IndexConditional::shared_ptr l1(new IndexConditional(o["l1"], o["x1"]));
  IndexConditional::shared_ptr x1(new IndexConditional(o["x1"]));

  SymbolicBayesNet expected;
  expected.push_back(x2);
  expected.push_back(l1);
  expected.push_back(x1);

  // create a test graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph(o);
	SymbolicFactorGraph fg(factorGraph);

	// eliminate it
  SymbolicBayesNet actual = *Inference::Eliminate(fg);

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
