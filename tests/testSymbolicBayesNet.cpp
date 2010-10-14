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

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/slam/smallExample.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/inference-inl.h>
#include <gtsam/inference/Factor-inl.h>
#include <gtsam/nonlinear/Ordering.h>

using namespace std;
using namespace gtsam;
using namespace example;

//Symbol _B_('B', 0), _L_('L', 0);
//Conditional::shared_ptr
//	B(new Conditional(_B_)),
//	L(new Conditional(_L_, _B_));

/* ************************************************************************* */
TEST( SymbolicBayesNet, constructor )
{
  Ordering o; o += "x2","l1","x1";
	// Create manually
	Conditional::shared_ptr
		x2(new Conditional(o["x2"],o["l1"], o["x1"])),
		l1(new Conditional(o["l1"],o["x1"])),
		x1(new Conditional(o["x1"]));
	BayesNet<Conditional> expected;
	expected.push_back(x2);
	expected.push_back(l1);
	expected.push_back(x1);

	// Create from a factor graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph(o);
	SymbolicFactorGraph fg(factorGraph);

	// eliminate it
  SymbolicBayesNet actual = *Inference::Eliminate(fg);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
