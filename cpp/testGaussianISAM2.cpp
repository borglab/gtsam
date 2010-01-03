/**
 * @file    testGaussianISAM2.cpp
 * @brief   Unit tests for GaussianISAM2
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "Ordering.h"
#include "GaussianBayesNet.h"
#include "ISAM2-inl.h"
#include "GaussianISAM2.h"
#include "smallExample.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* *
TEST( ISAM2, ISAM2_smoother )
{
	// Create smoother with 7 nodes
	ExampleNonlinearFactorGraph smoother;
	VectorConfig poses;
	boost::tie(smoother, poses) = createNonlinearSmoother(7);

	// run ISAM2 for every factor
	GaussianISAM2 actual;
	BOOST_FOREACH(boost::shared_ptr<NonlinearFactor<VectorConfig> > factor, smoother) {
		ExampleNonlinearFactorGraph factorGraph;
		factorGraph.push_back(factor);
		actual.update(factorGraph, poses);
	}

	// Create expected Bayes Tree by solving smoother with "natural" ordering
	Ordering ordering;
	for (int t = 1; t <= 7; t++) ordering += symbol('x', t);
	GaussianISAM2 expected(smoother, ordering, poses);

	// Check whether BayesTree is correct
	CHECK(assert_equal(expected, actual));

	// obtain solution
	VectorConfig e; // expected solution
	Vector v = Vector_(2, 0., 0.);
	for (int i=1; i<=7; i++)
		e.insert(symbol('x', i), v);
	VectorConfig optimized = optimize2(actual); // actual solution
	CHECK(assert_equal(e, optimized));
}

/* ************************************************************************* */
TEST( ISAM2, ISAM2_smoother2 )
{
	// Create smoother with 7 nodes
	ExampleNonlinearFactorGraph smoother;
	VectorConfig poses;
	boost::tie(smoother, poses) = createNonlinearSmoother(7);

	// Create initial tree from first 4 timestamps in reverse order !
	Ordering ord; ord += "x4","x3","x2","x1";
	ExampleNonlinearFactorGraph factors1;
	for (int i=0;i<7;i++) factors1.push_back(smoother[i]);
	GaussianISAM2 actual(factors1, ord, poses);

	// run ISAM2 with remaining factors
	ExampleNonlinearFactorGraph factors2;
	for (int i=7;i<13;i++) factors2.push_back(smoother[i]);
	actual.update(factors2, poses);

	// Create expected Bayes Tree by solving smoother with "natural" ordering
	Ordering ordering;
	for (int t = 1; t <= 7; t++) ordering += symbol('x', t);
	GaussianISAM2 expected(smoother, ordering, poses);

	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
