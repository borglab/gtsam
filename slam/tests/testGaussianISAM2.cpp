/**
 * @file    testGaussianISAM2.cpp
 * @brief   Unit tests for GaussianISAM2
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/slam/GaussianISAM2.h>
#include <gtsam/slam/smallExample.h>

using namespace std;
using namespace gtsam;
using namespace example;

const double tol = 1e-4;

/* ************************************************************************* */
TEST( ISAM2, solving )
{
	Graph nlfg = createNonlinearFactorGraph();
	Config noisy = createNoisyConfig();
	Ordering ordering;
	ordering += symbol('x', 1);
	ordering += symbol('x', 2);
	ordering += symbol('l', 1);
	// FIXME: commented out due due to compile error in ISAM - this should be fixed
//	GaussianISAM2 btree(nlfg, ordering, noisy);
//	VectorConfig actualDelta = optimize2(btree);
//	VectorConfig delta = createCorrectDelta();
//	CHECK(assert_equal(delta, actualDelta, 0.01));
//	Config actualSolution = noisy.expmap(actualDelta);
//	Config solution = createConfig();
//	CHECK(assert_equal(solution, actualSolution, tol));
}

/* ************************************************************************* */
TEST( ISAM2, ISAM2_smoother )
{
	// Create smoother with 7 nodes
	Graph smoother;
	Config poses;
	boost::tie(smoother, poses) = createNonlinearSmoother(7);

	// run ISAM2 for every factor
	GaussianISAM2 actual;
	BOOST_FOREACH(boost::shared_ptr<NonlinearFactor<Config> > factor, smoother) {
		Graph factorGraph;
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
	// FIXME: commented out due due to compile error in ISAM - this should be fixed
//	for (int i=1; i<=7; i++)
//		e.insert(symbol('x', i), v);
//	VectorConfig optimized = optimize2(actual); // actual solution
//	CHECK(assert_equal(e, optimized));
}

/* ************************************************************************* */
TEST( ISAM2, ISAM2_smoother2 )
{
	// Create smoother with 7 nodes
	Graph smoother;
	Config poses;
	boost::tie(smoother, poses) = createNonlinearSmoother(7);

	// Create initial tree from first 4 timestamps in reverse order !
	Ordering ord; ord += "x4","x3","x2","x1";
	Graph factors1;
	for (int i=0;i<7;i++) factors1.push_back(smoother[i]);
	GaussianISAM2 actual(factors1, ord, poses);

	// run ISAM2 with remaining factors
	Graph factors2;
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
