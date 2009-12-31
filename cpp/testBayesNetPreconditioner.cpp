/**
 *  @file   testBayesNetConditioner.cpp
 *  @brief  Unit tests for BayesNetConditioner
 *  @author Frank Dellaert
 **/

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <CppUnitLite/TestHarness.h>

#include "Ordering.h"
#include "smallExample.h"
#include "BayesNetPreconditioner.h"
#include "iterative-inl.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( BayesNetPreconditioner, operators )
{
	// Build a simple Bayes net
	// small Bayes Net x <- y, x=2D, y=1D
	// 1 2 3   x1   0
	// 0 1 2 * x2 = 0
	// 0 0 1   x3   1

	// Create a scalar Gaussian on y
	GaussianBayesNet bn = scalarGaussian("y", 1, 0.1);

	// Add a conditional node with one parent |Rx+Sy-d|
	Matrix R11 = Matrix_(2, 2, 1.0, 2.0, 0.0, 1.0), S12 = Matrix_(2, 1, 3.0, 2.0);
	Vector d = zero(2);
	Vector sigmas = Vector_(2, 0.1, 0.1);
	push_front(bn, "x", d, R11, "y", S12, sigmas);

	// Create Precondioner class
	GaussianFactorGraph dummy;
	BayesNetPreconditioner P(dummy,bn);

	// inv(R1)*d should equal solution [1;-2;1]
	VectorConfig D;
	D.insert("x", d);
	D.insert("y", Vector_(1, 1.0 / 0.1)); // corrected by sigma
	VectorConfig expected1;
	expected1.insert("x", Vector_(2, 1.0, -2.0));
	expected1.insert("y", Vector_(1, 1.0));
	VectorConfig actual1 = P.backSubstitute(D);
	CHECK(assert_equal(expected1,actual1));

	// inv(R1')*ones should equal ?
	VectorConfig ones;
	ones.insert("x", Vector_(2, 1.0, 1.0));
	ones.insert("y", Vector_(1, 1.0));
	VectorConfig expected2;
	expected2.insert("x", Vector_(2, 0.1, -0.1));
	expected2.insert("y", Vector_(1, 0.0));
	VectorConfig actual2 = P.backSubstituteTranspose(ones);
	CHECK(assert_equal(expected2,actual2));
}

/* ************************************************************************* */
TEST( BayesNetPreconditioner, conjugateGradients )
{
	// Build a planar graph
	GaussianFactorGraph Ab;
	VectorConfig xtrue;
	size_t N = 3;
	boost::tie(Ab, xtrue) = planarGraph(N); // A*x-b

	// Get the spanning tree and corresponding ordering
	GaussianFactorGraph Ab1, Ab2; // A1*x-b1 and A2*x-b2
	boost::tie(Ab1, Ab2) = splitOffPlanarTree(N, Ab);

	// Eliminate the spanning tree to build a prior
	Ordering ordering = planarOrdering(N);
	GaussianBayesNet Rc1 = Ab1.eliminate(ordering); // R1*x-c1
	VectorConfig xbar = optimize(Rc1); // xbar = inv(R1)*c1

	// Create BayesNet-preconditioned system
	BayesNetPreconditioner system(Ab,Rc1);

	// Create zero config y0 and perturbed config y1
	VectorConfig y0;
	Vector z2 = zero(2);
	BOOST_FOREACH(const string& j, ordering) y0.insert(j,z2);

	VectorConfig y1 = y0;
	y1.getReference("x23") = Vector_(2, 1.0, -1.0);
	VectorConfig x1 = system.x(y1);

	// Solve using PCG
	bool verbose = false;
	double epsilon = 1e-6; // had to crank this down !!!
	size_t maxIterations = 100;
	VectorConfig actual_y = gtsam::conjugateGradients<BayesNetPreconditioner,
			VectorConfig, Errors>(system, y1, verbose, epsilon, maxIterations);
	VectorConfig actual_x = system.x(actual_y);
	CHECK(assert_equal(xtrue,actual_x));

	// Compare with non preconditioned version:
	VectorConfig actual2 = conjugateGradientDescent(Ab, x1, verbose, epsilon,
			maxIterations);
	CHECK(assert_equal(xtrue,actual2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
