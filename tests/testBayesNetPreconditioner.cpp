/**
 *  @file   testBayesNetConditioner.cpp
 *  @brief  Unit tests for BayesNetConditioner
 *  @author Frank Dellaert
 **/

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/BayesNetPreconditioner.h>
#include <gtsam/linear/iterative-inl.h>

using namespace std;
using namespace gtsam;

#include <gtsam/slam/smallExample.h>
using namespace example;

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
	BOOST_FOREACH(const Symbol& j, ordering) y0.insert(j,z2);

	VectorConfig y1 = y0;
	y1["x2003"] = Vector_(2, 1.0, -1.0);
	VectorConfig x1 = system.x(y1);

	// Check gradient for y0
	VectorConfig expectedGradient0;
	expectedGradient0.insert("x1001", Vector_(2,-1000.,-1000.));
	expectedGradient0.insert("x1002", Vector_(2,    0., -300.));
	expectedGradient0.insert("x1003", Vector_(2,    0., -300.));
	expectedGradient0.insert("x2001", Vector_(2, -100.,  200.));
	expectedGradient0.insert("x2002", Vector_(2, -100.,    0.));
	expectedGradient0.insert("x2003", Vector_(2, -100., -200.));
	expectedGradient0.insert("x3001", Vector_(2, -100.,  100.));
	expectedGradient0.insert("x3002", Vector_(2, -100.,    0.));
	expectedGradient0.insert("x3003", Vector_(2, -100., -100.));
	VectorConfig actualGradient0 = system.gradient(y0);
	CHECK(assert_equal(expectedGradient0,actualGradient0));
#ifdef VECTORBTREE
	CHECK(actualGradient0.cloned(y0));
#endif

	// Solve using PCG
	bool verbose = false;
	double epsilon = 1e-6; // had to crank this down !!!
	size_t maxIterations = 100;
	VectorConfig actual_y = gtsam::conjugateGradients<BayesNetPreconditioner,
			VectorConfig, Errors>(system, y1, verbose, epsilon, epsilon, maxIterations);
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
