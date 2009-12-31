/**
 *  @file   testSubgraphConditioner.cpp
 *  @brief  Unit tests for SubgraphPreconditioner
 *  @author Frank Dellaert
 **/

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "numericalDerivative.h"
#include "Ordering.h"
#include "smallExample.h"
#include "SubgraphPreconditioner.h"
#include "iterative-inl.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( SubgraphPreconditioner, planarGraph )
{
	// Check planar graph construction
	GaussianFactorGraph A;
	VectorConfig xtrue;
	boost::tie(A, xtrue) = planarGraph(3);
	LONGS_EQUAL(13,A.size());
	LONGS_EQUAL(9,xtrue.size());
	DOUBLES_EQUAL(0,A.error(xtrue),1e-9); // check zero error for xtrue

	// Check canonical ordering
	Ordering expected, ordering = planarOrdering(3);
	expected += "x33", "x23", "x13", "x32", "x22", "x12", "x31", "x21", "x11";
	CHECK(assert_equal(expected,ordering));

	// Check that xtrue is optimal
	GaussianBayesNet R1 = A.eliminate(ordering);
	VectorConfig actual = optimize(R1);
	CHECK(assert_equal(xtrue,actual));
}

/* ************************************************************************* */
TEST( SubgraphPreconditioner, splitOffPlanarTree )
{
	// Build a planar graph
	GaussianFactorGraph A;
	VectorConfig xtrue;
	boost::tie(A, xtrue) = planarGraph(3);

	// Get the spanning tree and constraints, and check their sizes
	GaussianFactorGraph T, C;
	boost::tie(T, C) = splitOffPlanarTree(3, A);
	LONGS_EQUAL(9,T.size());
	LONGS_EQUAL(4,C.size());

	// Check that the tree can be solved to give the ground xtrue
	Ordering ordering = planarOrdering(3);
	GaussianBayesNet R1 = T.eliminate(ordering);
	VectorConfig xbar = optimize(R1);
	CHECK(assert_equal(xtrue,xbar));
}

/* ************************************************************************* */
double error(const VectorConfig& x) {
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

	SubgraphPreconditioner system(Rc1, Ab2, xbar);
	return system.error(x);
}

/* ************************************************************************* */
TEST( SubgraphPreconditioner, system )
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

	// Create Subgraph-preconditioned system
	SubgraphPreconditioner system(Rc1, Ab2, xbar);

	// Create zero config
	VectorConfig zeros;
	Vector z2 = zero(2);
	BOOST_FOREACH(const string& j, ordering) zeros.insert(j,z2);

	// Set up y0 as all zeros
	VectorConfig y0 = zeros;

	// y1 = perturbed y0
	VectorConfig y1 = zeros;
	y1.getReference("x23") = Vector_(2, 1.0, -1.0);

	// Check corresponding x  values
	VectorConfig expected_x1 = xtrue, x1 = system.x(y1);
	expected_x1.getReference("x23") = Vector_(2, 2.01, 2.99);
	expected_x1.getReference("x33") = Vector_(2, 3.01, 2.99);
	CHECK(assert_equal(xtrue, system.x(y0)));
	CHECK(assert_equal(expected_x1,system.x(y1)));

	// Check errors
	DOUBLES_EQUAL(0,Ab.error(xtrue),1e-9);
	DOUBLES_EQUAL(3,Ab.error(x1),1e-9);
	DOUBLES_EQUAL(0,system.error(y0),1e-9);
	DOUBLES_EQUAL(3,system.error(y1),1e-9);

	// Test gradient in x
	VectorConfig expected_gx0 = zeros;
	VectorConfig expected_gx1 = zeros;
	CHECK(assert_equal(expected_gx0,Ab.gradient(xtrue)));
	expected_gx1.getReference("x13") = Vector_(2, -100., 100.);
	expected_gx1.getReference("x22") = Vector_(2, -100., 100.);
	expected_gx1.getReference("x23") = Vector_(2, 200., -200.);
	expected_gx1.getReference("x32") = Vector_(2, -100., 100.);
	expected_gx1.getReference("x33") = Vector_(2, 100., -100.);
	CHECK(assert_equal(expected_gx1,Ab.gradient(x1)));

	// Test gradient in y
	VectorConfig expected_gy0 = zeros;
	VectorConfig expected_gy1 = zeros;
	expected_gy1.getReference("x13") = Vector_(2, 2., -2.);
	expected_gy1.getReference("x22") = Vector_(2, -2., 2.);
	expected_gy1.getReference("x23") = Vector_(2, 3., -3.);
	expected_gy1.getReference("x32") = Vector_(2, -1., 1.);
	expected_gy1.getReference("x33") = Vector_(2, 1., -1.);
	CHECK(assert_equal(expected_gy0,system.gradient(y0)));
	CHECK(assert_equal(expected_gy1,system.gradient(y1)));

	// Check it numerically for good measure
	// TODO use boost::bind(&SubgraphPreconditioner::error,&system,_1)
	Vector numerical_g1 = numericalGradient<VectorConfig> (error, y1, 0.001);
	Vector expected_g1 = Vector_(18, 0., 0., 0., 0., 2., -2., 0., 0., -2., 2.,
			3., -3., 0., 0., -1., 1., 1., -1.);
	CHECK(assert_equal(expected_g1,numerical_g1));
}

/* ************************************************************************* */
TEST( SubgraphPreconditioner, conjugateGradients )
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

	// Create Subgraph-preconditioned system
	SubgraphPreconditioner system(Rc1, Ab2, xbar);

	// Create zero config y0 and perturbed config y1
	VectorConfig y0;
	Vector z2 = zero(2);
	BOOST_FOREACH(const string& j, ordering) y0.insert(j,z2);

	VectorConfig y1 = y0;
	y1.getReference("x23") = Vector_(2, 1.0, -1.0);
	VectorConfig x1 = system.x(y1);

	// Solve for the remaining constraints using PCG
	bool verbose = false;
	double epsilon = 1e-3;
	size_t maxIterations = 100;
	VectorConfig actual = gtsam::conjugateGradients<SubgraphPreconditioner,
			VectorConfig, Errors>(system, y1, verbose, epsilon, maxIterations);
	CHECK(assert_equal(y0,actual));

	// Compare with non preconditioned version:
	VectorConfig actual2 = conjugateGradientDescent(Ab, x1, verbose, epsilon,
			maxIterations);
	CHECK(assert_equal(xtrue,actual2,1e-5));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
