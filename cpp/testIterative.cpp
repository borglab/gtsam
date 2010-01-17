/**
 *  @file   testIterative.cpp
 *  @brief  Unit tests for iterative methods
 *  @author Frank Dellaert
 **/

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include "Ordering.h"
#include "iterative.h"
#include "smallExample.h"
#include "pose2SLAM.h"
#include "FactorGraph-inl.h"
#include "NonlinearFactorGraph-inl.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( Iterative, steepestDescent )
{
	// Expected solution
	Ordering ord;
	ord += "l1", "x1", "x2";
	GaussianFactorGraph fg = createGaussianFactorGraph();
	VectorConfig expected = fg.optimize(ord); // destructive

	// Do gradient descent
	GaussianFactorGraph fg2 = createGaussianFactorGraph();
	VectorConfig zero = createZeroDelta();
	bool verbose = false;
	VectorConfig actual = steepestDescent(fg2, zero, verbose);
	CHECK(assert_equal(expected,actual,1e-2));
}

/* ************************************************************************* */
TEST( Iterative, conjugateGradientDescent )
{
	// Expected solution
	Ordering ord;
	ord += "l1", "x1", "x2";
	GaussianFactorGraph fg = createGaussianFactorGraph();
	VectorConfig expected = fg.optimize(ord); // destructive

	// create graph and get matrices
	GaussianFactorGraph fg2 = createGaussianFactorGraph();
	Matrix A;
	Vector b;
	Vector x0 = gtsam::zero(6);
	boost::tie(A, b) = fg2.matrix(ord);
	Vector expectedX = Vector_(6, -0.1, 0.1, -0.1, -0.1, 0.1, -0.2);

	// Do conjugate gradient descent, System version
	System Ab(A, b);
	Vector actualX = conjugateGradientDescent(Ab, x0);
	CHECK(assert_equal(expectedX,actualX,1e-9));

	// Do conjugate gradient descent, Matrix version
	Vector actualX2 = conjugateGradientDescent(A, b, x0);
	CHECK(assert_equal(expectedX,actualX2,1e-9));

	// Do conjugate gradient descent on factor graph
	VectorConfig zero = createZeroDelta();
	VectorConfig actual = conjugateGradientDescent(fg2, zero);
	CHECK(assert_equal(expected,actual,1e-2));

	// Test method
	VectorConfig actual2 = fg2.conjugateGradientDescent(zero);
	CHECK(assert_equal(expected,actual2,1e-2));
}

/* ************************************************************************* *
TEST( Iterative, conjugateGradientDescent_hard_constraint )
{
	typedef Pose2Config::Key Key;

	Pose2Config config;
	config.insert(1, Pose2(0.,0.,0.));
	config.insert(2, Pose2(1.5,0.,0.));

	Pose2Graph graph;
	Matrix cov = eye(3);
	graph.push_back(Pose2Graph::sharedFactor(new Pose2Factor(Key(1), Key(2), Pose2(1.,0.,0.), cov)));
	graph.addConstraint(1, config[1]);

	VectorConfig zeros;
	zeros.insert("x1",zero(3));
	zeros.insert("x2",zero(3));

	GaussianFactorGraph fg = graph.linearize(config);
	VectorConfig actual = conjugateGradientDescent(fg, zeros, true, 1e-3, 1e-5, 10);

	VectorConfig expected;
	expected.insert("x1", zero(3));
	expected.insert("x2", Vector_(-0.5,0.,0.));
	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Iterative, conjugateGradientDescent_soft_constraint )
{
	Pose2Config config;
	config.insert(1, Pose2(0.,0.,0.));
	config.insert(2, Pose2(1.5,0.,0.));

	Pose2Graph graph;
	Matrix cov = eye(3);
	Matrix cov2 = eye(3) * 1e-10;
	graph.addPrior(1, Pose2(0.,0.,0.), cov2);
	graph.addConstraint(1,2, Pose2(1.,0.,0.), cov);

	VectorConfig zeros;
	zeros.insert("x1",zero(3));
	zeros.insert("x2",zero(3));

	GaussianFactorGraph fg = graph.linearize(config);
	VectorConfig actual = conjugateGradientDescent(fg, zeros, false, 1e-3, 1e-5, 100);

	VectorConfig expected;
	expected.insert("x1", zero(3));
	expected.insert("x2", Vector_(3,-0.5,0.,0.));
	CHECK(assert_equal(expected, actual));
}
/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
