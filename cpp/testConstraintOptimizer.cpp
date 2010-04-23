/**
 * @file testConstraintOptimizer.cpp
 * @brief Tests the optimization engine for SQP and BFGS Quadratic programming techniques
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>

#include <ConstraintOptimizer.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Example of a single Constrained QP problem from the matlab testCQP.m file.
TEST( matrix, CQP_example ) {

	Matrix A = Matrix_(3, 2,
			-1.0,  -1.0,
			-2.0,   1.0,
			 1.0,  -1.0);
	Matrix At = trans(A),
		   B = 2.0 * eye(3,3);

	Vector b = Vector_(2, 4.0, -2.0),
	 	   g = zero(3);

	Matrix G = zeros(5,5);
	insertSub(G, B, 0, 0);
	insertSub(G, A, 0, 3);
	insertSub(G, At, 3, 0);

	Vector rhs = zero(5);
	subInsert(rhs, -1.0*g, 0);
	subInsert(rhs, -1.0*b, 3);

	// solve the system with the LDL solver
	Vector actualFull = solve_ldl(G, rhs);
	Vector actual = sub(actualFull, 0, 3);

	Vector expected = Vector_(3, 2.0/7.0, 10.0/7.0, -6.0/7.0);

	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( matrix, CQP_example_automatic ) {

	Matrix A = Matrix_(3, 2,
			-1.0,  -1.0,
			-2.0,   1.0,
			 1.0,  -1.0);
	Matrix At = trans(A),
		   B = 2.0 * eye(3,3);

	Vector g = zero(3),
		   h = Vector_(2, 4.0, -2.0);

	Vector actState, actLam;
	boost::tie(actState, actLam) = solveCQP(B, A, g, h);

	Vector expected = Vector_(3, 2.0/7.0, 10.0/7.0, -6.0/7.0);

	CHECK(assert_equal(expected, actState));
	CHECK(actLam.size() == 2);
}
	

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
