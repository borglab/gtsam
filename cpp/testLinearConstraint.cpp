/**
 * @file testLinearConstraint.cpp
 * @brief Tests for linear constraints
 * @author Alex Cunningham
 */


#include <CppUnitLite/TestHarness.h>
#include "LinearConstraint.h"
#include "smallExample.h"

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST ( LinearConstraint, basic_unary )
{
	// create an initialized factor with a unary factor
	Vector v(2); v(0)=1.2; v(1)=3.4;
	string key = "x0";
	LinearConstraint factor(v, key);

	// get the data back out of it
	CHECK(assert_equal(v, factor.get_b()));
	Matrix expected = eye(2);
	CHECK(assert_equal(expected, factor.get_A("x0")));
}

/* ************************************************************************* */
TEST( LinearConstraint, basic_binary )
{
	Matrix A1(2,2);
	A1(0,0) = -10.0 ; A1(0,1) = 0.0;
	A1(1,0) = 0.0 ; A1(1,1) = -10.0;

	Matrix A2(2,2);
	A2(0,0) = 10.0 ; A2(0,1) = 0.0;
	A2(1,0) = 0.0 ; A2(1,1) = 10.0;

	Vector b(2);
	b(0) = 2 ; b(1) = -1;

	LinearConstraint lc("x1", A1,  "x2", A2, b);

	// verify contents
	CHECK( assert_equal(A1, lc.get_A("x1")));
	CHECK( assert_equal(A2, lc.get_A("x2")));
	CHECK( assert_equal(b, lc.get_b()));
}

/* ************************************************************************* */
TEST( LinearConstraint, basic_arbitrary )
{
	Matrix A1(2,2);
	A1(0,0) = -10.0 ; A1(0,1) = 0.0;
	A1(1,0) = 0.0 ; A1(1,1) = -10.0;

	Matrix A2(2,2);
	A2(0,0) = 10.0 ; A2(0,1) = 0.0;
	A2(1,0) = 0.0 ; A2(1,1) = 10.0;

	Matrix A3(2,2);
	A3(0,0) = 10.0 ; A3(0,1) = 7.0;
	A3(1,0) = 7.0 ; A3(1,1) = 10.0;

	Vector b(2);
	b(0) = 2 ; b(1) = -1;

	// build a map
	map<string, Matrix> matrices;
	matrices.insert(make_pair("x1", A1));
	matrices.insert(make_pair("x2", A2));
	matrices.insert(make_pair("x3", A3));

	LinearConstraint lc(matrices, b);

	// verify contents
	CHECK( assert_equal(A1, lc.get_A("x1")));
	CHECK( assert_equal(A2, lc.get_A("x2")));
	CHECK( assert_equal(A3, lc.get_A("x3")));
	CHECK( assert_equal(b, lc.get_b()));
}

/* ************************************************************************* */
TEST ( LinearConstraint, size )
{
	Matrix A1(2,2);
	A1(0,0) = -10.0 ; A1(0,1) = 0.0;
	A1(1,0) = 0.0 ; A1(1,1) = -10.0;

	Matrix A2(2,2);
	A2(0,0) = 10.0 ; A2(0,1) = 0.0;
	A2(1,0) = 0.0 ; A2(1,1) = 10.0;

	Matrix A3(2,2);
	A3(0,0) = 10.0 ; A3(0,1) = 7.0;
	A3(1,0) = 7.0 ; A3(1,1) = 10.0;

	Vector b(2);
	b(0) = 2 ; b(1) = -1;

	// build some constraints
	LinearConstraint lc1(b, "x1");
	LinearConstraint lc2("x1", A1,  "x2", A2, b);
	map<string, Matrix> matrices;
	matrices.insert(make_pair("x1", A1));
	matrices.insert(make_pair("x2", A2));
	matrices.insert(make_pair("x3", A3));
	LinearConstraint lc3(matrices, b);

	CHECK(lc1.size() == 1);
	CHECK(lc2.size() == 2);
	CHECK(lc3.size() == 3);

}

/* ************************************************************************* */
TEST ( LinearConstraint, equals )
{
	Matrix A1(2,2);
	A1(0,0) = -10.0 ; A1(0,1) = 0.0;
	A1(1,0) = 0.0 ; A1(1,1) = -10.0;

	Matrix A2(2,2);
	A2(0,0) = 10.0 ; A2(0,1) = 0.0;
	A2(1,0) = 0.0 ; A2(1,1) = 10.0;

	Vector b(2);
	b(0) = 2 ; b(1) = -1;

	LinearConstraint lc1("x1", A1,  "x2", A2, b);
	LinearConstraint lc2("x1", A1,  "x2", A2, b);

	// check for basic equality
	CHECK(lc1.equals(lc2));
	CHECK(lc2.equals(lc1));
}

/* ************************************************************************* */
TEST ( LinearConstraint, eliminate1 )
{
	// construct a linear constraint
	Vector v(2); v(0)=1.2; v(1)=3.4;
	string key = "x0";
	LinearConstraint lc(v, key);

	// eliminate it to get a constrained conditional gaussian
	ConstrainedConditionalGaussian::shared_ptr ccg = lc.eliminate(key);

	// solve the ccg to get a value
	FGConfig fg;
	CHECK(assert_equal(ccg->solve(fg), v));
}

/* ************************************************************************* */
TEST ( LinearConstraint, eliminate2 )
{
	// Construct a linear constraint
	// RHS
	Vector b(2); b(0)=3.0; b(1)=4.0;

	// A1 - invertible
	Matrix A1(2,2);
	A1(0,0) = 1.0 ; A1(0,1) = 2.0;
	A1(1,0) = 2.0 ; A1(1,1) = 1.0;

	// A2 - not invertible - solve will throw an exception
	Matrix A2(2,2);
	A2(0,0) = 1.0 ; A2(0,1) = 2.0;
	A2(1,0) = 2.0 ; A2(1,1) = 4.0;

	LinearConstraint lc("x", A1, "y", A2, b);

	Vector y = Vector_(2, 1.0, 2.0);

	FGConfig fg1;
	fg1.insert("y", y);

	Vector expected = Vector_(2, -3.3333, 0.6667);

	// eliminate x for basic check
	ConstrainedConditionalGaussian::shared_ptr actual = lc.eliminate("x");
	CHECK(assert_equal(expected, actual->solve(fg1), 1e-4));

	// eliminate y to test thrown error
	FGConfig fg2;
	fg2.insert("x", expected);
	actual = lc.eliminate("y");
	try {
		Vector output = actual->solve(fg2);
		CHECK(false);
	} catch (...) {
		CHECK(true);
	}
}

/* ************************************************************************* */
TEST ( LinearConstraint, involves )
{
	Matrix A1(2,2);
	A1(0,0) = -10.0 ; A1(0,1) = 0.0;
	A1(1,0) = 0.0 ; A1(1,1) = -10.0;

	Matrix A2(2,2);
	A2(0,0) = 10.0 ; A2(0,1) = 0.0;
	A2(1,0) = 0.0 ; A2(1,1) = 10.0;

	Vector b(2);
	b(0) = 2 ; b(1) = -1;

	LinearConstraint lc("x1", A1,  "x2", A2, b);

	CHECK(lc.involves("x1"));
	CHECK(lc.involves("x2"));
	CHECK(!lc.involves("x3"));
}

/* ************************************************************************* */
TEST ( LinearConstraint, keys )
{
	Matrix A1(2,2);
	A1(0,0) = -10.0 ; A1(0,1) = 0.0;
	A1(1,0) = 0.0 ; A1(1,1) = -10.0;

	Matrix A2(2,2);
	A2(0,0) = 10.0 ; A2(0,1) = 0.0;
	A2(1,0) = 0.0 ; A2(1,1) = 10.0;

	Vector b(2);
	b(0) = 2 ; b(1) = -1;

	LinearConstraint lc("x1", A1,  "x2", A2, b);

	list<string> actual = lc.keys();

	list<string> expected;
	expected.push_back("x1");
	expected.push_back("x2");

	CHECK(actual == expected);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

