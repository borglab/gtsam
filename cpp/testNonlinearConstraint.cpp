/*
 * @file testNonlinearConstraint.cpp
 * @brief Tests for nonlinear constraints handled via SQP
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <VectorConfig.h>
#include <NonlinearConstraint.h>

using namespace gtsam;

// define some functions to use
// these examples use scalar variables and a single constraint

/* ************************************************************************* */
// unary functions
/* ************************************************************************* */
namespace test1 {
/** p = 1, gradG(x) = 2x */
Matrix grad_g(const VectorConfig& config, const std::string& key) {
	double x = config[key](0);
	return Matrix_(1,1, 2*x);
}

/** p = 1, g(x) = x^2-5 = 0 */
Vector g_func(const VectorConfig& config, const std::string& key) {
	double x = config[key](0);
	return Vector_(1, x*x-5);
}
} // \namespace test1

/* ************************************************************************* */
TEST( NonlinearConstraint, unary_construction ) {
	// construct a constraint on x
	// the lagrange multipliers will be expected on L_x1
	// and there is only one multiplier
	size_t p = 1;
	NonlinearConstraint1<VectorConfig> c1("x", *test1::grad_g, *test1::g_func, p, "L_x1");

	// get a configuration to use for finding the error
	VectorConfig config;
	config.insert("x", Vector_(1, 1.0));

	// calculate the error
	Vector actual = c1.error_vector(config);
	Vector expected = Vector_(1.0, -4.0);
	CHECK(assert_equal(actual, expected, 1e-5));
}

///* ************************************************************************* */
//TEST( NonlinearConstraint, unary_linearize ) {
//	// constuct a constraint
//	Vector lambda = Vector_(1, 1.0);
//	NonlinearConstraint1<VectorConfig> c1("x", *grad_g1, *g1_func, lambda);
//
//	// get a configuration to use for linearization
//	VectorConfig config;
//	config.insert("x", Vector_(1, 1.0));
//
//	// determine the gradient of the function
//	Matrix
//
//}

///* ************************************************************************* */
//// binary functions
//Matrix grad1_g2(const VectorConfig& config) {
//	return eye(1);
//}
//
//Matrix grad2_g2(const VectorConfig& config) {
//	return eye(1);
//}
//
//Vector g2_func(const VectorConfig& config) {
//	return zero(1);
//}
//
///* ************************************************************************* */
//TEST( NonlinearConstraint, binary_construction ) {
//	Vector lambda = Vector_(1, 1.0);
//	NonlinearConstraint2<VectorConfig> c1("x", *grad1_g2, "y", *grad2_g2, *g2_func, lambda);
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
