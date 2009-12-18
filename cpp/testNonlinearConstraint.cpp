/**
 * @file testNonlinearConstraint.cpp
 * @brief Tests for nonlinear constraints handled via SQP
 * @author Alex Cunningham
 */

#include <list>
#include <boost/bind.hpp>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/list.hpp> // for operator +=
#include <VectorConfig.h>
#include <NonlinearConstraint.h>
#include <NonlinearConstraint-inl.h>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
// unary functions with scalar variables
/* ************************************************************************* */
namespace test1 {

	/** p = 1, g(x) = x^2-5 = 0 */
	Vector g(const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		return Vector_(1, x * x - 5);
	}

	/** p = 1, jacobianG(x) = 2x */
	Matrix G(const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, 2 * x);
	}

} // \namespace test1

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_scalar_construction ) {
	// construct a constraint on x
	// the lagrange multipliers will be expected on L_x1
	// and there is only one multiplier
	size_t p = 1;
	list<string> keys;	keys += "x";
	NonlinearConstraint1<VectorConfig> c1(boost::bind(test1::g, _1, keys),
										  "x", boost::bind(test1::G, _1, keys),
										  p, "L_x1");

	// get a configuration to use for finding the error
	VectorConfig config;
	config.insert("x", Vector_(1, 1.0));

	// calculate the error
	Vector actual = c1.error_vector(config);
	Vector expected = Vector_(1, -4.0);
	CHECK(assert_equal(actual, expected, 1e-5));
}

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_scalar_linearize ) {
	size_t p = 1;
	list<string> keys;	keys += "x";
	NonlinearConstraint1<VectorConfig> c1(boost::bind(test1::g, _1, keys),
										  "x", boost::bind(test1::G, _1, keys),
										  p, "L_x1");

	// get a configuration to use for linearization
	VectorConfig realconfig;
	realconfig.insert("x", Vector_(1, 1.0));

	// get a configuration of Lagrange multipliers
	VectorConfig lagrangeConfig;
	lagrangeConfig.insert("L_x1", Vector_(1, 3.0));

	// linearize the system
	GaussianFactor::shared_ptr actualFactor, actualConstraint;
	boost::tie(actualFactor, actualConstraint) = c1.linearize(realconfig, lagrangeConfig);

	// verify
	GaussianFactor expectedFactor("x", Matrix_(1,1, 6.0), "L_x1", eye(1), zero(1), 1.0);
	GaussianFactor expectedConstraint("x", Matrix_(1,1, 2.0), Vector_(1, 4.0), 0.0);
	CHECK(assert_equal(*actualFactor, expectedFactor));
	CHECK(assert_equal(*actualConstraint, expectedConstraint));
}

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_scalar_equal ) {
	list<string> keys1, keys2; keys1 += "x"; keys2 += "y";
	NonlinearConstraint1<VectorConfig>
		c1(boost::bind(test1::g, _1, keys1), "x", boost::bind(test1::G, _1, keys1), 1, "L_x1", true),
		c2(boost::bind(test1::g, _1, keys1), "x", boost::bind(test1::G, _1, keys1), 1, "L_x1"),
		c3(boost::bind(test1::g, _1, keys1), "x", boost::bind(test1::G, _1, keys1), 2, "L_x1"),
		c4(boost::bind(test1::g, _1, keys2), "y", boost::bind(test1::G, _1, keys2), 1, "L_x1");

	CHECK(assert_equal(c1, c2));
	CHECK(assert_equal(c2, c1));
	CHECK(!c1.equals(c3));
	CHECK(!c1.equals(c4));
}

/* ************************************************************************* */
// binary functions with scalar variables
/* ************************************************************************* */
namespace test2 {

	/** p = 1, g(x) = x^2-5 -y = 0 */
	Vector g(const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		double y = config[keys.back()](0);
		return Vector_(1, x * x - 5.0 - y);
	}

	/** jacobian for x, jacobianG(x,y) in x: 2x*/
	Matrix G1(const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, 2.0 * x);
	}

	/** jacobian for y, jacobianG(x,y) in y: -1 */
	Matrix G2(const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.back()](0);
		return Matrix_(1, 1, -1.0);
	}

} // \namespace test2

/* ************************************************************************* */
TEST( NonlinearConstraint2, binary_scalar_construction ) {
	// construct a constraint on x and y
	// the lagrange multipliers will be expected on L_xy
	// and there is only one multiplier
	size_t p = 1;
	list<string> keys; keys += "x", "y";
	NonlinearConstraint2<VectorConfig> c1(
			boost::bind(test2::g, _1, keys),
			"x", boost::bind(test2::G1, _1, keys),
			"y", boost::bind(test2::G1, _1, keys),
			p, "L_xy");

	// get a configuration to use for finding the error
	VectorConfig config;
	config.insert("x", Vector_(1, 1.0));
	config.insert("y", Vector_(1, 2.0));

	// calculate the error
	Vector actual = c1.error_vector(config);
	Vector expected = Vector_(1.0, -6.0);
	CHECK(assert_equal(actual, expected, 1e-5));
}

/* ************************************************************************* */
TEST( NonlinearConstraint2, binary_scalar_linearize ) {
	// create a constraint
	size_t p = 1;
	list<string> keys; keys += "x", "y";
	NonlinearConstraint2<VectorConfig> c1(
			boost::bind(test2::g, _1, keys),
			"x", boost::bind(test2::G1, _1, keys),
			"y", boost::bind(test2::G2, _1, keys),
			p, "L_xy");

	// get a configuration to use for finding the error
	VectorConfig realconfig;
	realconfig.insert("x", Vector_(1, 1.0));
	realconfig.insert("y", Vector_(1, 2.0));

	// get a configuration of Lagrange multipliers
	VectorConfig lagrangeConfig;
	lagrangeConfig.insert("L_xy", Vector_(1, 3.0));

	// linearize the system
	GaussianFactor::shared_ptr actualFactor, actualConstraint;
	boost::tie(actualFactor, actualConstraint) = c1.linearize(realconfig, lagrangeConfig);

	// verify
	GaussianFactor expectedFactor("x", Matrix_(1,1, 6.0),
							 "y", Matrix_(1,1, -3.0),
							 "L_xy", eye(1), zero(1), 1.0);
	GaussianFactor expectedConstraint("x", Matrix_(1,1, 2.0),
								 "y", Matrix_(1,1, -1.0),
								 Vector_(1, 6.0), 0.0);
	CHECK(assert_equal(*actualFactor, expectedFactor));
	CHECK(assert_equal(*actualConstraint, expectedConstraint));
}

/* ************************************************************************* */
TEST( NonlinearConstraint2, binary_scalar_equal ) {
	list<string> keys1, keys2, keys3;
	keys1 += "x", "y"; keys2 += "y", "x"; keys3 += "x", "z";
	NonlinearConstraint2<VectorConfig>
		c1(boost::bind(test2::g, _1, keys1), "x", boost::bind(test2::G1, _1, keys1), "y", boost::bind(test2::G2, _1, keys1), 1, "L_xy"),
		c2(boost::bind(test2::g, _1, keys1), "x", boost::bind(test2::G1, _1, keys1), "y", boost::bind(test2::G2, _1, keys1), 1, "L_xy"),
		c3(boost::bind(test2::g, _1, keys2), "y", boost::bind(test2::G1, _1, keys2), "x", boost::bind(test2::G2, _1, keys2), 1, "L_xy"),
		c4(boost::bind(test2::g, _1, keys3), "x", boost::bind(test2::G1, _1, keys3), "z", boost::bind(test2::G2, _1, keys3), 3, "L_xy");

	CHECK(assert_equal(c1, c2));
	CHECK(assert_equal(c2, c1));
	CHECK(!c1.equals(c3));
	CHECK(!c1.equals(c4));
}

/* ************************************************************************* */
// Inequality tests
/* ************************************************************************* */
namespace inequality1 {

	/** p = 1, g(x) x^2 - 5 > 0 */
	Vector g(const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		double g = x * x - 5;
		return Vector_(1, g); // return the actual cost
	}

	/** p = 1, jacobianG(x) = 2*x */
	Matrix G(const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, 2 * x);
	}

} // \namespace inequality1

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_inequality ) {
	size_t p = 1;
	list<string> keys; keys += "x";
	NonlinearConstraint1<VectorConfig> c1(boost::bind(inequality1::g, _1, keys),
										  "x", boost::bind(inequality1::G, _1, keys),
										   p, "L_x1",
										  false); // inequality constraint

	// get configurations to use for evaluation
	VectorConfig config1, config2;
	config1.insert("x", Vector_(1, 10.0)); // should be inactive
	config2.insert("x", Vector_(1, 1.0)); // should have nonzero error

	// check error
	CHECK(!c1.active(config1));
	Vector actualError2 = c1.error_vector(config2);
	CHECK(assert_equal(actualError2, Vector_(1, -4.0, 1e-9)));
	CHECK(c1.active(config2));
}

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_inequality_linearize ) {
	size_t p = 1;
	list<string> keys; keys += "x";
	NonlinearConstraint1<VectorConfig> c1(boost::bind(inequality1::g, _1, keys),
										  "x", boost::bind(inequality1::G, _1, keys),
										   p, "L_x1",
										  false); // inequality constraint

	// get configurations to use for linearization
	VectorConfig config1, config2;
	config1.insert("x", Vector_(1, 10.0)); // should have zero error
	config2.insert("x", Vector_(1, 1.0)); // should have nonzero error

	// get a configuration of Lagrange multipliers
	VectorConfig lagrangeConfig;
	lagrangeConfig.insert("L_x1", Vector_(1, 3.0));

	// linearize for inactive constraint
	GaussianFactor::shared_ptr actualFactor1, actualConstraint1;
	boost::tie(actualFactor1, actualConstraint1) = c1.linearize(config1, lagrangeConfig);

	// check if the factor is active
	CHECK(!c1.active(config1));

	// linearize for active constraint
	GaussianFactor::shared_ptr actualFactor2, actualConstraint2;
	boost::tie(actualFactor2, actualConstraint2) = c1.linearize(config2, lagrangeConfig);
	CHECK(c1.active(config2));

	// verify
	GaussianFactor expectedFactor("x", Matrix_(1,1, 6.0), "L_x1", eye(1), zero(1), 1.0);
	GaussianFactor expectedConstraint("x", Matrix_(1,1, 2.0), Vector_(1, 4.0), 0.0);
	CHECK(assert_equal(*actualFactor2, expectedFactor));
	CHECK(assert_equal(*actualConstraint2, expectedConstraint));
}

/* ************************************************************************* */
// Binding arbitrary functions
/* ************************************************************************* */
namespace binding1 {

	/** p = 1, g(x) x^2 - r > 0 */
	Vector g(double r, const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		double g = x * x - r;
		return Vector_(1, g); // return the actual cost
	}

	/** p = 1, jacobianG(x) = 2*x */
	Matrix G(double coeff, const VectorConfig& config,
			const list<string>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, coeff * x);
	}

} // \namespace binding1

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_binding ) {
	size_t p = 1;
	double coeff = 2;
	double radius = 5;
	list<string> keys; keys += "x";
	NonlinearConstraint1<VectorConfig> c1(
										  boost::bind(binding1::g, radius, _1, keys),
										  "x", boost::bind(binding1::G, coeff, _1, keys),
										  p, "L_x1",
										  false); // inequality constraint

	// get configurations to use for evaluation
	VectorConfig config1, config2;
	config1.insert("x", Vector_(1, 10.0)); // should have zero error
	config2.insert("x", Vector_(1, 1.0)); // should have nonzero error

	// check error
	CHECK(!c1.active(config1));
	Vector actualError2 = c1.error_vector(config2);
	CHECK(assert_equal(actualError2, Vector_(1, -4.0, 1e-9)));
	CHECK(c1.active(config2));
}

/* ************************************************************************* */
namespace binding2 {

	/** p = 1, g(x) = x^2-5 -y = 0 */
	Vector g(double r, const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		double y = config[keys.back()](0);
		return Vector_(1, x * x - r - y);
	}

	/** jacobian for x, jacobianG(x,y) in x: 2x*/
	Matrix G1(double c, const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, c * x);
	}

	/** jacobian for y, jacobianG(x,y) in y: -1 */
	Matrix G2(double c, const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.back()](0);
		return Matrix_(1, 1, -1.0 * c);
	}

} // \namespace binding2

/* ************************************************************************* */
TEST( NonlinearConstraint2, binary_binding ) {
	// construct a constraint on x and y
	// the lagrange multipliers will be expected on L_xy
	// and there is only one multiplier
	size_t p = 1;
	double a = 2.0;
	double b = 1.0;
	double r = 5.0;
	list<string> keys; keys += "x", "y";
	NonlinearConstraint2<VectorConfig> c1(
			boost::bind(binding2::g, r, _1, keys),
			"x", boost::bind(binding2::G1, a, _1, keys),
			"y", boost::bind(binding2::G2, b, _1, keys),
			p, "L_xy");

	// get a configuration to use for finding the error
	VectorConfig config;
	config.insert("x", Vector_(1, 1.0));
	config.insert("y", Vector_(1, 2.0));

	// calculate the error
	Vector actual = c1.error_vector(config);
	Vector expected = Vector_(1.0, -6.0);
	CHECK(assert_equal(actual, expected, 1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
