/**
 * @file testNonlinearConstraint.cpp
 * @brief Tests for nonlinear constraints handled via SQP
 * @author Alex Cunningham
 */

#include <boost/bind.hpp>
#include <CppUnitLite/TestHarness.h>
#include <VectorConfig.h>
#include <NonlinearConstraint.h>
#include <NonlinearConstraint-inl.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// unary functions with scalar variables
/* ************************************************************************* */
namespace test1 {

	/** p = 1, g(x) = x^2-5 = 0 */
	Vector g(const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		return Vector_(1, x * x - 5);
	}

	/** p = 1, gradG(x) = 2x */
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
	NonlinearConstraint1<VectorConfig> c1(*test1::g, "x", *test1::G, p, "L_x1");

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
	NonlinearConstraint1<VectorConfig> c1(*test1::g, "x", *test1::G, p, "L_x1");

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
	NonlinearConstraint1<VectorConfig>
		c1(*test1::g, "x", *test1::G, 1, "L_x1", true),
		c2(*test1::g, "x", *test1::G, 1, "L_x1"),
		c3(*test1::g, "x", *test1::G, 2, "L_x1"),
		c4(*test1::g, "y", *test1::G, 1, "L_x1");

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

	/** gradient for x, gradG(x,y) in x: 2x*/
	Matrix G1(const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, 2.0 * x);
	}

	/** gradient for y, gradG(x,y) in y: -1 */
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
	NonlinearConstraint2<VectorConfig> c1(
			*test2::g,
			"x", *test2::G1,
			"y", *test2::G2,
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
	NonlinearConstraint2<VectorConfig> c1(
			*test2::g,
			"x", *test2::G1,
			"y", *test2::G2,
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
	NonlinearConstraint2<VectorConfig>
		c1(*test2::g, "x", *test2::G1, "y", *test2::G2, 1, "L_xy"),
		c2(*test2::g, "x", *test2::G1, "y", *test2::G2, 1, "L_xy"),
		c3(*test2::g, "y", *test2::G1, "x", *test2::G2, 1, "L_xy"),
		c4(*test2::g, "x", *test2::G1, "z", *test2::G2, 3, "L_xy");

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

	/** p = 1, gradG(x) = 2*x */
	Matrix G(const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, 2 * x);
	}

} // \namespace inequality1

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_inequality ) {
	size_t p = 1;
	NonlinearConstraint1<VectorConfig> c1(*inequality1::g,
										  "x", *inequality1::G,
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
	NonlinearConstraint1<VectorConfig> c1(*inequality1::g,
										  "x", *inequality1::G,
										   p, "L_x",
										  false); // inequality constraint

	// get configurations to use for linearization
	VectorConfig config1, config2;
	config1.insert("x", Vector_(1, 10.0)); // should have zero error
	config2.insert("x", Vector_(1, 1.0)); // should have nonzero error

	// get a configuration of Lagrange multipliers
	VectorConfig lagrangeConfig;
	lagrangeConfig.insert("L_x", Vector_(1, 3.0));

	// linearize for inactive constraint
	GaussianFactor::shared_ptr actualFactor1, actualConstraint1;
	boost::tie(actualFactor1, actualConstraint1) = c1.linearize(config1, lagrangeConfig);

	// check if the factualor is active
	CHECK(!c1.active(config1));

	// linearize for active constraint
	GaussianFactor::shared_ptr actualFactor2, actualConstraint2;
	boost::tie(actualFactor2, actualConstraint2) = c1.linearize(config2, lagrangeConfig);
	CHECK(c1.active(config2));

	// verify
	GaussianFactor expectedFactor("x", Matrix_(1,1, 6.0), "L_x", eye(1), zero(1), 1.0);
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

	/** p = 1, gradG(x) = 2*x */
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
	NonlinearConstraint1<VectorConfig> c1(
										  boost::bind(binding1::g, radius, _1, _2),
										  "x", boost::bind(binding1::G, coeff, _1, _2),
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

	/** gradient for x, gradG(x,y) in x: 2x*/
	Matrix G1(double c, const VectorConfig& config, const list<string>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, c * x);
	}

	/** gradient for y, gradG(x,y) in y: -1 */
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
	NonlinearConstraint2<VectorConfig> c1(
			boost::bind(binding2::g, r, _1, _2),
			"x", boost::bind(binding2::G1, a, _1, _2),
			"y", boost::bind(binding2::G2, b, _1, _2),
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
