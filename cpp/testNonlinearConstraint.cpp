/**
 * @file testNonlinearConstraint.cpp
 * @brief Tests for nonlinear constraints handled via SQP
 * @author Alex Cunningham
 */

#include <list>
#include <boost/bind.hpp>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/list.hpp> // for operator +=

#define GTSAM_MAGIC_KEY

#include <VectorConfig.h>
#include <NonlinearConstraint.h>
#include <NonlinearConstraint-inl.h>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

typedef TypedSymbol<Vector, 'x'> Key;
typedef NonlinearConstraint1<VectorConfig, Key, Vector> NLC1;
typedef NonlinearConstraint2<VectorConfig, Key, Vector, Key, Vector> NLC2;

/* ************************************************************************* */
// unary functions with scalar variables
/* ************************************************************************* */
namespace test1 {

	/** p = 1, g(x) = x^2-5 = 0 */
	Vector g(const VectorConfig& config, const list<Symbol>& keys) {
		double x = config[keys.front()](0);
		return Vector_(1, x * x - 5);
	}

	/** p = 1, jacobianG(x) = 2x */
	Matrix G(const VectorConfig& config, const list<Symbol>& keys) {
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
	list<Symbol> keys;	keys += "x1";
	Key x1(1);
	LagrangeKey L1(1);
	NLC1 c1(boost::bind(test1::g, _1, keys),
			x1, boost::bind(test1::G, _1, keys),
			p, L1);

	// get a configuration to use for finding the error
	VectorConfig config;
	config.insert("x1", Vector_(1, 1.0));

	// calculate the error
	Vector actual = c1.unwhitenedError(config);
	Vector expected = Vector_(1, -4.0);
	CHECK(assert_equal(actual, expected, 1e-5));
}

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_scalar_linearize ) {
	size_t p = 1;
	list<Symbol> keys;	keys += "x1";
	Key x1(1);
	LagrangeKey L1(1);
	NLC1 c1(boost::bind(test1::g, _1, keys),
			x1, boost::bind(test1::G, _1, keys),
			p, L1);

	// get a configuration to use for linearization (with lagrange multipliers)
	VectorConfig realconfig;
	realconfig.insert(x1, Vector_(1, 1.0));
	realconfig.insert(L1, Vector_(1, 3.0));

	// linearize the system
	GaussianFactor::shared_ptr linfactor = c1.linearize(realconfig);

	// verify - probabilistic component goes on top
	Vector sigmas = Vector_(2, 1.0, 0.0);
	SharedDiagonal mixedModel = noiseModel::Constrained::MixedSigmas(sigmas);
	// stack the matrices to combine
	Matrix Ax1 = Matrix_(2,1, 6.0, 2.0),
		   AL1 = Matrix_(2,1, 1.0, 0.0);
	Vector rhs = Vector_(2, 0.0, 4.0);
	GaussianFactor expectedFactor(x1, Ax1, L1, AL1, rhs, mixedModel);

	CHECK(assert_equal(*linfactor, expectedFactor));
}

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_scalar_equal ) {
	list<Symbol> keys1, keys2; keys1 += "x0"; keys2 += "x1";
	Key x(0), y(1);
	LagrangeKey L1(1);
	NLC1
		c1(boost::bind(test1::g, _1, keys1), x, boost::bind(test1::G, _1, keys1), 1, L1, true),
		c2(boost::bind(test1::g, _1, keys1), x, boost::bind(test1::G, _1, keys1), 1, L1),
		c3(boost::bind(test1::g, _1, keys1), x, boost::bind(test1::G, _1, keys1), 2, L1),
		c4(boost::bind(test1::g, _1, keys2), y, boost::bind(test1::G, _1, keys2), 1, L1);

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
	Vector g(const VectorConfig& config, const list<Symbol>& keys) {
		double x = config[keys.front()](0);
		double y = config[keys.back()](0);
		return Vector_(1, x * x - 5.0 - y);
	}

	/** jacobian for x, jacobianG(x,y) in x: 2x*/
	Matrix G1(const VectorConfig& config, const list<Symbol>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, 2.0 * x);
	}

	/** jacobian for y, jacobianG(x,y) in y: -1 */
	Matrix G2(const VectorConfig& config, const list<Symbol>& keys) {
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
	list<Symbol> keys; keys += "x0", "x1";
	Key x0(0), x1(1);
	LagrangeKey L1(1);
	NLC2 c1(
			boost::bind(test2::g, _1, keys),
			x0, boost::bind(test2::G1, _1, keys),
			x1, boost::bind(test2::G1, _1, keys),
			p, L1);

	// get a configuration to use for finding the error
	VectorConfig config;
	config.insert("x0", Vector_(1, 1.0));
	config.insert("x1", Vector_(1, 2.0));

	// calculate the error
	Vector actual = c1.unwhitenedError(config);
	Vector expected = Vector_(1.0, -6.0);
	CHECK(assert_equal(actual, expected, 1e-5));
}

/* ************************************************************************* */
TEST( NonlinearConstraint2, binary_scalar_linearize ) {
	// create a constraint
	size_t p = 1;
	list<Symbol> keys; keys += "x0", "x1";
	Key x0(0), x1(1);
	LagrangeKey L1(1);
	NLC2 c1(
			boost::bind(test2::g, _1, keys),
			x0, boost::bind(test2::G1, _1, keys),
			x1, boost::bind(test2::G2, _1, keys),
			p, L1);

	// get a configuration to use for finding the error
	VectorConfig realconfig;
	realconfig.insert(x0, Vector_(1, 1.0));
	realconfig.insert(x1, Vector_(1, 2.0));
	realconfig.insert(L1, Vector_(1, 3.0));

	// linearize the system
	GaussianFactor::shared_ptr actualFactor = c1.linearize(realconfig);

	// verify
	Matrix Ax0 = Matrix_(2,1, 6.0, 2.0),
		   Ax1 = Matrix_(2,1,-3.0,-1.0),
		   AL  = Matrix_(2,1, 1.0, 0.0);
	Vector rhs = Vector_(2, 0, 6.0),
		   sigmas = Vector_(2, 1.0, 0.0);
	SharedDiagonal expModel = noiseModel::Constrained::MixedSigmas(sigmas);

//	SharedDiagonal probModel = sharedSigma(p,1.0);
//	GaussianFactor expectedFactor(x0, Matrix_(1,1, 6.0),
//							 	  x1, Matrix_(1,1, -3.0),
//							      L1, eye(1), zero(1), probModel);
//	SharedDiagonal constraintModel = noiseModel::Constrained::All(p);
//	GaussianFactor expectedConstraint(x0, Matrix_(1,1, 2.0),
//								      x1, Matrix_(1,1, -1.0),
//								       Vector_(1, 6.0), constraintModel);
//	CHECK(assert_equal(*actualFactor, expectedFactor));
//	CHECK(assert_equal(*actualConstraint, expectedConstraint));
}

/* ************************************************************************* */
TEST( NonlinearConstraint2, binary_scalar_equal ) {
	list<Symbol> keys1, keys2, keys3;
	keys1 += "x0", "x1"; keys2 += "x1", "x0"; keys3 += "x0", "z";
	Key x0(0), x1(1), x2(2);
	LagrangeKey L1(1);
	NLC2
		c1(boost::bind(test2::g, _1, keys1), x0, boost::bind(test2::G1, _1, keys1), x1, boost::bind(test2::G2, _1, keys1), 1, L1),
		c2(boost::bind(test2::g, _1, keys1), x0, boost::bind(test2::G1, _1, keys1), x1, boost::bind(test2::G2, _1, keys1), 1, L1),
		c3(boost::bind(test2::g, _1, keys2), x1, boost::bind(test2::G1, _1, keys2), x0, boost::bind(test2::G2, _1, keys2), 1, L1),
		c4(boost::bind(test2::g, _1, keys3), x0, boost::bind(test2::G1, _1, keys3), x2, boost::bind(test2::G2, _1, keys3), 3, L1);

	CHECK(assert_equal(c1, c2));
	CHECK(assert_equal(c2, c1));
	CHECK(!c1.equals(c3));
	CHECK(!c1.equals(c4));
}

///* ************************************************************************* */
//// Inequality tests
///* ************************************************************************* */
//namespace inequality1 {
//
//	/** p = 1, g(x) x^2 - 5 > 0 */
//	Vector g(const VectorConfig& config, const Key& key) {
//		double x = config[key](0);
//		double g = x * x - 5;
//		return Vector_(1, g); // return the actual cost
//	}
//
//	/** p = 1, jacobianG(x) = 2*x */
//	Matrix G(const VectorConfig& config, const Key& key) {
//		double x = config[key](0);
//		return Matrix_(1, 1, 2 * x);
//	}
//
//} // \namespace inequality1
//
///* ************************************************************************* */
//TEST( NonlinearConstraint1, unary_inequality ) {
//	size_t p = 1;
//	Key x0(0);
//	NLC1 c1(boost::bind(inequality1::g, _1, x0),
//			x0, boost::bind(inequality1::G, _1, x0),
//			p, "L1",
//			false); // inequality constraint
//
//	// get configurations to use for evaluation
//	VectorConfig config1, config2;
//	config1.insert(x0, Vector_(1, 10.0)); // should be inactive
//	config2.insert(x0, Vector_(1, 1.0)); // should have nonzero error
//
//	// check error
//	CHECK(!c1.active(config1));
//	Vector actualError2 = c1.unwhitenedError(config2);
//	CHECK(assert_equal(actualError2, Vector_(1, -4.0, 1e-9)));
//	CHECK(c1.active(config2));
//}
//
///* ************************************************************************* */
//TEST( NonlinearConstraint1, unary_inequality_linearize ) {
//	size_t p = 1;
//	Key x0(0);
//	NLC1 c1(boost::bind(inequality1::g, _1, x0),
//			x0, boost::bind(inequality1::G, _1, x0),
//			p, "L1",
//			false); // inequality constraint
//
//	// get configurations to use for linearization
//	VectorConfig config1, config2;
//	config1.insert(x0, Vector_(1, 10.0)); // should have zero error
//	config2.insert(x0, Vector_(1, 1.0)); // should have nonzero error
//
//	// get a configuration of Lagrange multipliers
//	VectorConfig lagrangeConfig;
//	lagrangeConfig.insert("L1", Vector_(1, 3.0));
//
//	// linearize for inactive constraint
//	GaussianFactor::shared_ptr actualFactor1, actualConstraint1;
//	boost::tie(actualFactor1, actualConstraint1) = c1.linearize(config1, lagrangeConfig);
//
//	// check if the factor is active
//	CHECK(!c1.active(config1));
//
//	// linearize for active constraint
//	GaussianFactor::shared_ptr actualFactor2, actualConstraint2;
//	boost::tie(actualFactor2, actualConstraint2) = c1.linearize(config2, lagrangeConfig);
//	CHECK(c1.active(config2));
//
//	// verify
//	SharedDiagonal probModel = sharedSigma(p,1.0);
//	GaussianFactor expectedFactor(x0, Matrix_(1,1, 6.0), "L1", eye(1), zero(1), probModel);
//	SharedDiagonal constraintModel = noiseModel::Constrained::All(p);
//	GaussianFactor expectedConstraint(x0, Matrix_(1,1, 2.0), Vector_(1, 4.0), constraintModel);
//	CHECK(assert_equal(*actualFactor2, expectedFactor));
//	CHECK(assert_equal(*actualConstraint2, expectedConstraint));
//}
//
///* ************************************************************************* */
//// Binding arbitrary functions
///* ************************************************************************* */
//namespace binding1 {
//
//	/** p = 1, g(x) x^2 - r > 0 */
//	Vector g(double r, const VectorConfig& config, const Key& key) {
//		double x = config[key](0);
//		double g = x * x - r;
//		return Vector_(1, g); // return the actual cost
//	}
//
//	/** p = 1, jacobianG(x) = 2*x */
//	Matrix G(double coeff, const VectorConfig& config,
//			const Key& key) {
//		double x = config[key](0);
//		return Matrix_(1, 1, coeff * x);
//	}
//
//} // \namespace binding1
//
///* ************************************************************************* */
//TEST( NonlinearConstraint1, unary_binding ) {
//	size_t p = 1;
//	double coeff = 2;
//	double radius = 5;
//	Key x0(0);
//	NLC1 c1(boost::bind(binding1::g, radius, _1, x0),
//			x0, boost::bind(binding1::G, coeff, _1, x0),
//			p, "L1",
//			false); // inequality constraint
//
//	// get configurations to use for evaluation
//	VectorConfig config1, config2;
//	config1.insert(x0, Vector_(1, 10.0)); // should have zero error
//	config2.insert(x0, Vector_(1, 1.0)); // should have nonzero error
//
//	// check error
//	CHECK(!c1.active(config1));
//	Vector actualError2 = c1.unwhitenedError(config2);
//	CHECK(assert_equal(actualError2, Vector_(1, -4.0, 1e-9)));
//	CHECK(c1.active(config2));
//}
//
///* ************************************************************************* */
//namespace binding2 {
//
//	/** p = 1, g(x) = x^2-5 -y = 0 */
//	Vector g(double r, const VectorConfig& config, const Key& k1, const Key& k2) {
//		double x = config[k1](0);
//		double y = config[k2](0);
//		return Vector_(1, x * x - r - y);
//	}
//
//	/** jacobian for x, jacobianG(x,y) in x: 2x*/
//	Matrix G1(double c, const VectorConfig& config, const Key& key) {
//		double x = config[key](0);
//		return Matrix_(1, 1, c * x);
//	}
//
//	/** jacobian for y, jacobianG(x,y) in y: -1 */
//	Matrix G2(double c, const VectorConfig& config) {
//		return Matrix_(1, 1, -1.0 * c);
//	}
//
//} // \namespace binding2
//
///* ************************************************************************* */
//TEST( NonlinearConstraint2, binary_binding ) {
//	// construct a constraint on x and y
//	// the lagrange multipliers will be expected on L_xy
//	// and there is only one multiplier
//	size_t p = 1;
//	double a = 2.0;
//	double b = 1.0;
//	double r = 5.0;
//	Key x0(0), x1(1);
//	NLC2 c1(boost::bind(binding2::g, r, _1, x0, x1),
//			x0, boost::bind(binding2::G1, a, _1, x0),
//			x1, boost::bind(binding2::G2, b, _1),
//			p, "L1");
//
//	// get a configuration to use for finding the error
//	VectorConfig config;
//	config.insert(x0, Vector_(1, 1.0));
//	config.insert(x1, Vector_(1, 2.0));
//
//	// calculate the error
//	Vector actual = c1.unwhitenedError(config);
//	Vector expected = Vector_(1.0, -6.0);
//	CHECK(assert_equal(actual, expected, 1e-5));
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
