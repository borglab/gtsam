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
#include <TupleConfig-inl.h>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

typedef TypedSymbol<Vector, 'x'> Key;
typedef LieConfig<Key, Vector> VecConfig;
typedef NonlinearConstraint1<VecConfig, Key, Vector> NLC1;
typedef NonlinearConstraint2<VecConfig, Key, Vector, Key, Vector> NLC2;

/* ************************************************************************* */
// unary functions with scalar variables
/* ************************************************************************* */
namespace test1 {

	/** p = 1, g(x) = x^2-5 = 0 */
	Vector g(const VecConfig& config, const list<Key>& keys) {
		double x = config[keys.front()](0);
		return Vector_(1, x * x - 5);
	}

	/** p = 1, jacobianG(x) = 2x */
	Matrix G(const VecConfig& config, const list<Key>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, 2 * x);
	}

} // \namespace test1

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_scalar_construction ) {
	// construct a constraint on x
	size_t p = 1;
	Key x1(1);
	list<Key> keys;	keys += x1;
	NLC1 c1(boost::bind(test1::g, _1, keys),
			x1, boost::bind(test1::G, _1, keys),
			p, 10.0);

	// get a configuration to use for finding the error
	VecConfig config;
	config.insert(x1, Vector_(1, 1.0));

	// calculate the error
	Vector actualVec = c1.unwhitenedError(config);
	Vector expectedVec = Vector_(1, -4.0);
	CHECK(assert_equal(actualVec, expectedVec, 1e-5));

	double actError = c1.error(config);
	double expError = 160.0;
	DOUBLES_EQUAL(expError, actError, 1e-5);
}

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_scalar_linearize ) {
	size_t p = 1;
	Key x1(1);
	list<Key> keys;	keys += x1;
	NLC1 c1(boost::bind(test1::g, _1, keys),
			x1, boost::bind(test1::G, _1, keys),
			p);

	// get a configuration to use for linearization
	VecConfig realconfig;
	realconfig.insert(x1, Vector_(1, 1.0));

	// linearize the system
	GaussianFactor::shared_ptr linfactor = c1.linearize(realconfig);

	// verify
	SharedDiagonal model = noiseModel::Constrained::All(p);
	Matrix Ax1 = Matrix_(p,1, 2.0);
	Vector rhs = Vector_(p, 4.0);
	GaussianFactor expectedFactor(x1, Ax1, rhs, model);

	CHECK(assert_equal(*linfactor, expectedFactor));
}

/* ************************************************************************* */
TEST( NonlinearConstraint1, unary_scalar_equal ) {
	Key x(0), y(1);
	list<Key> keys1, keys2; keys1 += x; keys2 += y;
	NLC1
		c1(boost::bind(test1::g, _1, keys1), x, boost::bind(test1::G, _1, keys1), 1),
		c2(boost::bind(test1::g, _1, keys1), x, boost::bind(test1::G, _1, keys1), 1),
		c3(boost::bind(test1::g, _1, keys1), x, boost::bind(test1::G, _1, keys1), 2),
		c4(boost::bind(test1::g, _1, keys2), y, boost::bind(test1::G, _1, keys2), 1);

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
	Vector g(const VecConfig& config, const list<Key>& keys) {
		double x = config[keys.front()](0);
		double y = config[keys.back()](0);
		return Vector_(1, x * x - 5.0 - y);
	}

	/** jacobian for x, jacobianG(x,y) in x: 2x*/
	Matrix G1(const VecConfig& config, const list<Key>& keys) {
		double x = config[keys.front()](0);
		return Matrix_(1, 1, 2.0 * x);
	}

	/** jacobian for y, jacobianG(x,y) in y: -1 */
	Matrix G2(const VecConfig& config, const list<Key>& keys) {
		return Matrix_(1, 1, -1.0);
	}

} // \namespace test2

/* ************************************************************************* */
TEST( NonlinearConstraint2, binary_scalar_construction ) {
	// construct a constraint on x and y
	size_t p = 1;
	Key x0(0), x1(1);
	list<Key> keys; keys += x0, x1;
	NLC2 c1(
			boost::bind(test2::g, _1, keys),
			x0, boost::bind(test2::G1, _1, keys),
			x1, boost::bind(test2::G1, _1, keys),
			p);

	// get a configuration to use for finding the error
	VecConfig config;
	config.insert(x0, Vector_(1, 1.0));
	config.insert(x1, Vector_(1, 2.0));

	// calculate the error
	Vector actual = c1.unwhitenedError(config);
	Vector expected = Vector_(1.0, -6.0);
	CHECK(assert_equal(actual, expected, 1e-5));
}

/* ************************************************************************* */
TEST( NonlinearConstraint2, binary_scalar_linearize ) {
	// create a constraint
	size_t p = 1;
	Key x0(0), x1(1);
	list<Key> keys; keys += x0, x1;
	NLC2 c1(
			boost::bind(test2::g, _1, keys),
			x0, boost::bind(test2::G1, _1, keys),
			x1, boost::bind(test2::G2, _1, keys),
			p);

	// get a configuration to use for finding the error
	VecConfig realconfig;
	realconfig.insert(x0, Vector_(1, 1.0));
	realconfig.insert(x1, Vector_(1, 2.0));

	// linearize the system
	GaussianFactor::shared_ptr actualFactor = c1.linearize(realconfig);

	// verify
	Matrix Ax0 = Matrix_(1,1, 2.0),
		   Ax1 = Matrix_(1,1,-1.0);
	Vector rhs = Vector_(1, 6.0);
	SharedDiagonal expModel = noiseModel::Constrained::All(p);
	GaussianFactor expFactor(x0,Ax0, x1, Ax1,rhs, expModel);
	CHECK(assert_equal(expFactor, *actualFactor));
}

/* ************************************************************************* */
TEST( NonlinearConstraint2, binary_scalar_equal ) {
	list<Key> keys1, keys2, keys3;
	Key x0(0), x1(1), x2(2);
	keys1 += x0, x1; keys2 += x1, x0; keys3 += x0;
	NLC2
		c1(boost::bind(test2::g, _1, keys1), x0, boost::bind(test2::G1, _1, keys1), x1, boost::bind(test2::G2, _1, keys1), 1),
		c2(boost::bind(test2::g, _1, keys1), x0, boost::bind(test2::G1, _1, keys1), x1, boost::bind(test2::G2, _1, keys1), 1),
		c3(boost::bind(test2::g, _1, keys2), x1, boost::bind(test2::G1, _1, keys2), x0, boost::bind(test2::G2, _1, keys2), 1),
		c4(boost::bind(test2::g, _1, keys3), x0, boost::bind(test2::G1, _1, keys3), x2, boost::bind(test2::G2, _1, keys3), 3);

	CHECK(assert_equal(c1, c2));
	CHECK(assert_equal(c2, c1));
	CHECK(!c1.equals(c3));
	CHECK(!c1.equals(c4));
}

/* ************************************************************************* */
// Inequality tests - DISABLED
/* ************************************************************************* */
//namespace inequality1 {
//
//	/** p = 1, g(x) x^2 - 5 > 0 */
//	Vector g(const VecConfig& config, const Key& key) {
//		double x = config[key](0);
//		double g = x * x - 5;
//		return Vector_(1, g); // return the actual cost
//	}
//
//	/** p = 1, jacobianG(x) = 2*x */
//	Matrix G(const VecConfig& config, const Key& key) {
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
//			p, false); // inequality constraint
//
//	// get configurations to use for evaluation
//	VecConfig config1, config2;
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
//			p, false); // inequality constraint
//
//	// get configurations to use for linearization
//	VecConfig config1, config2;
//	config1.insert(x0, Vector_(1, 10.0)); // should have zero error
//	config2.insert(x0, Vector_(1, 1.0)); // should have nonzero error
//
//	// linearize for inactive constraint
//	GaussianFactor::shared_ptr actualFactor1 = c1.linearize(config1);
//
//	// check if the factor is active
//	CHECK(!c1.active(config1));
//
//	// linearize for active constraint
//	GaussianFactor::shared_ptr actualFactor2 = c1.linearize(config2);
//	CHECK(c1.active(config2));
//
//	// verify
//	Vector sigmas = Vector_(2, 1.0, 0.0);
//	SharedDiagonal model = noiseModel::Constrained::MixedSigmas(sigmas);
//	GaussianFactor expectedFactor(x0, Matrix_(2,1, 6.0, 2.0),
//								  L1, Matrix_(2,1, 1.0, 0.0),
//								  Vector_(2, 0.0, 4.0), model);
//
//	CHECK(assert_equal(*actualFactor2, expectedFactor));
//}

/* ************************************************************************* */
// Binding arbitrary functions
/* ************************************************************************* */
//namespace binding1 {
//
//	/** p = 1, g(x) x^2 - r > 0 */
//	Vector g(double r, const VecConfig& config, const Key& key) {
//		double x = config[key](0);
//		double g = x * x - r;
//		return Vector_(1, g); // return the actual cost
//	}
//
//	/** p = 1, jacobianG(x) = 2*x */
//	Matrix G(double coeff, const VecConfig& config,
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
//			p, false); // inequality constraint
//
//	// get configurations to use for evaluation
//	VecConfig config1, config2;
//	config1.insert(x0, Vector_(1, 10.0)); // should have zero error
//	config2.insert(x0, Vector_(1, 1.0)); // should have nonzero error
//
//	// check error
//	CHECK(!c1.active(config1));
//	Vector actualError2 = c1.unwhitenedError(config2);
//	CHECK(assert_equal(actualError2, Vector_(1, -4.0, 1e-9)));
//	CHECK(c1.active(config2));
//}

/* ************************************************************************* */
namespace binding2 {

	/** p = 1, g(x) = x^2-5 -y = 0 */
	Vector g(double r, const VecConfig& config, const Key& k1, const Key& k2) {
		double x = config[k1](0);
		double y = config[k2](0);
		return Vector_(1, x * x - r - y);
	}

	/** jacobian for x, jacobianG(x,y) in x: 2x*/
	Matrix G1(double c, const VecConfig& config, const Key& key) {
		double x = config[key](0);
		return Matrix_(1, 1, c * x);
	}

	/** jacobian for y, jacobianG(x,y) in y: -1 */
	Matrix G2(double c, const VecConfig& config) {
		return Matrix_(1, 1, -1.0 * c);
	}

} // \namespace binding2

/* ************************************************************************* */
TEST( NonlinearConstraint2, binary_binding ) {
	// construct a constraint on x and y
	size_t p = 1;
	double a = 2.0;
	double b = 1.0;
	double r = 5.0;
	Key x0(0), x1(1);
	NLC2 c1(boost::bind(binding2::g, r, _1, x0, x1),
			x0, boost::bind(binding2::G1, a, _1, x0),
			x1, boost::bind(binding2::G2, b, _1),
			p);

	// get a configuration to use for finding the error
	VecConfig config;
	config.insert(x0, Vector_(1, 1.0));
	config.insert(x1, Vector_(1, 2.0));

	// calculate the error
	Vector actual = c1.unwhitenedError(config);
	Vector expected = Vector_(1.0, -6.0);
	CHECK(assert_equal(actual, expected, 1e-5));
}



/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
