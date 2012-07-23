/**
 * @file testLinearizedFactor
 * @author Alex Cunningham
 */

#include <boost/assign/std/vector.hpp>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam_unstable/nonlinear/LinearizedFactor.h>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

static const double tol = 1e-5;

/* ************************************************************************* */
TEST( testLinearizedFactor, creation ) {
	// Create a set of local keys (No robot label)
	Key	l1 = 11, l2 = 12,
			l3 = 13, l4 = 14,
			l5 = 15, l6 = 16,
			l7 = 17, l8 = 18;

	// creating an ordering to decode the linearized factor
	Ordering ordering;
	ordering += l1,l2,l3,l4,l5,l6,l7,l8;

	// create a decoder for only the relevant variables
	LinearizedFactor::KeyLookup decoder;
	decoder[2] = l3;
	decoder[4] = l5;

	// create a linear factor
	SharedDiagonal model = noiseModel::Unit::Create(2);
	JacobianFactor::shared_ptr linear_factor(new JacobianFactor(
			ordering[l3], eye(2,2), ordering[l5], 2.0 * eye(2,2), zero(2), model));

	// create a set of values - build with full set of values
	gtsam::Values full_values, exp_values;
	full_values.insert(l3, Point2(1.0, 2.0));
	full_values.insert(l5, Point2(4.0, 3.0));
	exp_values = full_values;
	full_values.insert(l1, Point2(3.0, 7.0));

	// create the test LinearizedFactor
	// This is called in the constructor of DDFSlot for each linear factor
	LinearizedFactor actual1(linear_factor, decoder, full_values);
	LinearizedFactor actual2(linear_factor, ordering, full_values);

	EXPECT(assert_equal(actual1, actual2, tol));

	// Verify the keys
	vector<gtsam::Key> expKeys;
	expKeys += l3, l5;
	EXPECT(assert_container_equality(expKeys, actual1.keys()));
	EXPECT(assert_container_equality(expKeys, actual2.keys()));

	// Verify subset of linearization points
	EXPECT(assert_equal(exp_values, actual1.linearizationPoint(), tol));
	EXPECT(assert_equal(exp_values, actual2.linearizationPoint(), tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
