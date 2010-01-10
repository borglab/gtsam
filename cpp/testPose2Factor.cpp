/**
 *  @file  testPose2Factor.cpp
 *  @brief Unit tests for Pose2Factor Class
 *  @authors Frank Dellaert, Viorela Ila
 **/

#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "Pose2Factor.h"

using namespace std;
using namespace gtsam;

// Common measurement covariance
static double sx=0.5, sy=0.5,st=0.1;
static Matrix covariance = Matrix_(3,3,
		sx*sx, 0.0, 0.0,
		0.0, sy*sy, 0.0,
		0.0, 0.0, st*st
		);

/* ************************************************************************* */
// Very simple test establishing Ax-b \approx h(x)-z
TEST( Pose2Factor, error )
{
	// Choose a linearization point
	Pose2 p1; // robot at origin
	Pose2 p2(1, 0, 0); // robot at (1,0)
	Pose2Config x0;
	x0.insert("p1", p1);
	x0.insert("p2", p2);

	// Create factor
	Pose2 z = between(p1,p2);
	Pose2Factor factor("p1", "p2", z, covariance);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> linear = factor.linearize(x0);

	// Check error at x0 = zero !
	VectorConfig delta;
	delta.insert("p1", zero(3));
	delta.insert("p2", zero(3));
	Vector error_at_zero = Vector_(3,0.0,0.0,0.0);
	CHECK(assert_equal(error_at_zero,factor.error_vector(x0)));
	CHECK(assert_equal(-error_at_zero,linear->error_vector(delta)));

	// Check error after increasing p2
	VectorConfig plus = delta + VectorConfig("p2", Vector_(3, 0.1, 0.0, 0.0));
	Pose2Config x1 = expmap(x0, plus);
	Vector error_at_plus = Vector_(3,-0.1/sx,0.0,0.0);
	CHECK(assert_equal(error_at_plus,factor.error_vector(x1)));
	CHECK(assert_equal(-error_at_plus,linear->error_vector(plus)));
}

/* ************************************************************************* */
// common Pose2Factor for tests below
static Pose2 measured(2,2,M_PI_2);
static Pose2Factor factor("p1","p2",measured, covariance);

/* ************************************************************************* */
TEST( Pose2Factor, rhs )
{
	// Choose a linearization point
	Pose2 p1(1.1,2,M_PI_2); // robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
	Pose2 p2(-1,4.1,M_PI);  // robot at (-1,4.1) looking at negative (ground truth is at -1,4)
	Pose2Config x0;
	x0.insert("p1",p1);
	x0.insert("p2",p2);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> linear = factor.linearize(x0);

	// Check RHS
	Pose2 hx0 = between(p1,p2);
	CHECK(assert_equal(Pose2(2.1, 2.1, M_PI_2),hx0));
	Vector expected_b = Vector_(3, -0.1/sx, 0.1/sy, 0.0);
	CHECK(assert_equal(expected_b,factor.error_vector(x0)));
	CHECK(assert_equal(expected_b,linear->get_b()));
}

/* ************************************************************************* */
// The error |A*dx-b| approximates (h(x0+dx)-z) = -error_vector
// Hence i.e., b = approximates z-h(x0) = error_vector(x0)
Vector h(const Pose2& p1,const Pose2& p2) {
	Pose2Config x;
	x.insert("p1",p1);
	x.insert("p2",p2);
	return -factor.error_vector(x);
}

/* ************************************************************************* */
TEST( Pose2Factor, linearize )
{
	// Choose a linearization point at ground truth
	Pose2 p1(1,2,M_PI_2);
	Pose2 p2(-1,4,M_PI);
	Pose2Config x0;
	x0.insert("p1",p1);
	x0.insert("p2",p2);

	// expected linearization
	Matrix square_root_inverse_covariance = Matrix_(3,3,
	    2.0, 0.0, 0.0,
	    0.0, 2.0, 0.0,
	    0.0, 0.0, 10.0
	);
	Matrix expectedH1 = square_root_inverse_covariance*Matrix_(3,3,
	    0.0,-1.0,-2.0,
	    1.0, 0.0,-2.0,
	    0.0, 0.0,-1.0
	);
	Matrix expectedH2 = square_root_inverse_covariance*Matrix_(3,3,
	    1.0, 0.0, 0.0,
	    0.0, 1.0, 0.0,
	    0.0, 0.0, 1.0
	);
	Vector expected_b = Vector_(3, 0.0, 0.0, 0.0);

	// expected linear factor
	GaussianFactor expected("p1", expectedH1, "p2", expectedH2, expected_b, 1.0);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> actual = factor.linearize(x0);
	CHECK(assert_equal(expected,*actual));

	// Numerical do not work out because BetweenFactor is approximate ?
	Matrix numericalH1 = numericalDerivative21(h, p1, p2, 1e-5);
	CHECK(assert_equal(expectedH1,numericalH1));
	Matrix numericalH2 = numericalDerivative22(h, p1, p2, 1e-5);
	CHECK(assert_equal(expectedH2,numericalH2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
