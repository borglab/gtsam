/**
 *  @file  testPose2Prior.cpp
 *  @brief Unit tests for Pose2Prior Class
 *  @authors Frank Dellaert, Viorela Ila
 **/

#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "pose2SLAM.h"

using namespace std;
using namespace gtsam;

// Common measurement covariance
static double sx=0.5, sy=0.5,st=0.1;
static GaussianNoiseModel::shared_ptr model = Diagonal::Sigmas(Vector_(3,sx,sy,st));

/* ************************************************************************* */
// Very simple test establishing Ax-b \approx z-h(x)
TEST( Pose2Prior, error )
{
	// Choose a linearization point
	Pose2 p1(1, 0, 0); // robot at (1,0)
	Pose2Config x0;
	x0.insert(1, p1);

	// Create factor
	Pose2Prior factor(1, p1, model);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> linear = factor.linearize(x0);

	// Check error at x0, i.e. delta = zero !
	VectorConfig delta;
	delta.insert("x1", zero(3));
	Vector error_at_zero = Vector_(3,0.0,0.0,0.0);
	CHECK(assert_equal(error_at_zero,factor.error_vector(x0)));
	CHECK(assert_equal(-error_at_zero,linear->error_vector(delta)));

	// Check error after increasing p2
	VectorConfig plus = delta + VectorConfig("x1", Vector_(3, 0.1, 0.0, 0.0));
	Pose2Config x1 = expmap(x0, plus);
	Vector error_at_plus = Vector_(3,0.1/sx,0.0,0.0); // h(x)-z = 0.1 !
	CHECK(assert_equal(error_at_plus,factor.error_vector(x1)));
	CHECK(assert_equal(error_at_plus,linear->error_vector(plus)));
}

/* ************************************************************************* */
// common Pose2Prior for tests below
static Pose2 prior(2,2,M_PI_2);
static Pose2Prior factor(1,prior, model);

/* ************************************************************************* */
// The error |A*dx-b| approximates (h(x0+dx)-z) = -error_vector
// Hence i.e., b = approximates z-h(x0) = error_vector(x0)
Vector h(const Pose2& p1) {
	return factor.evaluateError(p1);
}

/* ************************************************************************* */
TEST( Pose2Prior, linearize )
{
	// Choose a linearization point at ground truth
	Pose2Config x0;
	x0.insert(1,prior);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> actual = factor.linearize(x0);

	// Test with numerical derivative
	Matrix numericalH = numericalDerivative11(h, prior, 1e-5);
	CHECK(assert_equal(numericalH,actual->get_A("x1")));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
