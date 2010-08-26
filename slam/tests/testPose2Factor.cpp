/**
 *  @file  testPose2Factor.cpp
 *  @brief Unit tests for Pose2Factor Class
 *  @authors Frank Dellaert, Viorela Ila
 **/

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/base/LieVector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/slam/pose2SLAM.h>

using namespace std;
using namespace gtsam;

// Common measurement covariance
static double sx=0.5, sy=0.5,st=0.1;
static noiseModel::Gaussian::shared_ptr covariance(
		noiseModel::Gaussian::Covariance(Matrix_(3, 3,
	sx*sx, 0.0, 0.0,
	0.0, sy*sy, 0.0,
	0.0, 0.0, st*st
	)));

/* ************************************************************************* */
// Very simple test establishing Ax-b \approx z-h(x)
TEST( Pose2Factor, error )
{
	// Choose a linearization point
	Pose2 p1; // robot at origin
	Pose2 p2(1, 0, 0); // robot at (1,0)
	Pose2Config x0;
	x0.insert(1, p1);
	x0.insert(2, p2);

	// Create factor
	Pose2 z = p1.between(p2);
	Pose2Factor factor(1, 2, z, covariance);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> linear = factor.linearize(x0);

	// Check error at x0, i.e. delta = zero !
	VectorConfig delta;
	delta.insert("x1", zero(3));
	delta.insert("x2", zero(3));
	Vector error_at_zero = Vector_(3,0.0,0.0,0.0);
	CHECK(assert_equal(error_at_zero,factor.unwhitenedError(x0)));
	CHECK(assert_equal(-error_at_zero,linear->error_vector(delta)));

	// Check error after increasing p2
	VectorConfig plus = delta;
	plus.insertAdd("x2", Vector_(3, 0.1, 0.0, 0.0));
	Pose2Config x1 = x0.expmap(plus);
	Vector error_at_plus = Vector_(3,0.1/sx,0.0,0.0); // h(x)-z = 0.1 !
	CHECK(assert_equal(error_at_plus,factor.whitenedError(x1)));
	CHECK(assert_equal(error_at_plus,linear->error_vector(plus)));
}

/* ************************************************************************* */
// common Pose2Factor for tests below
static Pose2 measured(2,2,M_PI_2);
static Pose2Factor factor(1,2,measured, covariance);

/* ************************************************************************* */
TEST( Pose2Factor, rhs )
{
	// Choose a linearization point
	Pose2 p1(1.1,2,M_PI_2); // robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
	Pose2 p2(-1,4.1,M_PI);  // robot at (-1,4.1) looking at negative (ground truth is at -1,4)
	Pose2Config x0;
	x0.insert(1,p1);
	x0.insert(2,p2);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> linear = factor.linearize(x0);

	// Check RHS
	Pose2 hx0 = p1.between(p2);
	CHECK(assert_equal(Pose2(2.1, 2.1, M_PI_2),hx0));
	Vector expected_b = Vector_(3, -0.1/sx, 0.1/sy, 0.0);
	CHECK(assert_equal(expected_b,-factor.whitenedError(x0)));
	CHECK(assert_equal(expected_b,linear->get_b()));
}

/* ************************************************************************* */
// The error |A*dx-b| approximates (h(x0+dx)-z) = -error_vector
// Hence i.e., b = approximates z-h(x0) = error_vector(x0)
LieVector h(const Pose2& p1,const Pose2& p2) {
	return LieVector(covariance->whiten(factor.evaluateError(p1,p2)));
}

/* ************************************************************************* */
TEST( Pose2Factor, linearize )
{
	// Choose a linearization point at ground truth
	Pose2 p1(1,2,M_PI_2);
	Pose2 p2(-1,4,M_PI);
	Pose2Config x0;
	x0.insert(1,p1);
	x0.insert(2,p2);

	// expected linearization
	Matrix expectedH1 = covariance->Whiten(Matrix_(3,3,
	    0.0,-1.0,-2.0,
	    1.0, 0.0,-2.0,
	    0.0, 0.0,-1.0
	));
	Matrix expectedH2 = covariance->Whiten(Matrix_(3,3,
	    1.0, 0.0, 0.0,
	    0.0, 1.0, 0.0,
	    0.0, 0.0, 1.0
	));
	Vector expected_b = Vector_(3, 0.0, 0.0, 0.0);

	// expected linear factor
	SharedDiagonal probModel1 = noiseModel::Unit::Create(3);
	GaussianFactor expected("x1", expectedH1, "x2", expectedH2, expected_b, probModel1);

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
